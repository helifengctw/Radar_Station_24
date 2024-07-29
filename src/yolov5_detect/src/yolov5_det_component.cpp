#include <utility>

#include "yolov5_detect_component.h"

namespace yolov5_detect {

    Yolov5Detector::Yolov5Detector(const rclcpp::NodeOptions &options) :
            Node("yolov5_detector", options),
            deepsort_(Deepsort_engine, 128, 256, 0, &gLogger_) {
        cudaSetDevice(kGpuId);

        if (FilePreparation(false, false, false,
                            &car_engine_name, &num_engine_name)) {
            RCLCPP_ERROR(this->get_logger(), "failed to prepare engine!!!!!");
        }

        // Deserialize the engine_car from file
        deserialize_engine(car_engine_name, &runtime, &car_engine, &context_car);
        CUDA_CHECK(cudaStreamCreate(&stream_car))
        // Init CUDA preprocessing
        cuda_preprocess_init(kMaxInputImageSize, true);
        // Prepare cpu and gpu buffers
        prepare_buffers(car_engine, &gpu_buffers_car[0],
                        &gpu_buffers_car[1], &cpu_output_buffer_car, true);

        // Deserialize the engine_num from file
        deserialize_engine(num_engine_name, &runtime, &num_engine, &context_num);
        CUDA_CHECK(cudaStreamCreate(&stream_num));
        // Init CUDA preprocessing
        cuda_preprocess_init(kMaxInputImageSize, false);
        // Prepare cpu and gpu buffers
        prepare_buffers(num_engine, &gpu_buffers_num[0],
                        &gpu_buffers_num[1], &cpu_output_buffer_num, false);

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "raw/image", 10, std::bind(&Yolov5Detector::ImageCallback, this, _1));

        rect_publisher_ = this->create_publisher<radar_interfaces::msg::YoloPoints>("rectangles", 1);
        yolo_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("yolo_detected_img", 1);
        deepsort_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("deep_sort_img", 1);

        load_parameters();
        if (show_by_cv_or_msg == 0) {
            win_name = camera_name + " yolo_show";
            cv::namedWindow(win_name);
            cv::startWindowThread();
        }
        pro_start_time = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        cost_matrix = std::vector<std::vector<int>>(20, std::vector<int>(20, 0));
        sorted_cost_matrix = std::vector<std::vector<int>>(20, std::vector<int>(20, 0));
    }

    Yolov5Detector::~Yolov5Detector() {
        // Release stream and buffers
        cudaStreamDestroy(stream_car);
        CUDA_CHECK(cudaFree(gpu_buffers_car[0]))
        CUDA_CHECK(cudaFree(gpu_buffers_car[1]))
        delete[] cpu_output_buffer_car;

        cudaStreamDestroy(stream_num);
        CUDA_CHECK(cudaFree(gpu_buffers_num[0]));
        CUDA_CHECK(cudaFree(gpu_buffers_num[1]));
        delete[] cpu_output_buffer_num;

        cuda_preprocess_destroy(true);
        cuda_preprocess_destroy(false);

        // Destroy the engine
        context_num->destroy();
        num_engine->destroy();

        context_car->destroy();
        car_engine->destroy();

        runtime->destroy();

        cv::destroyAllWindows();
    }

    void Yolov5Detector::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::vector<cv::Mat> img_batch;
        last_frame_time_ms = current_time_ms;
        current_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count() - pro_start_time;
        cv::Mat src, src_raw;
        if (rgb_or_bayer) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            cv::cvtColor(cv_ptr->image, src, cv::COLOR_BayerBG2BGR);
            if (src.empty()) {
                RCLCPP_ERROR(this->get_logger(), "image is empty");
                return;
            }
        } else {
            src = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        src.copyTo(src_raw);
        tune_img(src);
        img_batch.push_back(src);

        // 第一层识别开始，Preprocess
        cuda_batch_preprocess(img_batch, gpu_buffers_car[0],
                              kInputW, kInputH, stream_car, true);
        // Run inference
        infer(*context_car, stream_car, (void **) gpu_buffers_car,
              cpu_output_buffer_car, kBatchSize);
        // NMS
        std::vector<std::vector<Detection>> result_Detection_batch;
        batch_nms(result_Detection_batch, cpu_output_buffer_car, (int) img_batch.size(),
                  kOutputSize, kConfThresh, kNmsThresh);

        radar_interfaces::msg::YoloPoints yolo_point_list_msg;
        std::vector<Detection> detected_cars = result_Detection_batch[0];
        std::vector<cv::Mat> img_car_batch;
        // DeepSort preprocess boxs
        std::vector<DetectBox> D_box;
        Detection_2_SortBox(src, detected_cars, D_box);  // 把yolo的框转化为DS的框
//        deepsort_.sort(src, D_box);  //跑DS的识别
//        id_process(src, D_box, detected_cars);  //DS识别完了，将DS识别结果转化为yolo的框，供第二层使用
        show_deep_sort(src, D_box);  // 展示DS的识别效果图
        // 至此，第一层网络的识别、追踪已完成

        int iter_detected_car = 0, car_batches_count = 0, car_batch_length = 0;
        //  画面中识别到的车辆的迭代   第二层batch的个数       各个batch的已有长度
        // process for second yolo model
        for (auto detected_car: detected_cars) {
            ++iter_detected_car;
            cv::Rect detected_rect_of_car = get_rect(src_raw, detected_car.bbox);
            if (detected_rect_of_car.x < 0) {
                detected_rect_of_car.x = 0;
            }
            if (detected_rect_of_car.y < 0) {
                detected_rect_of_car.y = 0;
            }
            if ((detected_rect_of_car.x + detected_rect_of_car.width) > src_raw.cols) {
                detected_rect_of_car.width = src_raw.cols - detected_rect_of_car.x;
            }
            if ((detected_rect_of_car.y + detected_rect_of_car.height) > src_raw.rows) {
                detected_rect_of_car.height = src_raw.rows - detected_rect_of_car.y;
            }

            cv::Mat roi_car;
            src_raw(detected_rect_of_car).copyTo(roi_car);
            if (roi_car.empty()) continue;
            img_car_batch.push_back(roi_car);
            if (++car_batch_length < kNumDetectBatchSize && iter_detected_car < (int) detected_cars.size()) continue;

            // 第二层的batch已满，开始推理，Preprocess
            cuda_batch_preprocess(img_car_batch, gpu_buffers_num[0], kInputW,
                                  kInputH, stream_num, false);
            // Run inference
            infer(*context_num, stream_num, (void **) gpu_buffers_num,
                  cpu_output_buffer_num, kNumDetectBatchSize);
            // NMS
            std::vector<std::vector<Detection>> result_Detection_batch_num;
            batch_nms(result_Detection_batch_num, cpu_output_buffer_num, (int) img_car_batch.size(),
                      kOutputSize, kConfThresh, kNmsThresh);
            int iter_nums_batch = 0, car_count_in_batch = 0;
            // 这个for循环先把把这个car中的正对的装甲板找出来，然后再进行一系列判断
            for (auto result_Detection_nums: result_Detection_batch_num) {
//            Detection& car_for_this_iter = detected_cars[car_count_in_batch];
                car_count_in_batch = car_batches_count * kNumDetectBatchSize + iter_nums_batch;
                cv::Rect detected_car_rect = get_rect(
                        src_raw, detected_cars[car_count_in_batch].bbox); // 用装甲板来代替raw图像中的车
                Detection best_num_detection;  // 能够代表这个car rect的装甲板
                best_num_detection.conf = 0;
                best_num_detection.class_id = 14;
                cv::Rect best_num_rect(0, 0, 0, 0);
                // 对一张装甲板图像识别出的数字进行迭代，选出正对雷达站的装甲板
                for (auto &result_Detection_num: result_Detection_nums) {
                    cv::Rect number_in_roi = get_rect(img_car_batch[iter_nums_batch], result_Detection_num.bbox);
                    cv::Rect number_in_img = number_in_roi;
                    number_in_img.x += detected_car_rect.x;
                    number_in_img.y += detected_car_rect.y;
                    if (number_in_img.area() > best_num_rect.area()) {
                        best_num_rect = number_in_img;
                        best_num_detection = result_Detection_num;
                    }
                }
                //根据装甲板ID修改车的ID（14是未识别出，12是红方未识别出，13是蓝方未识别出，其他编号一致）
                int if_is_blue = int(detected_cars[car_count_in_batch].class_id);
                if ((int) best_num_detection.class_id == 14) {  // 第二层yolo没识别出来
                    // TODO: 检查当前是否有工程且距离较近，如果有就是一个工程给了两个框，直接去掉
                    detected_cars[car_count_in_batch].class_id = float(12 + if_is_blue);
                } else {  //yolo识别出来了
                    detected_cars[car_count_in_batch].class_id = best_num_detection.class_id + float(6 * if_is_blue);
                }
                detected_cars[car_count_in_batch].conf = best_num_detection.conf;

                radar_interfaces::msg::YoloPoint yolo_point_msg;
                yolo_point_msg.id = (uint8_t) detected_cars[car_count_in_batch].class_id;
                yolo_point_msg.conf = detected_cars[car_count_in_batch].conf;
                if (yolo_point_msg.id <= 5 || yolo_point_msg.id == 12) {
                    yolo_point_msg.color = false;
                } else {
                    yolo_point_msg.color = true;
                }
                yolo_point_msg.x = (int16_t) detected_car_rect.x;
                yolo_point_msg.y = (int16_t) detected_car_rect.y;
                yolo_point_msg.width = (int16_t) detected_car_rect.width;
                yolo_point_msg.height = (int16_t) detected_car_rect.height;
                yolo_point_list_msg.data.push_back(yolo_point_msg);
                ++iter_nums_batch;
            }

            std::vector<cv::Mat>().swap(img_car_batch);
            car_batch_length = 0;
            ++car_batches_count;
        }

        radar_interfaces::msg::YoloPoints tracker_yolo_point_list_msg = remove_duplicate(yolo_point_list_msg);
        if (init_tracker(tracker_yolo_point_list_msg)) {
            std::cout << std::endl << "call_count: " << call_count << std::endl;
            if (!tracker_yolo_point_list_msg.data.empty()) {
                if (filter_tracker_and_predict()) {
                    calc_cost(tracker_yolo_point_list_msg);
                    if (!sorted_cost_matrix.empty() && !sorted_points.data.empty()) {
                        KM_matching(tracker_yolo_point_list_msg.data.size(),
                                    prediction_points.size(), cost_matrix);
                        update_tracker(tracker_yolo_point_list_msg); // TODO
                        for (auto i : tracker) {
                            if (!i.obsoleted && current_time_ms - i.time < 3000) {
                                cv::rectangle(src, i.rect, cv::Scalar(0, 255, 0), 2);
                                cv::putText(src, std::to_string(i.track_id), cv::Point(i.rect.x, i.rect.y),
                                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                            }
                        }
                    }
                } else {
                    track_element temp_te;
                    for (auto yps: tracker_yolo_point_list_msg.data) {
                        temp_te.rect = cv::Rect(yps.x, yps.y, yps.width, yps.height);
                        temp_te.time = current_time_ms;
                        temp_te.last_time = current_time_ms - current_diff_time_threshold + 100;
                        temp_te.conf = yps.conf;
                        temp_te.class_id = yps.id;
                        temp_te.obsoleted = false;
                        if (yps.id < 11) {
                            temp_te.track_id = yps.id;
                        } else {
                            temp_te.track_id = tracker.size();
                            tracker.push_back(temp_te);
                        }
                    }
                }
            }
        }

//        show_yolo_result_in_img(tracker_yolo_point_list_msg, src);
        // 将yolo识别的矩形发布
        if (!tracker_yolo_point_list_msg.data.empty()) {
            this->rect_publisher_->publish(tracker_yolo_point_list_msg);
        } else {
            radar_interfaces::msg::YoloPoints rect_msg_4_none;
            rect_msg_4_none.text = "none";
            this->rect_publisher_->publish(rect_msg_4_none);
        }
        // 将yolo检图像测结果发布
        add_time_in_img(src, current_time_ms - last_frame_time_ms);
        if (show_by_cv_or_msg == 1) {
            cv_bridge::CvImage yolo_result_img;
            yolo_result_img.encoding = "bgr8";
            yolo_result_img.header.stamp = this->now();
            yolo_result_img.image = src;
            sensor_msgs::msg::Image yolo_result_msg;
            yolo_result_img.toImageMsg(yolo_result_msg);
            this->yolo_publisher_->publish(yolo_result_msg);
        } else if (show_by_cv_or_msg == 0) {
            cv::resize(src, src, cv::Size(src.cols * 0.7, src.rows * 0.7));
            cv::imshow(win_name, src);
        }
    }

    void Yolov5Detector::prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer,
                                         float **gpu_output_buffer, float **cpu_output_buffer, bool if_car) {
        assert(engine->getNbBindings() == 2);  // deprecated for TensorRT version not correspond
        // In order to bind the buffers, we need to know the names of the input and output tensors.
        // Note that indices are guaranteed to be less than IEngine::getNbBindings()
        const int inputIndex = engine->getBindingIndex(kInputTensorName);
        const int outputIndex = engine->getBindingIndex(kOutputTensorName);
        assert(inputIndex == 0);
        assert(outputIndex == 1);
        // Create GPU buffers on device
        if (if_car) {
            CUDA_CHECK(cudaMalloc((void **) gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
            CUDA_CHECK(cudaMalloc((void **) gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));
            *cpu_output_buffer = new float[kBatchSize * kOutputSize];
        } else {
            CUDA_CHECK(cudaMalloc((void **) gpu_input_buffer,
                                  kNumDetectBatchSize * 3 * kInputH * kInputW * sizeof(float)));
            CUDA_CHECK(cudaMalloc((void **) gpu_output_buffer, kNumDetectBatchSize * kOutputSize * sizeof(float)));
            *cpu_output_buffer = new float[kNumDetectBatchSize * kOutputSize];
        }
    }

    void Yolov5Detector::infer(IExecutionContext &context, cudaStream_t &stream, void **gpu_buffers,
                               float *output, int batchsize) {
        context.enqueue(batchsize, gpu_buffers, stream, nullptr);
        CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float),
                                   cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);
    }

    void Yolov5Detector::serialize_engine(unsigned int max_batchsize, bool &is_p6, float &gd, float &gw,
                                          std::string &wts_name, std::string &engine_name) {
        // Create builder
        IBuilder *builder = createInferBuilder(gLogger_);
        IBuilderConfig *config = builder->createBuilderConfig();

        // Create model to populate the network, then set the outputs and create an engine
        ICudaEngine *engine = nullptr;
        if (is_p6) {
            engine = build_det_p6_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
        } else {
            engine = build_det_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
        }
        assert(engine != nullptr);

        // Serialize the engine
        IHostMemory *serialized_engine = engine->serialize();
        assert(serialized_engine != nullptr);

        // Save engine to file
        std::ofstream p(engine_name, std::ios::binary);
        if (!p) {
            std::cerr << "Could not open plan output file" << std::endl;
            assert(false);
        }
        p.write(reinterpret_cast<const char *>(serialized_engine->data()), serialized_engine->size());

        // Close everything down
        engine->destroy();
        builder->destroy();
        config->destroy();
        serialized_engine->destroy();
    }

    void Yolov5Detector::deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine,
                                            IExecutionContext **context) {
        std::ifstream file(engine_name, std::ios::binary);
        if (!file.good()) {
            std::cerr << "read " << engine_name << " error!" << std::endl;
            assert(false);
        }
        size_t size = 0;
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        char *serialized_engine = new char[size];
        assert(serialized_engine);
        file.read(serialized_engine, size);
        file.close();

        *runtime = createInferRuntime(gLogger_);
        assert(*runtime);
        *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
        assert(*engine);
        *context = (*engine)->createExecutionContext();
        assert(*context);
        delete[] serialized_engine;
    }

    bool Yolov5Detector::FilePreparation(bool if_serialize_engine_car, bool if_serialize_engine_num, bool is_p6,
                                         std::string *return_car_engine, std::string *return_num_engine) {
        std::string data_dir = "/home/hlf/Desktop/radar24_ws/src/yolov5_detect/data";
        std::string car_wts_name = data_dir + "/weights/car_7_18_1_3k.wts";
        std::string car_engine_name = data_dir + "/engine/car_7_18_1_3k.engine";
        *return_car_engine = car_engine_name;
        std::string num_wts_name = data_dir + "/weights/num_1_5w.wts";
        std::string num_engine_name = data_dir + "/engine/num_1_5w.engine";
        *return_num_engine = num_engine_name;
        float gd = 0.33f, gw = 0.50f;  //extracted from parse_args()

        // Create a model using the API directly and serialize it to a file
        if (!car_wts_name.empty() && if_serialize_engine_car) {
            serialize_engine(kBatchSize, is_p6, gd, gw, car_wts_name, car_engine_name);
        }
        if (!num_wts_name.empty() && if_serialize_engine_num) {
            serialize_engine(kNumDetectBatchSize, is_p6, gd, gw, num_wts_name, num_engine_name);
        }
        if (if_serialize_engine_car || if_serialize_engine_num) {
            return true;
        } else {
            return false;
        }
    }

    void Yolov5Detector::show_deep_sort(cv::Mat &src, std::vector<DetectBox> &target_box) {
        cv::Mat src_2_show;
        src.copyTo(src_2_show);
        for (auto box: target_box) {
            cv::Point lt(box.x1, box.y1);
            cv::Point br(box.x2, box.y2);
            cv::rectangle(src_2_show, lt, br, cv::Scalar(255, 0, 0), 1);
            //std::string lbl = cv::format("ID:%d_C:%d_CONF:%.2f", (int)box.trackID, (int)box.classID, box.confidence);
            //std::string lbl = cv::format("ID:%d_C:%d", (int)box.trackID, (int)box.classID);
            std::string lbl = cv::format("ID:%d", (int) box.trackID);
            cv::putText(src_2_show, lbl, lt, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
        }
        cv_bridge::CvImage deepsort_result_img;
        deepsort_result_img.encoding = "bgr8";
        deepsort_result_img.header.stamp = this->now();
        deepsort_result_img.image = src_2_show;
        sensor_msgs::msg::Image deepsort_result_msg;
        deepsort_result_img.toImageMsg(deepsort_result_msg);
        this->deepsort_publisher_->publish(deepsort_result_msg);
    }

    void Yolov5Detector::load_parameters() {
        declare_parameter<std::string>("camera_name", "camera_init");
        declare_parameter<int>("show_by_cv_or_msg", 0);
        declare_parameter<bool>("rgb_or_bayer", false);
        declare_parameter<double>("light_gain", 0.0);
        declare_parameter<double>("saturation_gain", 0.0);
        declare_parameter<int>("roi_x", 0);
        declare_parameter<int>("roi_y", 0);
        declare_parameter<int>("image_width", 0);
        declare_parameter<int>("image_height", 0);
        declare_parameter<double>("last_diff_time_threshold", 0.0);


        camera_name = get_parameter("camera_name").as_string();
        show_by_cv_or_msg = get_parameter("show_by_cv_or_msg").as_int();
        rgb_or_bayer = get_parameter("rgb_or_bayer").as_bool();
        light_gain = get_parameter("light_gain").as_double();
        saturation_gain = get_parameter("saturation_gain").as_double();
        roi_x = get_parameter("roi_x").as_int();
        roi_y = get_parameter("roi_y").as_int();
        img_width = get_parameter("image_width").as_int();
        img_height = get_parameter("image_height").as_int();
        last_diff_time_threshold = get_parameter("last_diff_time_threshold").as_double();

//        camera_name = "camera_far";
//        show_by_cv_or_msg = 0;
//        rgb_or_bayer = false;
//        light_gain = 2.0;
//        saturation_gain = 1.6;

        RCLCPP_INFO(get_logger(), "camera_name:[%s]\nshow_by_cv_or_msg:[%d]\nrgb_or_bayer:[%d]",
                    camera_name.c_str(), show_by_cv_or_msg, rgb_or_bayer, light_gain);
        RCLCPP_INFO(get_logger(), "light_gain:[%f]\nsaturation_gain:[%f]", light_gain, saturation_gain);
    }

    void Yolov5Detector::Detection_2_SortBox(cv::Mat &img, const std::vector<Detection> &src_box,
                                             std::vector<DetectBox> &dst_box) {
        // src:center_x, center_y, w, h    dst:lt,rb
        if (!dst_box.empty()) dst_box.clear();
        for (auto i: src_box) {
            DetectBox temp_box;
            cv::Rect rect_res_car = get_rect(img, i.bbox);
            if (rect_res_car.x < 0) {
                rect_res_car.x = 0;
            }
            if (rect_res_car.y < 0) {
                rect_res_car.y = 0;
            }
            if ((rect_res_car.x + rect_res_car.width) > img.cols) {
                rect_res_car.width = img.cols - rect_res_car.x;
            }
            if ((rect_res_car.y + rect_res_car.height) > img.rows) {
                rect_res_car.height = img.rows - rect_res_car.y;
            }
            temp_box.x1 = (float) rect_res_car.x;
            temp_box.y1 = (float) rect_res_car.y;
            temp_box.x2 = (float) rect_res_car.x + (float) rect_res_car.width;
            temp_box.y2 = (float) rect_res_car.y + (float) rect_res_car.height;
            temp_box.confidence = i.conf;
            temp_box.classID = i.class_id;
            temp_box.trackID = 0;
            dst_box.push_back(temp_box);
        }
    }

    void Yolov5Detector::add_time_in_img(cv::Mat &img, double time) {
        std::string text = std::to_string(time) + " ms";
        // 计算文本的大小
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 1;
        int thickness = 2;
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
        // 计算文本的位置（右下角）
        cv::Point textOrg(img.cols - textSize.width, img.rows - baseline);
        // 在图像上添加文本
        cv::putText(img, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
    }

    bool Yolov5Detector::check_same_id(radar_interfaces::msg::YoloPoints yoloPointList, int id, int threshold) {
        int count = 0;
        for (auto i: yoloPointList.data) {
            if (i.id == id) {
                count++;
            }
        }
        return count > threshold;
    }

    void Yolov5Detector::show_yolo_result_in_img(radar_interfaces::msg::YoloPoints filter_yolo_point_list_msg,
                                                 cv::Mat &src_raw) {
        if (show_count++ > show_count_threshold) {
            show_yolo_point_list = std::move(filter_yolo_point_list_msg);
            show_count = 0;
        }
        for (auto i: show_yolo_point_list.data) {
            std::string str_2_print, dui_str;
            switch ((int) i.id) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4: {
                    str_2_print = std::to_string(char((int) i.id + 1));
                    break;
                }
                case 5: {
                    str_2_print = "G";
                    break;
                }
                case 6:
                case 7:
                case 8:
                case 9:
                case 10: {
                    str_2_print = std::to_string(char((int) i.id - 5));
                    break;
                }
                case 11: {
                    str_2_print = "G";
                    break;
                }
                case 12:
                case 13: {
                    str_2_print = "X";
                    break;
                }
                default:
                    str_2_print = "===";
            }
            cv::putText(src_raw, str_2_print, cv::Point(i.x + 12, i.y - 1),
                        cv::FONT_HERSHEY_PLAIN, 2, color_table["Green"], 2);

            if ((int) i.id < 6 || i.id == 12) {
                dui_str = "Red";
            } else if ((int) i.id < 12 || i.id == 13) {
                dui_str = "Blue";
            }
            cv::Rect detected_car_rect = cv::Rect(i.x, i.y, i.width, i.height);
            cv::rectangle(src_raw, detected_car_rect, color_table["Brick_red"], 4);
            cv::circle(src_raw, cv::Point(detected_car_rect.x, detected_car_rect.y - 1),
                       15, color_table[dui_str], -1);
        }
    }

    /*
     * 综合考虑两个框的重合度和id情况进行去重
     */
    radar_interfaces::msg::YoloPoints Yolov5Detector::remove_duplicate(radar_interfaces::msg::YoloPoints &yoloPointList) {
        radar_interfaces::msg::YoloPoints processed_msg;
        for (const auto &pp1: yoloPointList.data) {
            bool overlap = false;
            cv::Rect r1 = cv::Rect(pp1.x, pp1.y, pp1.width, pp1.height);
            for (auto &pp2: processed_msg.data) {  //检查与已经加入有没有重复，如果新的更复合要求则把已经加入队列里的替换为新的
                cv::Rect r2 = cv::Rect(pp2.x, pp2.y, pp2.width, pp2.height);
                cv::Rect intersect = r1 & r2;
                int overlapArea = intersect.area();
                if (int(pp1.id) == int(pp2.id) && int(pp1.id) < 12 && int(pp2.id) < 12) {
                    overlap = true; // 具有重叠度且两个id相同，则认定面积较大的
//                    std::cout << "[" << int(pp1.id) << ", " << int(pp2.id) << "] "<<overlapArea << " , 1:" << r1.area() << ", 2:" << r2.area() << std::endl;
                    if (r1.area() > r2.area()) pp2 = pp1;
                    break;
                } else if ((int(pp1.id) == 12 && int(pp2.id) < 6) ||
                           (int(pp1.id) == 13 && int(pp2.id) < 12 && int(pp2.id) > 5)) {
                    if (overlapArea > 3 * duplicate_threshold * r1.area() ||
                        overlapArea > 3 * duplicate_threshold * r2.area()) {
//                        std::cout << "[" << int(pp1.id) << ", " << int(pp2.id) << "] "<<overlapArea << " , 1:" << r1.area() << ", 2:" << r2.area() << std::endl;
                        overlap = true; // 重叠度较高且pp1为未知
                        break;
                    }
                } else if ((int(pp2.id) == 12 && int(pp1.id) < 6) ||
                           (int(pp2.id) == 13 && int(pp1.id) < 12 && int(pp1.id) > 5)) {
                    if (overlapArea > 3 * duplicate_threshold * r1.area() ||
                        overlapArea > 3 * duplicate_threshold * r2.area()) {
//                        std::cout << "[" << int(pp1.id) << ", " << int(pp2.id) << "] "<<overlapArea << " , 1:" << r1.area() << ", 2:" << r2.area() << std::endl;
                        overlap = true; // 重叠度较高且pp2为未知
                        pp2 = pp1;
                        break;
                    }
                } else if ((int(pp1.id) == 12 && int(pp2.id) == 12) || (int(pp1.id) == 13 && int(pp2.id) == 13)) {
                    if (overlapArea > 6 * duplicate_threshold * r1.area() ||
                        overlapArea > 6 * duplicate_threshold * r2.area()) {
//                        std::cout << "[" << int(pp1.id) << ", " << int(pp2.id) << "] "<<overlapArea << " , 1:" << r1.area() << ", 2:" << r2.area() << std::endl;
                        overlap = true; // 重叠度较高且都为未知
                        if (r1.area() > r2.area()) pp2 = pp1;
                        break;
                    }
                } else if ((int(pp1.id) < 6 && int(pp2.id) < 6) ||
                           (int(pp1.id) > 5 && int(pp1.id) < 12 && int(pp2.id) > 5 && int(pp2.id) < 12)) {
                    if (overlapArea > 8 * duplicate_threshold * r1.area() ||
                        overlapArea > 8 * duplicate_threshold * r2.area()) {
//                        std::cout << "[" << int(pp1.id) << ", " << int(pp2.id) << "] "<<overlapArea << " , 1:" << r1.area() << ", 2:" << r2.area() << std::endl;
                        overlap = true; // 重叠度较高且都为未知
                        if (r1.area() > r2.area()) pp2 = pp1;
                        break;
                    }
                } else if ((int(pp1.id) == 12 && int(pp2.id) == 13) || (int(pp1.id) == 13 && int(pp2.id) == 12)
                           || (int(pp1.id) < 6 && int(pp2.id) > 5 && int(pp2.id) < 12) ||
                           (int(pp1.id) > 5 && int(pp1.id) < 12 && int(pp2.id) < 6)) {
                    if (overlapArea > 8 * duplicate_threshold * r1.area() ||
                        overlapArea > 8 * duplicate_threshold * r2.area()) {
//                        std::cout << "[" << int(pp1.id) << ", " << int(pp2.id) << "] "<<overlapArea << " , 1:" << r1.area() << ", 2:" << r2.area() << std::endl;
                        overlap = true; // 重叠度较高且异色
                        if (r1.area() > r2.area()) pp2 = pp1;
                        break;
                    }
                }
            }
            if (!overlap) {
                processed_msg.data.push_back(pp1);
            }
        }
        return processed_msg;
    }

    bool Yolov5Detector::check_same_color(const radar_interfaces::msg::YoloPoint &a,
                                          const radar_interfaces::msg::YoloPoint &b) {
        return (
                ((a.id < 6 || a.id == 12) && (b.id < 6 || b.id == 12)) ||
                (((b.id >= 6 && b.id < 12) || b.id == 13) && ((a.id >= 6 && a.id < 12) || a.id == 13))
        );
    }

    void Yolov5Detector::tune_img(cv::Mat &src) {
        cv::Mat hsvImg, roi_img;

        cv::cvtColor(src, hsvImg, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> hsvChannels;
        cv::split(hsvImg, hsvChannels);
        hsvChannels[1] *= saturation_gain;
        hsvChannels[2] *= light_gain;

        cv::merge(hsvChannels, hsvImg);
        cv::cvtColor(hsvImg, src, cv::COLOR_HSV2BGR);
        cv::Rect roi;
        if (camera_name == "sensor_far") {
            roi= cv::Rect(roi_x, 0, img_width - roi_x, roi_y);
        } else if (camera_name == "sensor_close") {
            roi = cv::Rect(0, roi_y, roi_x, img_height- roi_y);
        }
        roi_img = src(roi);
        src = cv::Mat(src.rows, src.cols, src.type(), cv::Scalar(0, 0, 0));
        roi_img.copyTo(src(roi));
    }

    bool Yolov5Detector::init_tracker(const radar_interfaces::msg::YoloPoints &yps) {
        call_count++;
        if (call_count <= 3) {
            return false;
        }
        else if (call_count == 4) {
            for (int i = 0; i < 12; i++) {
                track_element temp_te;
                temp_te.obsoleted = false;
                temp_te.class_id = i;
                temp_te.time = current_time_ms - 1000;
                temp_te.last_time = current_time_ms - 2000;
                temp_te.track_id = i;
                tracker.push_back(temp_te);
            }
            for (auto i: yps.data) {
                track_element temp_te;
                temp_te.class_id = i.id;
                temp_te.conf = i.conf;
                temp_te.rect = cv::Rect(i.x, i.y, i.width, i.height);
                temp_te.time = current_time_ms;
                temp_te.last_time = current_time_ms - current_diff_time_threshold + 100;
                temp_te.obsoleted = false;
                if (i.id < 12) {
                    temp_te.track_id = i.id;
                    tracker[i.id] = temp_te;
                } else {
                    temp_te.track_id = (int) tracker.size();
                    tracker.push_back(temp_te);
                }
//                std::cout << "tracker_id: " << temp_te.track_id << ", class_id: " << temp_te.class_id
//                << ", time: " << temp_te.time << ", last_time: " << temp_te.last_time
//                << ", oboseleted: " << temp_te.obsoleted << std::endl;
            }
//            std::cout << "tracker_size: " << tracker.size() << std::endl;
            return false;
        } else if (call_count >= 5) return true;
    }

    bool Yolov5Detector::filter_tracker_and_predict() {
        prediction_points.clear();
        double last_diff_time = 0.0, current_diff_time = 0.0;
        for (auto & i : tracker) {
            current_diff_time = current_time_ms - i.time;
            if (current_diff_time > 3000 && i.track_id > 11)
                i.obsoleted = true;
        } // 去除太久未更新的traker_element
        std::pair<int, radar_interfaces::msg::YoloPoint> temp_p;
        for (const auto &i: tracker) {
            current_diff_time = current_time_ms - i.time;
//            std::cout << current_diff_time << "cdt" << std::endl;
            if (current_diff_time < current_diff_time_threshold && !i.obsoleted) {
                std::cout << "admit: " << std::endl;
                last_diff_time = i.time - i.last_time;
                if (last_diff_time < last_diff_time_threshold ) {
//                    std::cout << "l<400(g)" << std::endl;
                    temp_p.first = i.track_id;
                    temp_p.second.id = i.class_id;
                    temp_p.second.conf = i.conf;
                    std::cout << "[" << i.rect.x << ", " << i.rect.y << "] ==> [";
                    if (last_diff_time > last_diff_time_threshold / 2) {
                        std::cout << "the same" << std::endl;
                        temp_p.second.x = i.rect.x;
                        temp_p.second.y = i.rect.y;
                        temp_p.second.width = i.rect.width;
                        temp_p.second.height = i.rect.height;
                    } else { // 如果之前两帧（前一帧和前前帧）之间的时间差小于200ms，则进行线性预测
                        temp_p.second.x =
                                i.rect.x + (i.rect.x - i.last_rect.x) * current_diff_time / last_diff_time;
                        temp_p.second.y =
                                i.rect.y + (i.rect.y - i.last_rect.y) * current_diff_time / last_diff_time;
                        temp_p.second.width = i.rect.width;
                        temp_p.second.height = i.rect.height;
                        std::cout << temp_p.second.x << ", " << temp_p.second.y << "]" << std::endl;
                    }
                    prediction_points.push_back(temp_p);
                }
            }
        }
        return !prediction_points.empty();
    }

    void Yolov5Detector::calc_cost(radar_interfaces::msg::YoloPoints &now_points) {
        cost_matrix = std::vector<std::vector<int>>(20, std::vector<int>(20, 0));
        sorted_cost_matrix = std::vector<std::vector<int>>(20, std::vector<int>(20, 0));
        sorted_points.data.clear();
        int max_dist = 0, conflict = 0, sorted_size = 0;
        std::vector<radar_interfaces::msg::YoloPoint>::iterator remove_iter;
        if (now_points.data.size() > prediction_points.size())
            for (auto iter = now_points.data.begin(); iter != now_points.data.end(); ++iter)
                if (iter->id > 11) {
                    conflict = 1;
                    break;
                }

        for (int i = 0; i < now_points.data.size(); i++) {
            for (int j = 0; j < prediction_points.size(); j++) {
                double dist = calc_dist(now_points.data[i].x, now_points.data[i].y,
                                        prediction_points[j].second.x, prediction_points[j].second.y);
                int cost = int(dist / std::max(now_points.data[i].width, now_points.data[i].height) * 10);
                max_dist = std::max(max_dist, cost);
                cost_matrix[i][j] = cost;
            }
        }
        std::vector<std::pair<int, int>> max_cost_list_and_index;
        std::pair<int, int> temp_pair;
        for (int i = 0; i < now_points.data.size(); i++) {
            for (int j = 0; j < prediction_points.size(); j++) {
                cost_matrix[i][j] = max_dist + 20 - cost_matrix[i][j];
                if (prediction_points[j].second.id == now_points.data[i].id &&
                    prediction_points[j].second.id < 12 && now_points.data[i].id < 12) {
                    cost_matrix[i][j] *= 3;
                } else if (check_same_color(prediction_points[j].second, now_points.data[i])) {
                    cost_matrix[i][j] *= 1.5;
                }
            }
            temp_pair.first = *std::max_element(cost_matrix[i].begin(), cost_matrix[i].end());
            temp_pair.second = i;
            max_cost_list_and_index.push_back(temp_pair);
        }
        std::sort(max_cost_list_and_index.begin(), max_cost_list_and_index.end(),
                  [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
                      return a.first > b.first;
                  });
        if (conflict == 1) sorted_size = prediction_points.size();
        else if (conflict == 0) sorted_size = max_cost_list_and_index.size();

        std::cout << "change during sort: " << sorted_points.data.size() << ", " << prediction_points.size() << " ===> ";
        for (int i = 0; i < sorted_size; i++) {
            sorted_points.data.push_back(now_points.data[max_cost_list_and_index[i].second]);
            sorted_cost_matrix[i] = cost_matrix[max_cost_list_and_index[i].second];
        }
        while (sorted_points.data.size() > prediction_points.size()) {
            int target_iter = 0;
            for (int i = sorted_points.data.size() - 1; i > -1; i--) {
                if (sorted_points.data[i].id >= 12) {
                    target_iter = i;
                    track_element temp_te;
                    temp_te.class_id = sorted_points.data[i].id;
                    temp_te.conf = sorted_points.data[i].conf;
                    temp_te.rect = cv::Rect(sorted_points.data[i].x, sorted_points.data[i].y,
                                            sorted_points.data[i].width, sorted_points.data[i].height);
                    temp_te.time = current_time_ms;
                    temp_te.last_time = current_time_ms - current_diff_time_threshold + 100;
                    temp_te.track_id = (int) tracker.size();
                    temp_te.obsoleted = false;
                    tracker.push_back(temp_te);
                    break;
                }
            }
            sorted_points.data.erase(sorted_points.data.begin() + target_iter);
            sorted_cost_matrix.erase(sorted_cost_matrix.begin() + target_iter);
        }
        std::cout << sorted_points.data.size() << ", " << prediction_points.size() << std::endl;
        cost_matrix = sorted_cost_matrix;
        now_points.data = sorted_points.data;
    }

    void Yolov5Detector::update_tracker(radar_interfaces::msg::YoloPoints &now_points) {
        int x_count = 0, y_count = 0;
        std::vector<int> matchX(now_points.data.size());
        for (auto i: matchY) {
            if (i < 0 || i >= matchX.size()) continue;
            matchX[i] = y_count++;
        }
        std::cout << "match list: ";
        for (auto i: matchX) {
            std::cout << "[" << x_count << ", " << i << "] *" ;
            radar_interfaces::msg::YoloPoint temp_yp = now_points.data[x_count++];
            if (temp_yp.id == prediction_points[i].second.id &&
                temp_yp.id < 12 && prediction_points[i].second.id < 12) {
                std::cout << "a*  " ;
                tracker[prediction_points[i].first].last_rect = tracker[prediction_points[i].first].rect;
                tracker[prediction_points[i].first].rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width,
                                                                    temp_yp.height);
                tracker[prediction_points[i].first].last_time = tracker[prediction_points[i].first].time;
                tracker[prediction_points[i].first].time = current_time_ms;
                tracker[prediction_points[i].first].conf = temp_yp.conf;
            } else if (check_same_color(temp_yp, prediction_points[i].second)) {
                if (tracker[prediction_points[i].first].class_id < 12 && temp_yp.id > 11) {
                    std::cout << "b*  " ;
                    now_points.data[x_count].id = tracker[prediction_points[i].first].class_id;
                    tracker[prediction_points[i].first].last_rect = tracker[prediction_points[i].first].rect;
                    tracker[prediction_points[i].first].rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width,
                                                                        temp_yp.height);
                    tracker[prediction_points[i].first].last_time = tracker[prediction_points[i].first].time;
                    tracker[prediction_points[i].first].time = current_time_ms;
                    tracker[prediction_points[i].first].conf = temp_yp.conf;
                } else if (tracker[prediction_points[i].first].class_id > 11 && temp_yp.id < 12) {
                    if (tracker[temp_yp.id].time < current_time_ms || tracker[temp_yp.id].conf < temp_yp.conf) {
                        std::cout << "c*  " ;
                        tracker[temp_yp.id].last_rect = tracker[prediction_points[i].first].rect;
                        tracker[temp_yp.id].rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width, temp_yp.height);
                        tracker[temp_yp.id].last_time = tracker[prediction_points[i].first].time;
                        tracker[temp_yp.id].time = current_time_ms;
                        tracker[temp_yp.id].conf = temp_yp.conf;
                        tracker[prediction_points[i].first].obsoleted = true;
                    } else {
                        std::cout << "d*  " ;
                        tracker[prediction_points[i].first].last_rect = tracker[prediction_points[i].first].rect;
                        tracker[prediction_points[i].first].rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width,
                                                                            temp_yp.height);
                        tracker[prediction_points[i].first].last_time = tracker[prediction_points[i].first].time;
                        tracker[prediction_points[i].first].time = current_time_ms;
                        tracker[prediction_points[i].first].conf /= 2;
                        now_points.data[x_count].id = tracker[prediction_points[i].first].class_id;
                    }
                } else if (tracker[prediction_points[i].first].class_id > 11 && temp_yp.id > 11) {
                    if (tracker[prediction_points[i].first].time < current_time_ms) {
                        std::cout << "e*  " ;
                        tracker[prediction_points[i].first].last_rect = tracker[prediction_points[i].first].rect;
                        tracker[prediction_points[i].first].rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width,
                                                                            temp_yp.height);
                        tracker[prediction_points[i].first].last_time = tracker[prediction_points[i].first].time;
                        tracker[prediction_points[i].first].time = current_time_ms;
                    } else {
                        std::cout << "f*  " ;
                        track_element temp_te;
                        temp_te.rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width, temp_yp.height);
                        temp_te.time = current_time_ms;
                        temp_te.last_time = current_time_ms - current_diff_time_threshold + 100;
                        temp_te.conf = temp_yp.conf;
                        temp_te.class_id = temp_yp.id;
                        temp_te.track_id = tracker.size();
                        temp_te.obsoleted = false; //去除前12以外的tracker_element的方法
                        tracker.push_back(temp_te);
                    }
                } else if (temp_yp.id < 12 && prediction_points[i].second.id < 12) {
                    if (tracker[temp_yp.id].time < current_time_ms || tracker[temp_yp.id].conf < temp_yp.conf) {
                        std::cout << "g*  " ;
                        tracker[temp_yp.id].last_rect = tracker[prediction_points[i].first].rect;
                        tracker[temp_yp.id].rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width, temp_yp.height);
                        tracker[temp_yp.id].last_time = tracker[prediction_points[i].first].time;
                        tracker[temp_yp.id].time = current_time_ms;
                        tracker[temp_yp.id].conf = temp_yp.conf;
                        tracker[prediction_points[i].first].time = current_time_ms - 40000;
                    } else {
                        std::cout << "h*  " ;
                        tracker[prediction_points[i].first].last_rect = tracker[prediction_points[i].first].rect;
                        tracker[prediction_points[i].first].rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width,
                                                                            temp_yp.height);
                        tracker[prediction_points[i].first].last_time = tracker[prediction_points[i].first].time;
                        tracker[prediction_points[i].first].time = current_time_ms;
                        tracker[prediction_points[i].first].conf /= 2;
                        now_points.data[x_count].id = tracker[prediction_points[i].first].class_id;
                        tracker[temp_yp.id].time = current_time_ms - 40000; //去除前12tracker_element的方法
                    }
                }
            } else { //TODO 异色
                std::cout << "i*  " ;
                track_element temp_te;
                temp_te.rect = cv::Rect(temp_yp.x, temp_yp.y, temp_yp.width, temp_yp.height);
                temp_te.time = current_time_ms;
                temp_te.last_time = current_time_ms - current_diff_time_threshold + 100;
                temp_te.conf = temp_yp.conf;
                temp_te.class_id = temp_yp.id;
                temp_te.track_id = tracker.size();
                temp_te.obsoleted = false;
                tracker.push_back(temp_te);
            }
        }
        std::cout << std::endl;
    }

    double Yolov5Detector::calc_dist(int x1, int y1, int x2, int y2) {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }

}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(yolov5_detect::Yolov5Detector);
