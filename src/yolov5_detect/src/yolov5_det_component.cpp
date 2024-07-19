#include "yolov5_detect_component.h"

namespace yolov5_detect {

    Yolov5Detector::Yolov5Detector(const rclcpp::NodeOptions &options) :
            Node("yolov5_detector", options),
            deepsort_(Deepsort_engine, 128, 256, 0, &gLogger_) {
        cudaSetDevice(kGpuId);

        if (FilePreparation(false, false, false, &car_engine_name, &num_engine_name)) {
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

//        filter_stack = std::vector<std::deque<cv::Point2f>>(12);
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
        std::chrono::duration<double, std::milli> diff_time = std::chrono::high_resolution_clock::now() - last_frame_time;
        double diff_time_ms = diff_time.count();
        last_frame_time = std::chrono::high_resolution_clock::now();
        // Decode the Bayer image to BGR format
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat src, src_raw;
        src = cv_bridge::toCvShare(msg, "bgr8")->image;
//        try {
//            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
//        } catch (cv_bridge::Exception &e) {
//            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//            return;
//        }
//        cv::cvtColor(cv_ptr->image, src, cv::COLOR_BayerBG2BGR);
//        if (src.empty()) {
//            RCLCPP_ERROR(this->get_logger(), "image is empty");
//            return;
//        }
        src.copyTo(src_raw);
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
            src(detected_rect_of_car).copyTo(roi_car);
            if (roi_car.empty()) continue;
            img_car_batch.push_back(roi_car);
            if (++car_batch_length < kNumDetectBatchSize && iter_detected_car < (int) detected_cars.size()) continue;

            // 第二层的batch已满，开始推理，Preprocess
            cuda_batch_preprocess(img_car_batch, gpu_buffers_num[0], kInputW,
                                  kInputH, stream_num, false);
            // Run inference
            infer(*context_num, stream_num, (void **) gpu_buffers_num,
                  cpu_output_buffer_num, kBatchSize); // , kNumDetectBatchSize
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
                for (auto & result_Detection_num : result_Detection_nums) {
//                std::cout << result_Detection_nums[m].class_id << std::endl;
                    cv::Rect number_in_roi = get_rect(img_car_batch[iter_nums_batch], result_Detection_num.bbox);
                    cv::Rect number_in_img = number_in_roi;
                    number_in_img.x += detected_car_rect.x;
                    number_in_img.y += detected_car_rect.y;
                    if (number_in_img.area() > best_num_rect.area()) {
                        best_num_rect = number_in_img;
                        best_num_detection = result_Detection_num;
//                        if (best_num_detection.class_id == 4) best_num_detection.class_id = 2;
                    }
//                cv::putText(src_raw, std::to_string((int)result_Detection_nums[m].class_id), cv::Point(number_in_img.x, number_in_img.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
//                if (result_Detection_nums[m].conf > best_num_detection.conf) {
//                    best_num_detection = result_Detection_nums[m];
//                }
                }
                //根据装甲板ID修改车的ID（14是未识别出，12是红方未识别出，13是蓝方未识别出，其他编号一致）
                int if_is_blue, tracker_id;
                if ((int) best_num_detection.class_id == 14) {  // 第二层yolo没识别出来
                    // TODO: 检查当前是否有工程且距离较近，如果有就是一个工程给了两个框，直接去掉
                    if_is_blue = int(detected_cars[car_count_in_batch].class_id);
                    detected_cars[car_count_in_batch].class_id = float(12 + if_is_blue);
                } else {  //yolo识别出来了
                    if_is_blue = int(detected_cars[car_count_in_batch].class_id);
                    detected_cars[car_count_in_batch].class_id = best_num_detection.class_id + float(6 * if_is_blue);
                }
                detected_cars[car_count_in_batch].conf = best_num_detection.conf;

                radar_interfaces::msg::YoloPoint yolo_point_msg;
                yolo_point_msg.id = (uint8_t) detected_cars[car_count_in_batch].class_id;
                for (int iii = 0; iii < 12; iii++) {
                    if (yolo_point_msg.id == iii) {
                        yolo_point_msg.tracker_id = (int16_t) id_tracker_table[iii];
                        break;
                    }
                }
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

        radar_interfaces::msg::YoloPoints filter_yolo_point_list_msg;
        yolo_point_list_msg = remove_duplicate(yolo_point_list_msg); //自身框去重，相当于NMS
        if (last_yolo_point_list.data.empty()) {
            filter_yolo_point_list_msg = yolo_point_list_msg;
        } else if (!yolo_point_list_msg.data.empty()){ // 帧间运动匹配
            for (auto now_yolo_point: yolo_point_list_msg.data) {
                cv::Rect now_rect = cv::Rect(now_yolo_point.x, now_yolo_point.y, now_yolo_point.width,now_yolo_point.height);
                int now_id = now_yolo_point.id;
                cv::Rect overlap_rect;
                double max_overlap_area = 0;
                int max_area_iter = 0;
                for (int iter = 0; iter < (int) last_yolo_point_list.data.size(); iter++) { // 找出与last point重合度最大的point
                    auto last_yolo_point = last_yolo_point_list.data[iter];
                    cv::Rect last_rect = cv::Rect(last_yolo_point.x, last_yolo_point.y, last_yolo_point.width,
                                                  last_yolo_point.height);
                    overlap_rect = last_rect & now_rect;
                    if (overlap_rect.area() > max_overlap_area) {
                        max_overlap_area = overlap_rect.area();
                        max_area_iter = iter;
                    }
                }
                if (max_overlap_area < now_rect.area() * overlap_threshold) { // now在last中无对应点，认为其新增了point
                    if (now_id < 12) {
                        filter_yolo_point_list_msg.data.push_back(now_yolo_point);
                    } else if (now_id == 12 || now_id == 13) {
                        // TODO: 加一个处理，给未知的预测一个值
                        filter_yolo_point_list_msg.data.push_back(now_yolo_point);
                    }
                } else { //now中某点能与last中部分点对应
                    int last_id = last_yolo_point_list.data[max_area_iter].id;
                    if (now_id == last_id || (now_id < 12 && last_id > 11)) {
                        filter_yolo_point_list_msg.data.push_back(now_yolo_point);
                        cv::circle(src_raw, cv::Point(now_yolo_point.x + 16,now_yolo_point.y + 16),
                                   10, color_table["Purple"], -1); // adopt now
                        last_yolo_point_list.data.erase(last_yolo_point_list.data.begin() + max_area_iter);
                        continue; // 如果是同一辆车或now识别出、last为识别出，直接用now，跳过now的循环
                    }
                    if (now_id > 11) { // now未识别出，直接用重合度最大的last进行纠正，无论last是否识别出
                        filter_yolo_point_list_msg.data.push_back(last_yolo_point_list.data[max_area_iter]);
                        cv::circle(src_raw, cv::Point(now_yolo_point.x + 16,now_yolo_point.y + 16),
                                   10, color_table["Yellow"], -1); // adopt last
                        last_yolo_point_list.data.erase(last_yolo_point_list.data.begin() + max_area_iter);
                    } else { // now和last都识别出了，但是相矛盾，进行纠正
                        if ((last_id < 6 &&  now_id > 5) || (now_id < 6 && last_id > 5)) {
                            filter_yolo_point_list_msg.data.push_back(now_yolo_point); // adopt now
                        } else if ((last_id < 6 && now_id < 6) || (now_id > 5 && last_id > 5 && last_id < 12)) {
                            if (diff_time_ms < time_threshold) {
                                filter_yolo_point_list_msg.data.push_back(last_yolo_point_list.data[max_area_iter]);
                                cv::circle(src_raw, cv::Point(now_yolo_point.x + 16,now_yolo_point.y + 16),
                                           10, color_table["Yellow"], -1); // adopt last
                                last_yolo_point_list.data.erase(last_yolo_point_list.data.begin() + max_area_iter);
                            }
                        }
                    }
                }
            }
        }
//        filter_yolo_point_list_msg = yolo_point_list_msg;
        last_yolo_point_list = filter_yolo_point_list_msg;
        show_yolo_result_in_img(filter_yolo_point_list_msg, src_raw);
        // 将yolo识别的矩形发布
        if (!filter_yolo_point_list_msg.data.empty()) {
            this->rect_publisher_->publish(filter_yolo_point_list_msg);
            // TODO : cancel debug out
//        std::cout << std::endl << "close_rectangles_publish_one_msg: " << std::endl;
//        for (auto &i : rect_msg.data) {
//            std::cout << "id: " << (int)i.id << " , [ " << i.x << " , " << i.y << " , (" << i.width << " , " << i.height << ") ]"<< std::endl;
//        }
            // TODO : end at here
        } else {
            radar_interfaces::msg::YoloPoints rect_msg_4_none;
            rect_msg_4_none.text = "none";
            this->rect_publisher_->publish(rect_msg_4_none);
        }
        // 将yolo检图像测结果发布
        add_time_in_img(src_raw, diff_time_ms);
        if (show_by_cv_or_msg == 1) {
            cv_bridge::CvImage yolo_result_img;
            yolo_result_img.encoding = "bgr8";
            yolo_result_img.header.stamp = this->now();
            yolo_result_img.image = src_raw;
            sensor_msgs::msg::Image yolo_result_msg;
            yolo_result_img.toImageMsg(yolo_result_msg);
            this->yolo_publisher_->publish(yolo_result_msg);
        } else if (show_by_cv_or_msg == 0) {
            cv::resize(src_raw, src_raw, cv::Size(src_raw.cols*0.7, src_raw.rows*0.7));
            cv::imshow(win_name, src_raw);
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
        } else {
            CUDA_CHECK(cudaMalloc((void **) gpu_input_buffer,
                                  kNumDetectBatchSize * 3 * kInputH * kInputW * sizeof(float)));
            CUDA_CHECK(cudaMalloc((void **) gpu_output_buffer, kNumDetectBatchSize * kOutputSize * sizeof(float)));
        }


        *cpu_output_buffer = new float[kBatchSize * kOutputSize];
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
        std::string num_wts_name = data_dir + "/weights/bestnum_5_26_1_5w.wts";
        std::string num_engine_name = data_dir + "/engine/bestnum_5_26_1_5w.engine";
        *return_num_engine = num_engine_name;
        float gd = 0.33f, gw = 0.50f;  //extracted from parse_args()

        // Create a model using the API directly and serialize it to a file
        if (!car_wts_name.empty() && if_serialize_engine_car) {
            serialize_engine(kBatchSize, is_p6, gd, gw, car_wts_name, car_engine_name);
        }
        if (!num_wts_name.empty() && if_serialize_engine_num) {
            serialize_engine(kBatchSize, is_p6, gd, gw, num_wts_name, num_engine_name);
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
        this->declare_parameter<std::string>("camera_name", "camera_init");
        camera_name = this->get_parameter("camera_name").as_string();
        this->declare_parameter<int>("show_by_cv_or_msg", 0);
        show_by_cv_or_msg = this->get_parameter("show_by_cv_or_msg").as_int();
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

    void Yolov5Detector::id_process(cv::Mat &img, const std::vector<DetectBox> &D_box,
                                    std::vector<Detection> &detected_cars) {
        // D_box:lt,rb    detected_cars:center_x, center_y, w, h
        // 1xxxxx stand for red, 2xxxxx stands for blue, xxxxx stand for tracker ID
        for (size_t i = 0; i < D_box.size(); i++) {
            cv::Point lt((int) D_box[i].x1, (int) D_box[i].y1), rb((int) D_box[i].x2, (int) D_box[i].y2);
            cv::Rect D_rect(lt, rb);
            for (size_t j = 0; j < detected_cars.size(); j++) {
                cv::Rect R_rect = get_rect(img, detected_cars[j].bbox);
                if (D_rect.contains(cv::Point(R_rect.x + R_rect.width / 2, R_rect.y + R_rect.height / 2))) {
                    detected_cars[j].class_id = ((detected_cars[j].class_id + 1) * 100000) + D_box[i].trackID;
                    break;
                }
            }
        }
    }


    void Yolov5Detector::id_correct(float id_yolo, float &id_dst, bool if_far) {
        int is_blue, tracker_id, *id_tracker_table;
//    if (if_far) id_tracker_table = id_tracker_far_table;
//    else id_tracker_table = id_tracker_close_table;
        if ((int) id_yolo == 14) {  // yolo识别没出来
            if (id_dst >= 10000) {  //tracker追踪到了这个车
                is_blue = int(id_dst) / 100000 - 1 == 1;
                tracker_id = int(id_dst) % 100000;
                bool if_find = false;
                for (int i = 0; i < 12; i++) {
                    if (id_tracker_table[i] == tracker_id) {
                        id_dst = (float) i;
                        if_find = true;
                        break;
                    }
                }
                if (!if_find) {
                    id_dst = float(12 + is_blue);
                }
            } else {  //tracker没追踪到
                is_blue = int(id_dst);
                id_dst = float(12 + is_blue);
            }
        } else {  //yolo识别出来了
            if (id_dst >= 10000) {  //tracker追踪到了这个车
                is_blue = int(id_dst) / 100000 - 1 == 1;
                tracker_id = int(id_dst) % 100000;
                id_dst = id_dst + float(6 * is_blue);
                for (int j = 0; j < 12; j++) {
                    if (id_tracker_table[j] == tracker_id) id_tracker_table[j] = 0;
                }
                std::cout << id_tracker_table[int(id_dst)] << std::endl;
                id_tracker_table[int(id_dst)] = tracker_id;
            } else {  //tracker没追踪到
                is_blue = int(id_dst);
                id_dst = id_dst + float(6 * is_blue);
            }
        }
    }

    int Yolov5Detector::encoding2mat_type(const std::string &encoding) {
        if (encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
            return CV_BayerRG2BGR;
        } else if (encoding == sensor_msgs::image_encodings::BAYER_BGGR8) {
            return CV_BayerBG2BGR;
        } else if (encoding == sensor_msgs::image_encodings::BAYER_GRBG8) {
            return CV_BayerGR2BGR;
        } else if (encoding == sensor_msgs::image_encodings::BAYER_GBRG8) {
            return CV_BayerGB2BGR;
        } else {
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    void Yolov5Detector::filter_points(const std::array<std::deque<cv::Point2f>, 12>& p_stack,
                       std::array<std::vector<cv::Point2f>, 6>& red_stack,
                       std::array<std::vector<cv::Point2f>, 6>& blue_stack) {
        for (int i = 0; i < 12; i++) {
            if (i < 6) {
                for (int j = 0; j < 8; j++) {
                    if ((p_stack[i][j].x > 0 || p_stack[i][j].y > 0) && red_stack[i].size() < 4) {
                        red_stack[i].push_back(p_stack[i][j]);
                    }
                }
            } else {
                for (int j = 0; j < 8; j++) {
                    if ((p_stack[i][j].x > 0 || p_stack[i][j].y > 0) && blue_stack[i].size() < 4) {
                        blue_stack[i].push_back(p_stack[i][j]);
                    }
                }
            }
        }
    }

    int Yolov5Detector::classify(const std::array<std::vector<cv::Point2f>, 6>& category_stack,
                                 const cv::Point2f& point) {
        double min_distance = std::numeric_limits<float>::max();
        int min_index = -1;

        for (int i = 0; i < 6; ++i) {
            for (const auto& j : category_stack[i]) {
                double dist = cv::norm(point-j);
                if (dist < min_distance) {
                    min_distance = dist;
                    min_index = i;
                }
            }
        }

        return min_index;
    }

    void Yolov5Detector::add_time_in_img(cv::Mat& img, double time) {
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
        for (auto i : yoloPointList.data) {
            if (i.id == id) {
                count++;
            }
        }
        return count > threshold;
    }

    void Yolov5Detector::show_yolo_result_in_img(radar_interfaces::msg::YoloPoints filter_yolo_point_list_msg, cv::Mat& src_raw) {
        if (show_count++ > show_count_threshold) {
            show_yolo_point_list = filter_yolo_point_list_msg;
            show_count = 0;
        }
        for (auto i : show_yolo_point_list.data) {
            std::string str_2_print, dui_str;
            switch ((int) i.id) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4: {
                    str_2_print = std::to_string(char((int)i.id + 1));
                    break;
                } case 5: {
                    str_2_print = "G";
                    break;
                } case 6:
                case 7:
                case 8:
                case 9:
                case 10: {
                    str_2_print = std::to_string(char((int)i.id - 5));
                    break;
                } case 11: {
                    str_2_print = "G";
                    break;
                } case 12:
                case 13: {
                    str_2_print = "X";
                    break;
                }
                default: str_2_print = "===";
            }
            cv::putText(src_raw, str_2_print,cv::Point(i.x + 12, i.y - 1),
                        cv::FONT_HERSHEY_PLAIN, 2,color_table["Green"], 2);

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
    radar_interfaces::msg::YoloPoints Yolov5Detector::remove_duplicate(radar_interfaces::msg::YoloPoints& yoloPointList) {
        radar_interfaces::msg::YoloPoints processed_msg;
        for (const auto& pp1 : yoloPointList.data) {
            bool overlap = false;
            cv::Rect r1 = cv::Rect(pp1.x, pp1.y, pp1.width, pp1.height);
            for (auto& pp2 : processed_msg.data) {  //检查与已经加入有没有重复，如果新的更复合要求则把已经加入队列里的替换为新的
                cv::Rect r2 = cv::Rect(pp2.x, pp2.y, pp2.width, pp2.height);
                cv::Rect intersect = r1 & r2;
                int overlapArea = intersect.area();
                if (pp1.id == pp2.id) {
                    overlap = true; // 具有重叠度且两个id相同，则认定面积较大的
                    if (r1.area() > r2.area()) {
                        pp2 = pp1;
                    }
                    break;
                } else if ((pp1.id == 12 && pp2.id < 6) ||(pp1.id == 13 && pp2.id < 12 && pp2.id > 5) ) {
                    if (overlapArea > 0.05 * r1.area() || overlapArea > 0.05 * r2.area()) {
                        overlap = true; // 重叠度较高且pp1为未知
                        break;
                    }
                } else if ((pp2.id == 12 && pp1.id < 6) ||(pp2.id == 13 && pp1.id < 12 && pp1.id > 5) ) {
                    if (overlapArea > 0.05 * r1.area() || overlapArea > 0.05 * r2.area()) {
                        overlap = true; // 重叠度较高且pp2为未知
                        pp2 = pp1;
                        break;
                    }
                } else if ((pp1.id == 12 && pp2.id == 12) || (pp1.id == 13 && pp2.id == 13)
                || (pp1.id < 6 && pp2.id < 6) || (pp1.id > 5 && pp1.id < 12 && pp2.id > 5 && pp2.id < 12)) {
                    if (overlapArea > 0.1 * r1.area() || overlapArea > 0.1 * r2.area()) {
                        overlap = true; // 重叠度较高且都为未知
                        if (r1.area() > r2.area()) pp2 = pp1;
                        break;
                    }
                }  else if ((pp1.id == 12 && pp2.id == 13) || (pp1.id == 13 && pp2.id == 12)
                || (pp1.id < 6 && pp2.id > 5 && pp2.id < 12) || (pp1.id > 5 && pp1.id < 12 && pp2.id < 6)) {
                    if (overlapArea > 0.2 * r1.area() || overlapArea > 0.2 * r2.area()) {
                        overlap = true; // 重叠度较高且都为已知
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

}




#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(yolov5_detect::Yolov5Detector);