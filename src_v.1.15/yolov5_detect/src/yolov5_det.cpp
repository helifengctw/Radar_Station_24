#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "radar_interfaces/msg/yolo_point.hpp"
#include "radar_interfaces/msg/yolo_points.hpp"


#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cmath>
#include <memory>

using namespace nvinfer1;
using std::placeholders::_1;

static Logger gLogger;
const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

const int kNumDetectBatchSize = 4;

float* gpu_buffers_car[2];
float* gpu_buffers_num[2];
float* cpu_output_buffer_car = nullptr;
float* cpu_output_buffer_num = nullptr;
cudaStream_t stream_car;
cudaStream_t stream_num;
IExecutionContext* context_car = nullptr;
IExecutionContext* context_num = nullptr;

bool parse_args(int argc, char** argv, std::string& wts, std::string& engine,
                bool& is_p6, float& gd, float& gw, std::string& img_dir) {
    if (argc < 4) return false;
    if (std::string(argv[1]) == "-s" && (argc == 5 || argc == 7)) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        auto net = std::string(argv[4]);
        if (net[0] == 'n') {
            gd = 0.33;
            gw = 0.25;
        } else if (net[0] == 's') {
            gd = 0.33;
            gw = 0.50;
        } else if (net[0] == 'm') {
            gd = 0.67;
            gw = 0.75;
        } else if (net[0] == 'l') {
            gd = 1.0;
            gw = 1.0;
        } else if (net[0] == 'x') {
            gd = 1.33;
            gw = 1.25;
        } else if (net[0] == 'c' && argc == 7) {
            gd = atof(argv[5]);
            gw = atof(argv[6]);
        } else {
            return false;
        }
        if (net.size() == 2 && net[1] == '6') {
            is_p6 = true;
        }
    } else if (std::string(argv[1]) == "-d" && argc == 4) {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
    } else {
        return false;
    }
    return true;
}

void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer,
                     float** gpu_output_buffer, float** cpu_output_buffer) {
    assert(engine->getNbBindings() == 2);  // deprecated for TensorRT version not correspond
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));

    *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers,
           float* output, int batchsize) {
    context.enqueue(batchsize, gpu_buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void serialize_engine(unsigned int max_batchsize, bool& is_p6, float& gd, float& gw, std::string& wts_name, std::string& engine_name) {
    // Create builder
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;
    if (is_p6) {
        engine = build_det_p6_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    } else {
        engine = build_det_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    }
    assert(engine != nullptr);

    // Serialize the engine
    IHostMemory* serialized_engine = engine->serialize();
    assert(serialized_engine != nullptr);

    // Save engine to file
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cerr << "Could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    // Close everything down
    engine->destroy();
    builder->destroy();
    config->destroy();
    serialized_engine->destroy();
}

void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

/* custom utils function */
bool FilePreparation(bool if_serialize_engine_car, bool if_serialize_engine_num, bool is_p6,
                      std::string * return_car_engine, std::string * return_num_engine){
    std::string data_dir = "/home/hlf/Desktop/radar24_ws/src/yolov5_detect/data";
    std::string car_wts_name = data_dir + "/weights/car_23_stable.wts";
    std::string car_engine_name = data_dir + "/engine/car_23_stable.engine";
    * return_car_engine = car_engine_name;
    std::string num_wts_name = data_dir + "/weights/num_23_stable.wts";
    std::string num_engine_name = data_dir + "/engine/num_23_stable.engine";
    * return_num_engine = num_engine_name;
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


class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/sensor_far/image_raw", 10, std::bind(&ImageSubscriber::ImageCallback, this, _1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void ImageCallback(sensor_msgs::msg::Image::SharedPtr) const;
};

int main(int argc, char** argv) {
    cudaSetDevice(kGpuId);

    std::string car_engine_name;
    std::string num_engine_name;
    // if necessary, serialize the wts file to an engine file, and return the directory of engine file
    if (FilePreparation(false, false, false,
                        &car_engine_name, &num_engine_name)) {
        return 0;
    }

    IRuntime* runtime = nullptr;

    // Deserialize the engine_car from file
    ICudaEngine* car_engine = nullptr;
    deserialize_engine(car_engine_name, &runtime, &car_engine, &context_car);
    CUDA_CHECK(cudaStreamCreate(&stream_car))
    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize, true);
    // Prepare cpu and gpu buffers
    prepare_buffers(car_engine, &gpu_buffers_car[0],
                    &gpu_buffers_car[1], &cpu_output_buffer_car);

    // Deserialize the engine_num from file
    ICudaEngine* num_engine = nullptr;
    deserialize_engine(num_engine_name, &runtime, &num_engine, &context_num);
    CUDA_CHECK(cudaStreamCreate(&stream_num));
    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize, false);
    // Prepare cpu and gpu buffers
    prepare_buffers(num_engine, &gpu_buffers_num[0],
                    &gpu_buffers_num[1], &cpu_output_buffer_num);

    cv::namedWindow("view");
//    cv::namedWindow("test");
    cv::startWindowThread();

    //Read images from camera
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();


    // Read images from directory
//    std::vector<std::string> file_names;
//    if (read_files_in_dir(img_dir.c_str(), file_names) < 0) {
//        std::cerr << "read_files_in_dir failed." << std::endl;
//        return -1;
//    }

    // batch predict
//    for (size_t i = 0; i < file_names.size(); i += kBatchSize) {
//        // Get a batch of images
//        std::vector<cv::Mat> img_batch_car;
//        std::vector<std::string> img_name_batch;
//        for (size_t j = i; j < i + kBatchSize && j < file_names.size(); j++) {
//            cv::Mat img = cv::imread(img_dir + "/" + file_names[j]);
//            img_batch_car.push_back(img);
//            img_name_batch.push_back(file_names[j]);
//        }
//
//        // Preprocess
//        cuda_batch_preprocess(img_batch_car, gpu_buffers[0], kInputW, kInputH, stream);
//
//        // Run inference
//        auto start = std::chrono::system_clock::now();
//        infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);
//        auto end = std::chrono::system_clock::now();
//        std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
//
//        // NMS
//        std::vector<std::vector<Detection>> res_batch_car;
//        batch_nms(res_batch_car, cpu_output_buffer, img_batch_car.size(), kOutputSize, kConfThresh, kNmsThresh);
//
//        // Draw bounding boxes
//        draw_bbox(img_batch_car, res_batch_car);
//
//        // Save images
//        for (size_t j = 0; j < img_batch_car.size(); j++) {
//                cv::imwrite("_" + img_name_batch[j], img_batch_car[j]);
//        }
//    }

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

    return 0;
}


void ImageSubscriber::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    std::vector<cv::Mat> img_batch_car;
    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat src_raw;
    src.copyTo(src_raw);
    img_batch_car.push_back(src);

    // Preprocess
    cuda_batch_preprocess(img_batch_car, gpu_buffers_car[0], kInputW, kInputH, stream_car, true);

    // Run inference
    auto start_time = std::chrono::system_clock::now();
    infer(*context_car, stream_car, (void**)gpu_buffers_car,
          cpu_output_buffer_car, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch_car;
    batch_nms(res_batch_car, cpu_output_buffer_car, (int)img_batch_car.size(),
              kOutputSize, kConfThresh, kNmsThresh);

    draw_bbox(img_batch_car, res_batch_car);

    radar_interfaces::msg::YoloPoints rect_msg;

    int num_detect_total_count = 0, num_detect_batch_count = 0;

    auto res_cars = res_batch_car[0];
    for (auto res_car : res_cars){
//        std::cout << "car_detect_count : " << num_detect_batch_count << std::endl;
        cv::Mat roi_car;
        cv::Rect rect_res_car = get_rect(src_raw, res_car.bbox);
        if (rect_res_car.x < 0) {
            rect_res_car.x = 0;
        }
        if (rect_res_car.y < 0) {
            rect_res_car.y = 0;
        }
        if ((rect_res_car.x + rect_res_car.width) > src_raw.cols) {
            rect_res_car.width = src_raw.cols - rect_res_car.x;
        }
        if ((rect_res_car.y + rect_res_car.height) > src_raw.rows) {
            rect_res_car.height = src_raw.rows - rect_res_car.y;
        }

        src_raw(rect_res_car).copyTo(roi_car);

        std::vector<cv::Mat> img_batch_num(kNumDetectBatchSize);
        img_batch_num.push_back(roi_car);
        if (roi_car.empty()) {
            continue;
        }

        ++num_detect_batch_count;
        ++num_detect_total_count;
        if (num_detect_batch_count < kNumDetectBatchSize && num_detect_total_count < (int)res_cars.size()) {
            continue;
        }

        num_detect_batch_count = 0;

        // Preprocess
        cuda_batch_preprocess(img_batch_num, gpu_buffers_num[0],roi_car.cols,
                              roi_car.rows, stream_num, false);

        // Run inference
        infer(*context_num, stream_num, (void**)gpu_buffers_num,
              cpu_output_buffer_num, kBatchSize); // kNumDetectBatchSize

        // NMS
        std::vector<std::vector<Detection>> res_batch_num;
        batch_nms(res_batch_num, cpu_output_buffer_num, (int)img_batch_num.size(),
                  kOutputSize, kConfThresh, kNmsThresh);

        int b = 0;
        for (auto res_nums : res_batch_num) {
            b++;
            cv::Rect rough_rect = get_rect(src_raw, res_car.bbox); // 用装甲板来代替raw图像中的车
            Detection real_res;
            real_res.conf = 0;
            real_res.class_id = 14;
            cv::Rect real_rect(0, 0, 0, 0);
            for (size_t m = 0; m < res_nums.size(); m++) {
                cv::Rect number_in_roi = get_rect(img_batch_num[b], res_nums[m].bbox);
                cv::Rect number_in_img = number_in_roi;
                number_in_img.x += rough_rect.x;
                number_in_img.y += rough_rect.y;
                // 选出正对雷达站的装甲板
                if (real_rect.area() < number_in_img.area()) {
                    real_rect = number_in_img;
                }
                cv::rectangle(src_raw, number_in_img, cv::Scalar(0x27, 0xC1, 0x36), 1);

                //cv::putText(img, std::to_string((int)res[m].class_id), cv::Point(number_in_img.x, number_in_img.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                if (res_nums[m].conf > real_res.conf) {
                    real_res = res_nums[m];
                }
            }

            //根据装甲板ID修改车的ID
            //std::cout << real_res.class_id << std::endl;
            if ((int) real_res.class_id == 14) {
                if ((int) res_car.class_id == 0) {
                    res_car.class_id = 12;
                } else {
                    res_car.class_id = 13;
                }
            } else {
                res_car.class_id = real_res.class_id;
            }
            //根据装甲板的rect修改车的rect
            if (real_rect.area() > 0) {
                rough_rect = real_rect;
            }
            else{
                int change = rough_rect.width * 0.2;
                rough_rect.x += change;
                rough_rect.width -= 2 * change;
                change = rough_rect.height * 0.2;
                rough_rect.y += change;
                rough_rect.height -= 2 * change;
            }
            cv::rectangle(src_raw, rough_rect, cv::Scalar(0x27, 0xC1, 0x36), 2);

            if ((int) res_car.class_id <= 4) {
                cv::putText(src_raw, std::string("red") + std::to_string((int) res_car.class_id + 1),
                            cv::Point(rough_rect.x, rough_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
            } else if ((int) res_car.class_id == 5) {
                cv::putText(src_raw, "red guard", cv::Point(rough_rect.x, rough_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                            cv::Scalar(0x27, 0xC1, 0x36), 2);
            } else if ((int) res_car.class_id == 11) {
                cv::putText(src_raw, "blue guard", cv::Point(rough_rect.x, rough_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                            cv::Scalar(0x27, 0xC1, 0x36), 2);
            } else if ((int) res_car.class_id == 12) {
                cv::putText(src_raw, "red", cv::Point(rough_rect.x, rough_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                            cv::Scalar(0x27, 0xC1, 0x36), 2);
            } else if ((int) res_car.class_id == 13) {
                cv::putText(src_raw, "blue", cv::Point(rough_rect.x, rough_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                            cv::Scalar(0x27, 0xC1, 0x36), 2);
            } else {
                cv::putText(src_raw, std::string("blue") + std::to_string((int) res_car.class_id - 5),
                            cv::Point(rough_rect.x, rough_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
            }
            radar_interfaces::msg::YoloPoint yolo_msg;
            yolo_msg.id = res_car.class_id;
            if (yolo_msg.id <= 5 || yolo_msg.id == 12) {
                yolo_msg.color = false;
            } else {
                yolo_msg.color = true;
            }
            yolo_msg.x = (int16_t) rough_rect.x;
            yolo_msg.y = (int16_t) rough_rect.y;
            yolo_msg.width = (int16_t) rough_rect.width;
            yolo_msg.height = (int16_t) rough_rect.height;
            rect_msg.data.push_back(yolo_msg);
        }
    }

    auto end_time = std::chrono::system_clock::now();
    std::cout << "inference time: " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
              << "ms" << std::endl;

//    for (int i = 0; i < 5; i++) {
//        std::string fff_name = "/home/hlf/Downloads/myFiles/Jisai_23/yolov5_origin/ultralytics_yolov5/data/radar_images/";
//        fff_name = fff_name + "_" + std::to_string(i) + ".jpg";
//        std::cout << fff_name << std::endl;
//        cv::imwrite(fff_name, src_raw);
//    }

    cv::imshow("view", src);

}
    
