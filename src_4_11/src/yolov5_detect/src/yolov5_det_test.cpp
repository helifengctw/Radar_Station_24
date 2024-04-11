#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "radar_interfaces/msg/yolo_point.hpp"
#include "radar_interfaces/msg/yolo_points.hpp"
#include "deepsort.h"

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

string Deepsort_engine = "/home/hlf/Desktop/deep_sort/yolov5_deepsort_ws/src/yolo_sort/data/deepsort.engine";
DeepSort DS_close(Deepsort_engine, 128, 256, 0, &gLogger);
DeepSort DS_far(Deepsort_engine, 128, 256, 0, &gLogger);

//DS = new DeepSort(car_engine_name, 128, 256, 0, &gLogger);


std::map<std::string, cv::Scalar> color_table = {
        {"White", cv::Scalar(0xfa, 0xfa, 0xff)}, {"Red", cv::Scalar(0x00, 0x00, 0xff)},
        {"Green_light", cv::Scalar(0xcc, 0xff, 0xcc)}, {"Orange", cv::Scalar(0x00, 0x8c, 0xff)},
        {"Blue", cv::Scalar(0xff, 0x90, 0x1e)}, {"Yellow", cv::Scalar(0x00, 0xff, 0xff)}
};


void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer,
                     float** gpu_output_buffer, float** cpu_output_buffer, bool if_car);
void infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers,
           float* output, int batchsize);
void serialize_engine(unsigned int max_batchsize, bool& is_p6, float& gd, float& gw,
                      std::string& wts_name, std::string& engine_name);
void deserialize_engine(std::string& engine_name, IRuntime** runtime,
                        ICudaEngine** engine, IExecutionContext** context);
/* custom utils function */
bool FilePreparation(bool if_serialize_engine_car, bool if_serialize_engine_num, bool is_p6,
                     std::string * return_car_engine, std::string * return_num_engine);
void Detection_2_SortBox(cv::Mat& img, const std::vector<Detection>& src_box, std::vector<DetectBox> & dst_box);
void id_process(cv::Mat& img, const std::vector<DetectBox>& D_box, std::vector<Detection>& res_cars);
void show_deep_sort(cv::Mat& src, std::vector<DetectBox> & target_box, bool if_far);
void id_correct(float id_yolo, float& id_dst, bool if_far);

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber") {
        far_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/sensor_far/image_raw", 10, std::bind(&ImageSubscriber::FarImageCallback, this, _1));
        close_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/sensor_close/image_raw", 10, std::bind(&ImageSubscriber::CloseImageCallback, this, _1));

        far_rect_publisher_ = this->create_publisher<radar_interfaces::msg::YoloPoints>(
                "far_rectangles", 1);
        close_rect_publisher_ = this->create_publisher<radar_interfaces::msg::YoloPoints>(
                "close_rectangles", 1);
        far_yolo_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/yolo_far", 1);
        close_yolo_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/yolo_close", 1);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr far_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr close_subscription_;

    rclcpp::Publisher<radar_interfaces::msg::YoloPoints>::SharedPtr far_rect_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::YoloPoints>::SharedPtr close_rect_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr far_yolo_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr close_yolo_publisher_;

    void FarImageCallback(sensor_msgs::msg::Image::SharedPtr) const;
    void CloseImageCallback(sensor_msgs::msg::Image::SharedPtr) const;
    void DeepsortCallback(sensor_msgs::msg::Image::SharedPtr) const;
};

long int write_count = 0, sampling_point_close = 0, write_count_close = 0, valid_count_far = 0;
int id_tracker_close_table[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int id_tracker_far_table[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int main(int argc, char** argv) {
    cudaSetDevice(kGpuId);

    std::string car_engine_name;
    std::string num_engine_name;

    // if necessary, serialize the wts file to an engine file, and return the directory of engine file
    if (FilePreparation(false, false, false,&car_engine_name, &num_engine_name)) {
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
                    &gpu_buffers_car[1], &cpu_output_buffer_car, true);

    // Deserialize the engine_num from file
    ICudaEngine* num_engine = nullptr;
    deserialize_engine(num_engine_name, &runtime, &num_engine, &context_num);
    CUDA_CHECK(cudaStreamCreate(&stream_num));
    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize, false);
    // Prepare cpu and gpu buffers
    prepare_buffers(num_engine, &gpu_buffers_num[0],
                    &gpu_buffers_num[1], &cpu_output_buffer_num, false);

    cv::namedWindow("sensor_far_view");
    cv::namedWindow("sensor_close_view");
//    cv::namedWindow("test");
    cv::startWindowThread();

    //Read images from camera
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();

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


void ImageSubscriber::FarImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    std::vector<cv::Mat> img_car_batch;
    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat src_raw;
    src.copyTo(src_raw);
    img_car_batch.push_back(src);

    // Preprocess
    cuda_batch_preprocess(img_car_batch, gpu_buffers_car[0],
                          kInputW, kInputH, stream_car, true);

    // Run inference
    auto start_time = std::chrono::system_clock::now();
    infer(*context_car, stream_car, (void**)gpu_buffers_car,
          cpu_output_buffer_car, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch_car;
    batch_nms(res_batch_car, cpu_output_buffer_car, (int)img_car_batch.size(),
              kOutputSize, kConfThresh, kNmsThresh);

//    draw_bbox(img_car_batch, res_batch_car);
//    cv::resize(src, src, cv::Size(), 0.7, 0.7, cv::INTER_LANCZOS4);
//    cv::imshow("sensor_far_view", src);

    radar_interfaces::msg::YoloPoints rect_msg;
    int iter_res_car = 0, num_batch_size = 0, num_batch_count = 0;
    std::vector<Detection> res_cars = res_batch_car[0];
    std::vector<cv::Mat> img_num_batch;

    // DeepSort deal boxs
    std::vector<DetectBox> D_box;
    Detection_2_SortBox(src, res_cars, D_box);
//    for (auto i : D_box) {
//        std::cout << i.x1 << " , " << i.y1 << "\t";
//    }
//    cout << endl << "++++++++++++++" << endl;

    DS_far.sort(src, D_box);
//    for (auto i : D_box) {
//        std::cout << i.x1 << " , " << i.y1 << "\t";
//    }
//    cout << endl << "+-----------------------+" << endl;

    id_process(src, D_box, res_cars);
//    for (auto i : D_box) {
//        std::cout << i.x1 << " , " << i.y1 << "\t";
//    }
//    cout << endl << "+===================================+" << endl;

//    show_deep_sort(src, D_box, true);

    // preprocess for second yolo model
    for (auto res_car : res_cars){
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
        src(rect_res_car).copyTo(roi_car);

        ++iter_res_car;
        if (roi_car.empty()) continue;
        img_num_batch.push_back(roi_car);
//        if (++valid_count >= 1) {
//            if (cv::imwrite("/home/hlf/Downloads/myFiles/test/far/_" + std::to_string(write_count) + ".jpg", roi_car_resized)) {
//                write_count++;
//            }
//            valid_count = 0;
//        }
        if (++num_batch_size < kNumDetectBatchSize && iter_res_car < (int)res_cars.size()) continue;

        // Preprocess
        cuda_batch_preprocess(img_num_batch, gpu_buffers_num[0],kInputW,
                              kInputH, stream_num, false);

        // Run inference
        infer(*context_num, stream_num, (void**)gpu_buffers_num,
              cpu_output_buffer_num, kBatchSize); // , kNumDetectBatchSize

        // NMS
        std::vector<std::vector<Detection>> res_batch_num;
        batch_nms(res_batch_num, cpu_output_buffer_num, (int)img_num_batch.size(),
                  kOutputSize, kConfThresh, kNmsThresh);

//        draw_bbox(img_num_batch, res_batch_num);
//        for (int i = 0; i < (int)res_batch_num.size(); i++) {
//            if (!res_batch_num[i].empty()) {
//                cv::imshow("sensor_far_view", img_num_batch[i]);
////                std::cout << res_batch_num[i][0].class_id << std::endl;
//                break;
//            }
//        }

        int iter_num_batch = 0;
        for (auto res_nums : res_batch_num) {
//            Detection& car_for_this_iter = res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch];
            cv::Rect rough_rect = get_rect(
                    src_raw, res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].bbox); // 用装甲板来代替raw图像中的车
            Detection real_res;
            real_res.conf = 0;
            real_res.class_id = 14;
            cv::Rect real_rect(0, 0, 0, 0);
            for (size_t m = 0; m < res_nums.size(); m++) {
//                std::cout << res_nums[m].class_id << std::endl;

                cv::Rect number_in_roi = get_rect(img_num_batch[iter_num_batch], res_nums[m].bbox);
                cv::Rect number_in_img = number_in_roi;
                number_in_img.x += rough_rect.x;
                number_in_img.y += rough_rect.y;
                // 选出正对雷达站的装甲板
                if (number_in_img.area() > real_rect.area()) {
                    real_rect = number_in_img;
                }
//                cv::putText(src_raw, std::to_string((int)res_nums[m].class_id), cv::Point(number_in_img.x, number_in_img.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                if (res_nums[m].conf > real_res.conf) {
                    real_res = res_nums[m];
                }
            }

            //根据装甲板ID修改车的ID（14是未识别出，12是红方未识别出，13是蓝方未识别出，其他编号一致）
            int is_blue, tracker_id;
            if ((int) real_res.class_id == 14) {  // yolo没识别出来
                if (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id >= 10000) {  //tracker追踪到了这个车
                    is_blue = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) / 100000 -1 == 1;
                    tracker_id = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) % 100000;
                    bool if_find = false;
                    for (int i = 0; i < 12; i++) {
                        if (id_tracker_far_table[i] == tracker_id) {
                            res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = (float)i;
                            if_find = true;
                            break;
                        }
                    }
                    if (!if_find) {
                        res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = float(12 + is_blue);
                    }
                } else {  //tracker没追踪到
                    is_blue = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id);
                    res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = float(12 + is_blue);
                }
            } else {  //yolo识别出来了
                if (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id >=
                    10000) {  //tracker追踪到了这个车
                    is_blue = int(res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) / 100000 -
                              1 == 1;
                    tracker_id =
                            int(res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) % 100000;
                    res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id =
                            real_res.class_id + float(6 * is_blue);
                    for (auto &j: id_tracker_far_table) {
                        if (j == tracker_id) j = 0;
                    }
                    id_tracker_far_table[int(
                            res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id)] = tracker_id;
                } else {  //tracker没追踪到
                    is_blue = int(res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id);
                    res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id =
                            real_res.class_id + float(6 * is_blue);
                }
            }

            for (int ii = 0; ii < 12; ii ++) {
                if (id_tracker_far_table[ii] != 0) {
                    std::cout << ii << ": " << id_tracker_far_table[ii] << "]  [";
                }
            }
            std::cout << endl;

            //根据装甲板的rect修改车的rect
            if (real_rect.area() > 0) {
                rough_rect = real_rect;
            } else {
                int change = rough_rect.width * 0.2;
                rough_rect.x += change;
                rough_rect.width -= 2 * change;
                change = rough_rect.height * 0.2;
                rough_rect.y += change;
                rough_rect.height -= 2 * change;
            }
            cv::rectangle(src_raw, rough_rect, color_table["Orange"], 2);
            res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].conf = real_res.conf;

            std::string str_2_print;
            switch ((int) res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4: {
                    str_2_print = std::string("red ") + std::to_string(
                            char((int)res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id + 1));
                    break;
                } case 5: {
                    str_2_print = "red guard ";
                    break;
                } case 6:
                case 7:
                case 8:
                case 9:
                case 10: {
                    str_2_print = std::string("blue ") + std::to_string(
                            char((int)res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id - 5));
                    break;
                } case 11: {
                    str_2_print = "blue guard ";
                    break;
                } case 12: {
                    str_2_print = "red X";
                    break;
                } case 13: {
                    str_2_print = "blue X";
                    break;
                }
                default: str_2_print = "===";
            }
//            str_2_print += "r_id: " + std::to_string(real_res.class_id);
            cv::putText(src_raw, str_2_print,cv::Point(rough_rect.x, rough_rect.y - 1),
                        cv::FONT_HERSHEY_PLAIN, 1,color_table["Green_light"], 1);

            radar_interfaces::msg::YoloPoint yolo_msg;
            yolo_msg.id = res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id;
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

            ++iter_num_batch;
        }

        std::vector<cv::Mat>().swap(img_num_batch);
        num_batch_size = 0;
        ++num_batch_count;
    }

    auto end_time = std::chrono::system_clock::now();
//    std::cout << "far inference time: " <<
//              std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
//              << "ms" << std::endl;

    // 将yolo识别的矩形发布
    if (rect_msg.data.size() != 0) {
        this->far_rect_publisher_->publish(rect_msg);
    } else {
        radar_interfaces::msg::YoloPoints rect_msg_4_none;
        rect_msg_4_none.text = "none";
        this->far_rect_publisher_->publish(rect_msg_4_none);
    }

    // 将yolo检图像测结果发布
    cv_bridge::CvImage yolo_result_img;
    yolo_result_img.encoding = "bgr8";
    yolo_result_img.header.stamp = this->now();
    yolo_result_img.image = src_raw;
    sensor_msgs::msg::Image yolo_result_msg;
    yolo_result_img.toImageMsg(yolo_result_msg);
    this->far_yolo_publisher_->publish(yolo_result_msg);

    cv::resize(src_raw, src_raw, cv::Size(), 0.9, 0.9, cv::INTER_LANCZOS4);
    cv::imshow("sensor_far_view", src_raw);
//    valid_count_far++;
//    if (valid_count_far >= 1) {
//        valid_count_far = 0;
//        write_count_far++;
//        cv::imwrite("/home/hlf/Downloads/myFiles/test/far/" + std::to_string(write_count_far) + ".jpg", src);
//    }
}


void ImageSubscriber::DeepsortCallback(sensor_msgs::msg::Image::SharedPtr msg) const {
    std::vector<cv::Mat> img_car_batch;
    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat src_raw;
    src.copyTo(src_raw);
    img_car_batch.push_back(src);

    // Preprocess
    cuda_batch_preprocess(img_car_batch, gpu_buffers_car[0],
                          kInputW, kInputH, stream_car, true);

    // Run inference
    auto start_time = std::chrono::system_clock::now();
    infer(*context_car, stream_car, (void**)gpu_buffers_car,
          cpu_output_buffer_car, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch_car;
    batch_nms(res_batch_car, cpu_output_buffer_car, (int)img_car_batch.size(),
              kOutputSize, kConfThresh, kNmsThresh);

    draw_bbox(img_car_batch, res_batch_car);
//    cv::resize(src, src, cv::Size(), 0.7, 0.7, cv::INTER_LANCZOS4);
    cv::imshow("sensor_far_view", src);

    std::vector<Detection> res_cars = res_batch_car[0];
    std::vector<DetectBox> D_box;
    Detection_2_SortBox(src_raw, res_cars, D_box);
    DS_close.sort(src, D_box);

    for (auto box : D_box) {
        cv::Point lt(box.x1, box.y1);
        cv::Point br(box.x2, box.y2);
        cv::rectangle(src_raw, lt, br, cv::Scalar(255, 0, 0), 1);
        //std::string lbl = cv::format("ID:%d_C:%d_CONF:%.2f", (int)box.trackID, (int)box.classID, box.confidence);
        //std::string lbl = cv::format("ID:%d_C:%d", (int)box.trackID, (int)box.classID);
        std::string lbl = cv::format("ID:%d_x:%f_y:%f", (int)box.trackID, (box.x1+box.x2)/2, (box.y1+box.y2)/2);
        cv::putText(src_raw, lbl, lt, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,0));
    }
    cv::imshow("sensor_close_view", src_raw);
}


void ImageSubscriber::CloseImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    std::vector<cv::Mat> img_car_batch;
    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat src_raw;
    src.copyTo(src_raw);
    img_car_batch.push_back(src);

    // Preprocess
    cuda_batch_preprocess(img_car_batch, gpu_buffers_car[0],
                          kInputW, kInputH, stream_car, true);

    // Run inference
    auto start_time = std::chrono::system_clock::now();
    infer(*context_car, stream_car, (void**)gpu_buffers_car,
          cpu_output_buffer_car, kBatchSize);

    // NMS
    std::vector<std::vector<Detection>> res_batch_car;
    batch_nms(res_batch_car, cpu_output_buffer_car, (int)img_car_batch.size(),
              kOutputSize, kConfThresh, kNmsThresh);

//    draw_bbox(img_car_batch, res_batch_car);
//    cv::resize(src, src, cv::Size(), 0.7, 0.7, cv::INTER_LANCZOS4);
//    cv::imshow("sensor_far_view", src);

    radar_interfaces::msg::YoloPoints rect_msg;
    int iter_res_car = 0, num_batch_size = 0, num_batch_count = 0;
    std::vector<Detection> res_cars = res_batch_car[0];
    std::vector<cv::Mat> img_num_batch;

    // DeepSort deal boxs
    std::vector<DetectBox> D_box;
    Detection_2_SortBox(src, res_cars, D_box);
//    for (auto i : D_box) {
//        std::cout << i.x1 << " , " << i.y1 << "\t";
//    }
//    cout << endl << "++++++++++++++" << endl;

    DS_close.sort(src, D_box);
//    for (auto i : D_box) {
//        std::cout << i.x1 << " , " << i.y1 << "\t";
//    }
//    cout << endl << "+-----------------------+" << endl;

    id_process(src, D_box, res_cars);
//    for (auto i : D_box) {
//        std::cout << i.x1 << " , " << i.y1 << "\t";
//    }
//    cout << endl << "+===================================+" << endl;

//    show_deep_sort(src, D_box, false);

    // preprocess for second yolo model
    for (auto res_car : res_cars){
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
        src(rect_res_car).copyTo(roi_car);

        ++iter_res_car;
        if (roi_car.empty()) continue;
        img_num_batch.push_back(roi_car);
        sampling_point_close++;
        if (sampling_point_close >= 10) {
            sampling_point_close = 0;
            write_count_close++;
            cv::imwrite("/home/hlf/Downloads/myFiles/test/close/" + std::to_string(write_count_close) + ".jpg", roi_car);
        }

        if (++num_batch_size < kNumDetectBatchSize && iter_res_car < (int)res_cars.size()) continue;

        // Preprocess
        cuda_batch_preprocess(img_num_batch, gpu_buffers_num[0],kInputW,
                              kInputH, stream_num, false);

        // Run inference
        infer(*context_num, stream_num, (void**)gpu_buffers_num,
              cpu_output_buffer_num, kBatchSize); // , kNumDetectBatchSize

        // NMS
        std::vector<std::vector<Detection>> res_batch_num;
        batch_nms(res_batch_num, cpu_output_buffer_num, (int)img_num_batch.size(),
                  kOutputSize, kConfThresh, kNmsThresh);

//        draw_bbox(img_num_batch, res_batch_num);
//        for (int i = 0; i < (int)res_batch_num.size(); i++) {
//            if (!res_batch_num[i].empty()) {
//                cv::imshow("sensor_far_view", img_num_batch[i]);
////                std::cout << res_batch_num[i][0].class_id << std::endl;
//                break;
//            }
//        }

        int iter_num_batch = 0;
        for (auto res_nums : res_batch_num) {
//            Detection& car_for_this_iter = res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch];
            cv::Rect rough_rect = get_rect(
                    src_raw, res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].bbox); // 用装甲板来代替raw图像中的车
            Detection real_res;
            real_res.conf = 0;
            real_res.class_id = 14;
            cv::Rect real_rect(0, 0, 0, 0);
            for (size_t m = 0; m < res_nums.size(); m++) {
//                std::cout << res_nums[m].class_id << std::endl;

                cv::Rect number_in_roi = get_rect(img_num_batch[iter_num_batch], res_nums[m].bbox);
                cv::Rect number_in_img = number_in_roi;
                number_in_img.x += rough_rect.x;
                number_in_img.y += rough_rect.y;
                // 选出正对雷达站的装甲板
                if (number_in_img.area() > real_rect.area()) {
                    real_rect = number_in_img;
                }
//                cv::putText(src_raw, std::to_string((int)res_nums[m].class_id), cv::Point(number_in_img.x, number_in_img.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                if (res_nums[m].conf > real_res.conf) {
                    real_res = res_nums[m];
                }
            }

            //根据装甲板ID修改车的ID（14是未识别出，12是红方未识别出，13是蓝方未识别出，其他编号一致）
//            id_correct(real_res.class_id,
//                       res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id, false);
            int is_blue, tracker_id;
            if ((int) real_res.class_id == 14) {  // yolo没识别出来
                if (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id >= 10000) {  //tracker追踪到了这个车
                    is_blue = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) / 100000 -1 == 1;
                    tracker_id = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) % 100000;
                    bool if_find = false;
                    for (int i = 0; i < 12; i++) {
                        if (id_tracker_close_table[i] == tracker_id) {
                            res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = (float)i;
                            if_find = true;
                            break;
                        }
                    }
                    if (!if_find) {
                        res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = float(12 + is_blue);
                    }
                } else {  //tracker没追踪到
                    is_blue = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id);
                    res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = float(12 + is_blue);
                }
            } else {  //yolo识别出来了
                if (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id >= 10000) {  //tracker追踪到了这个车
                    is_blue = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) / 100000 -1 == 1;
                    tracker_id = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) % 100000;
                    res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = real_res.class_id + float(6 * is_blue);
                    for (auto &j : id_tracker_close_table) {
                        if (j == tracker_id) j = 0;
                    }
                    id_tracker_close_table[int(res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id)] = tracker_id;
                } else {  //tracker没追踪到
                    is_blue = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id);
                    res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = real_res.class_id + float(6 * is_blue);
                }
            }

//            if (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id >= 10000) {  //tracker追踪到了这个车
//                is_blue = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) / 100000 -1 == 1;
//                int tracker_id = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) % 100000;
//                bool if_find = false;
//                for (int i = 0; i < 12; i++) {
//                    if (id_tracker_close_table[i] == tracker_id) {
//                        res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = (float)i;
//                        if_find = true;
//                        break;
//                    }
//                }
//                if (!if_find) {
//                    if ((int) real_res.class_id == 14) {
//                        res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = float(12 + is_blue);
//                    } else {
//                        res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = real_res.class_id + float(6 * is_blue);
//                        id_tracker_close_table[int(res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id)] = tracker_id;
//                    }
//                }
//
//            } else {  //tracker没追踪到
//                is_blue = int (res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id);
//                if ((int) real_res.class_id == 14) {
//                    res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = float(12 + is_blue);
//                } else {
//                    res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id = real_res.class_id + float(6 * is_blue);
//                }
//            }

            for (int ii = 0; ii < 12; ii++) {
                if (id_tracker_close_table[ii] != 0) {
                    std::cout << " [" << ii+1 << ": " << id_tracker_close_table[ii] << "] ";
                }
                if (ii >= 11) std::cout << " " << std::endl;
            }

            //根据装甲板的rect修改车的rect
            if (real_rect.area() > 0) {
                rough_rect = real_rect;
            } else {
                int change = rough_rect.width * 0.2;
                rough_rect.x += change;
                rough_rect.width -= 2 * change;
                change = rough_rect.height * 0.2;
                rough_rect.y += change;
                rough_rect.height -= 2 * change;
            }
            cv::rectangle(src_raw, rough_rect, color_table["Orange"], 2);
            res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].conf = real_res.conf;

            std::string str_2_print;
            switch ((int) res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4: {
                    str_2_print = std::string("red ") + std::to_string(
                            char((int)res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id + 1));
                    break;
                } case 5: {
                    str_2_print = "red guard ";
                    break;
                } case 6:
                case 7:
                case 8:
                case 9:
                case 10: {
                    str_2_print = std::string("blue ") + std::to_string(
                            char((int)res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id - 5));
                    break;
                } case 11: {
                    str_2_print = "blue guard ";
                    break;
                } case 12: {
                    str_2_print = "red X";
                    break;
                } case 13: {
                    str_2_print = "blue X";
                    break;
                }
                default: str_2_print = "===";
            }
            cv::putText(src_raw, str_2_print,cv::Point(rough_rect.x, rough_rect.y - 1),
                        cv::FONT_HERSHEY_PLAIN, 1,color_table["Green_light"], 1);

            radar_interfaces::msg::YoloPoint yolo_msg;
            yolo_msg.id = res_cars[num_batch_count * kNumDetectBatchSize + iter_num_batch].class_id;
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

            ++iter_num_batch;
        }

        std::vector<cv::Mat>().swap(img_num_batch);
        num_batch_size = 0;
        ++num_batch_count;
    }

    auto end_time = std::chrono::system_clock::now();
//    std::cout << "close inference time: " <<
//              std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
//              << "ms" << std::endl;

    // 将yolo识别的矩形发布
    if (rect_msg.data.size() != 0) {
        this->close_rect_publisher_->publish(rect_msg);
    } else {
        radar_interfaces::msg::YoloPoints rect_msg_4_none;
        rect_msg_4_none.text = "none";
        this->close_rect_publisher_->publish(rect_msg_4_none);
    }

    // 将yolo检图像测结果发布
    cv_bridge::CvImage yolo_result_img;
    yolo_result_img.encoding = "bgr8";
    yolo_result_img.header.stamp = this->now();
    yolo_result_img.image = src_raw;
    sensor_msgs::msg::Image yolo_result_msg;
    yolo_result_img.toImageMsg(yolo_result_msg);
    this->close_yolo_publisher_->publish(yolo_result_msg);

    cv::resize(src_raw, src_raw, cv::Size(), 0.9, 0.9, cv::INTER_LANCZOS4);
    cv::imshow("sensor_close_view", src_raw);
}

//// snesor_close callback for tacker
//cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
//auto model = cv::createBackgroundSubtractorMOG2();
//
//void ImageSubscriber::CloseImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
//    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
//    cv::Mat src_raw;
//    src.copyTo(src_raw);
//
//    cv::Mat fgmk;
//    model->apply(src_raw, fgmk);
//    cv::morphologyEx(fgmk, fgmk, cv::MORPH_OPEN, kernel);
//
//    std::vector<std::vector<cv::Point>> contours;
//    cv::findContours(fgmk, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//    for (auto c : contours) {
//        if (cv::contourArea(c) < 1500) continue;
//        cv::Rect rec;
//        rec = cv::boundingRect(c);
//        cv::rectangle(src_raw, rec, cv::Scalar(0, 255, 0), 2);
//    }
//
//    cv::imshow("sensor_far_view", fgmk);
//    cv::imshow("sensor_close_view", src_raw);
//
//}



void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer,
                     float** gpu_output_buffer, float** cpu_output_buffer, bool if_car) {
    assert(engine->getNbBindings() == 2);  // deprecated for TensorRT version not correspond
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    if (if_car) {
        CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));
    } else {
        CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kNumDetectBatchSize * 3 * kInputH * kInputW * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer, kNumDetectBatchSize * kOutputSize * sizeof(float)));
    }


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
    std::string num_wts_name = data_dir + "/weights/num_24_1300.wts";
    std::string num_engine_name = data_dir + "/engine/num_24_1300.engine";
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

void Detection_2_SortBox(cv::Mat& img, const std::vector<Detection>& src_box, std::vector<DetectBox>& dst_box) {
    // src:center_x, center_y, w, h    dst:lt,rb
    if (!dst_box.empty()) dst_box.clear();
    for (auto i : src_box) {
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
        temp_box.x1 = (float)rect_res_car.x;
        temp_box.y1 = (float)rect_res_car.y;
        temp_box.x2 = (float)rect_res_car.x + (float)rect_res_car.width;
        temp_box.y2 = (float)rect_res_car.y + (float)rect_res_car.height;
        temp_box.confidence = i.conf;
        temp_box.classID = i.class_id;
        temp_box.trackID = 0;
        dst_box.push_back(temp_box);
    }
}

void id_process(cv::Mat& img, const std::vector<DetectBox>& D_box, std::vector<Detection>& res_cars) {
    // D_box:lt,rb    res_cars:center_x, center_y, w, h
    // 1xxxxx stand for red, 2xxxxx stands for blue, xxxxx stand for tracker ID
    for (size_t i = 0; i < D_box.size(); i++) {
        cv::Point lt((int)D_box[i].x1, (int)D_box[i].y1), rb((int)D_box[i].x2, (int)D_box[i].y2);
        cv::Rect D_rect(lt, rb);
        for (size_t j = 0; j < res_cars.size(); j++) {
            cv::Rect R_rect = get_rect(img, res_cars[j].bbox);
            if (D_rect.contains(cv::Point(R_rect.x + R_rect.width/2, R_rect.y + R_rect.height/2) )) {
                res_cars[j].class_id = ((res_cars[j].class_id + 1) * 100000) + D_box[i].trackID;
                break;
            }
        }
    }
}

void show_deep_sort(cv::Mat& src, std::vector<DetectBox> & target_box, bool if_far) {
    for (auto box : target_box) {
        cv::Point lt(box.x1, box.y1);
        cv::Point br(box.x2, box.y2);
        cv::rectangle(src, lt, br, cv::Scalar(255, 0, 0), 1);
        //std::string lbl = cv::format("ID:%d_C:%d_CONF:%.2f", (int)box.trackID, (int)box.classID, box.confidence);
        //std::string lbl = cv::format("ID:%d_C:%d", (int)box.trackID, (int)box.classID);
        std::string lbl = cv::format("ID:%d", (int)box.trackID);
        cv::putText(src, lbl, lt, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,0));
    }
    if (if_far) {
        cv::imshow("sensor_close_view", src);
    } else {
        cv::imshow("sensor_far_view", src);
    }
}

void id_correct(float id_yolo, float& id_dst, bool if_far) {
    int is_blue, tracker_id, *id_tracker_table;
//    if (if_far) id_tracker_table = id_tracker_far_table;
//    else id_tracker_table = id_tracker_close_table;
    if ((int) id_yolo == 14) {  // yolo识别没出来
        if (id_dst >= 10000) {  //tracker追踪到了这个车
            is_blue = int (id_dst) / 100000 -1 == 1;
            tracker_id = int (id_dst) % 100000;
            bool if_find = false;
            for (int i = 0; i < 12; i++) {
                if (id_tracker_close_table[i] == tracker_id) {
                    id_dst = (float)i;
                    if_find = true;
                    break;
                }
            }
            if (!if_find) {
                id_dst = float(12 + is_blue);
            }
        } else {  //tracker没追踪到
            is_blue = int (id_dst);
            id_dst = float(12 + is_blue);
        }
    } else {  //yolo识别出来了
        if (id_dst >= 10000) {  //tracker追踪到了这个车
            is_blue = int (id_dst) / 100000 -1 == 1;
            tracker_id = int (id_dst) % 100000;
            id_dst = id_dst + float(6 * is_blue);
            for (int j = 0; j < 12; j++) {
                if (id_tracker_close_table[j] == tracker_id) id_tracker_close_table[j] = 0;
            }
            std::cout << id_tracker_close_table[int(id_dst)] << std::endl;
            id_tracker_close_table[int(id_dst)] = tracker_id;
        } else {  //tracker没追踪到
            is_blue = int (id_dst);
            id_dst = id_dst + float(6 * is_blue);
        }
    }
}









