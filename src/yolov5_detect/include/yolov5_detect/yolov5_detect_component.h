#ifndef YOLOV5_DECTECT_COMPONENT_HPP
#define YOLOV5_DECTECT_COMPONENT_HPP

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
#include "deepsort.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>

using namespace nvinfer1;
using std::placeholders::_1;


namespace yolov5_detect{
    std::map<std::string, cv::Scalar> color_table = {
            {"White", cv::Scalar(0xff, 0xff, 0xff)},
            {"Red", cv::Scalar(0x00, 0x00, 0xff)},
            {"Blue", cv::Scalar(0xff, 0x90, 0x1e)},
            {"Green", cv::Scalar(0x32, 0xCD, 0x32)}, // Blue
            {"Yellow", cv::Scalar(0x00, 0xff, 0xff)},
            {"Antique_white", cv::Scalar(0xD7, 0xEB, 0xFA)},
            {"Purple", cv::Scalar(0xF0, 0x20, 0xA0)}, //
            {"Blue_green", cv::Scalar(0xD4, 0xFF, 0x7F)}, //
            {"Brick_red", cv::Scalar(0x1F, 0x66, 0x9c)}
    };

    const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
    const int kNumDetectBatchSize = 4;
    Logger gLogger_;
    const std::string Deepsort_engine = "/home/hlf/Desktop/deep_sort/yolov5_deepsort_ws/src/yolo_sort/data/deepsort.engine";


    class Yolov5Detector : public rclcpp::Node{
    public:
        Yolov5Detector(const rclcpp::NodeOptions &options);
        ~Yolov5Detector();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
        void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        rclcpp::Publisher<radar_interfaces::msg::YoloPoints>::SharedPtr rect_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yolo_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr deepsort_publisher_;

        long int write_count = 0, sampling_point = 0;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_time;
        int id_tracker_table[12] = {0}, id_tracker_last_table[12] = {0}, progress[12] = {0};
        std::array<std::deque<cv::Point2f>, 12> point_stack;
        std::array<std::vector<cv::Point2f>, 6> point_filter_red_stack, point_filter_blue_stack;
        float conf_table[12] = {0};
        std::string camera_name, win_name;
        float overlap_threshold = 0.1;
        double time_threshold = 90.0;
        int show_count_threshold = 1, show_count = 0, show_by_cv_or_msg;
        radar_interfaces::msg::YoloPoints last_yolo_point_list, show_yolo_point_list;

        DeepSort deepsort_;

        float* gpu_buffers_car[2];
        float* gpu_buffers_num[2];
        float* cpu_output_buffer_car = nullptr;
        float* cpu_output_buffer_num = nullptr;
        cudaStream_t stream_car;
        cudaStream_t stream_num;
        IExecutionContext* context_car = nullptr;
        IExecutionContext* context_num = nullptr;
        std::string car_engine_name;
        std::string num_engine_name;
        IRuntime* runtime = nullptr;
        ICudaEngine* car_engine = nullptr;
        ICudaEngine* num_engine = nullptr;

        void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer,
                             float** gpu_output_buffer, float** cpu_output_buffer, bool if_car);
        void infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers,
                   float* output, int batchsize);
        void serialize_engine(unsigned int max_batchsize, bool& is_p6, float& gd, float& gw,
                              std::string& wts_name, std::string& engine_name);
        void deserialize_engine(std::string& engine_name, IRuntime** runtime,
                                ICudaEngine** engine, IExecutionContext** context);
        bool FilePreparation(bool if_serialize_engine_car, bool if_serialize_engine_num, bool is_p6,
                             std::string * return_car_engine, std::string * return_num_engine);
        void show_deep_sort(cv::Mat& src, std::vector<DetectBox> & target_box);
        void load_parameters();

        /* custom utils function */
        void Detection_2_SortBox(cv::Mat& img, const std::vector<Detection>& src_box, std::vector<DetectBox> & dst_box);
        void id_process(cv::Mat& img, const std::vector<DetectBox>& D_box, std::vector<Detection>& detected_cars);
        void id_correct(float id_yolo, float& id_dst, bool if_far);
        int encoding2mat_type(const std::string & encoding);
        void filter_points(const std::array<std::deque<cv::Point2f>, 12>& p_stack,
                           std::array<std::vector<cv::Point2f>, 6>& red_stack,
                           std::array<std::vector<cv::Point2f>, 6>& blue_stack);
        int classify(const std::array<std::vector<cv::Point2f>, 6>& category_stack, const cv::Point2f& point);
        bool check_same_id(radar_interfaces::msg::YoloPoints yoloPointList, int id, int threshold);
        void add_time_in_img(cv::Mat& img, double time);
        void show_yolo_result_in_img(radar_interfaces::msg::YoloPoints filter_yolo_point_list_msg, cv::Mat& src_raw);
        radar_interfaces::msg::YoloPoints remove_duplicate(radar_interfaces::msg::YoloPoints& yoloPointList);
    };
}

#endif