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
#include "radar_interfaces/msg/point.hpp"
#include "radar_interfaces/msg/yolo_points.hpp"
#include "deepsort.h"
#include "KM_match.h"

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

    struct track_element {
        bool obsoleted;
        int track_id;
        int class_id;
        float conf;
        cv::Rect rect;
        cv::Rect last_rect;
        double time;
        double last_time;
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

        rclcpp::Publisher<radar_interfaces::msg::YoloPoints>::SharedPtr rect_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yolo_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr deepsort_publisher_;

        void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void load_parameters();
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

        /* custom utils function */
        void tune_img(cv::Mat& img);
        void Detection_2_SortBox(cv::Mat& img, const std::vector<Detection>& src_box, std::vector<DetectBox> & dst_box);
        bool check_same_id(radar_interfaces::msg::YoloPoints yoloPointList, int id, int threshold);
        void add_time_in_img(cv::Mat& img, double time);
        void show_yolo_result_in_img(radar_interfaces::msg::YoloPoints filter_yolo_point_list_msg, cv::Mat& src_raw);
        radar_interfaces::msg::YoloPoints remove_duplicate(radar_interfaces::msg::YoloPoints& yoloPointList);
        bool check_same_color(const radar_interfaces::msg::YoloPoint &a, const radar_interfaces::msg::YoloPoint &b);

        long int write_count = 0, sampling_point = 0;
        std::string camera_name, win_name;
        float overlap_threshold = 0.1, duplicate_threshold = 0.1;
        double time_threshold = 90.0, pro_start_time = 0, current_time_ms = 0.0, last_frame_time_ms = 0.0,
        light_gain = 0.0, saturation_gain = 0.0;
        int show_count_threshold = 0, show_count = 0, show_by_cv_or_msg = 0, call_count = 0;
        radar_interfaces::msg::YoloPoints last_yolo_point_list, show_yolo_point_list;
        bool rgb_or_bayer = false;

        std::vector<track_element> tracker;
        bool init_tracker(const radar_interfaces::msg::YoloPoints&);
        double calc_dist(int x1, int y1, int x2, int y2);
        std::vector<std::pair<int, radar_interfaces::msg::YoloPoint>> prediction_points;
        std::vector<std::vector<int>> cost_matrix, sorted_cost_matrix;
        radar_interfaces::msg::YoloPoints sorted_points;
        bool filter_tracker_and_predict();
        void calc_cost(radar_interfaces::msg::YoloPoints &now_points);
        void update_tracker(radar_interfaces::msg::YoloPoints &now_points);

        DeepSort deepsort_;

        float* gpu_buffers_car[2], *gpu_buffers_num[2];
        float* cpu_output_buffer_car = nullptr, * cpu_output_buffer_num = nullptr;
        cudaStream_t stream_car, stream_num;
        IExecutionContext* context_car = nullptr, *context_num = nullptr;
        std::string car_engine_name, num_engine_name;
        IRuntime* runtime = nullptr;
        ICudaEngine* car_engine = nullptr, *num_engine = nullptr;
    };
}

#endif