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
//#include "deepsort.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cmath>
#include <memory>

using namespace nvinfer1;
using std::placeholders::_1;

class Yolov5_Detect_Node : public rclcpp::Node {
public:
    Yolov5_Detect_Node() : Node("image_subscriber") {
        far_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/sensor_far/image_raw", 10, std::bind(&Yolov5_Detect_Node::FarImageCallback, this, _1));
        close_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/sensor_close/image_raw", 10, std::bind(&Yolov5_Detect_Node::CloseImageCallback, this, _1));
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr far_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr close_subscription_;

    void FarImageCallback(sensor_msgs::msg::Image::SharedPtr) const;
    void CloseImageCallback(sensor_msgs::msg::Image::SharedPtr) const;
    int save_threshold = 20;
};

long int write_count_far = 0, valid_count_far = 0;
long int write_count_close = 0, valid_count_close = 0;
void tune_img(cv::Mat& src) {
    cv::Mat hsvImg;

    cv::cvtColor(src, hsvImg, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImg, hsvChannels);
    hsvChannels[1] *= 2.0;
    hsvChannels[2] *= 1.6;

    cv::merge(hsvChannels, hsvImg);
    cv::cvtColor(hsvImg, src, cv::COLOR_HSV2BGR);
}

int main(int argc, char** argv) {
    cv::namedWindow("sensor_far_view");
    cv::namedWindow("sensor_close_view");
    cv::startWindowThread();

    //Read images from camera
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Yolov5_Detect_Node>());
    rclcpp::shutdown();

    return 0;
}


void Yolov5_Detect_Node::FarImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    tune_img(src);
    if (++valid_count_far >= save_threshold) {
        if (cv::imwrite("/home/hlf/Downloads/myFiles/img_car/tuned_cs/far/_" + std::to_string(write_count_far) + ".jpg", src)) {
            write_count_far++;
        }
        valid_count_far = 0;
    }

    cv::resize(src, src, cv::Size(), 0.7, 0.7, cv::INTER_LANCZOS4);
    cv::imshow("sensor_far_view", src);
}


void Yolov5_Detect_Node::CloseImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    tune_img(src);
    if (++valid_count_close >= save_threshold) {
        if (cv::imwrite("/home/hlf/Downloads/myFiles/img_car/tuned_cs/close/_" + std::to_string(write_count_close) + ".jpg", src)) {
            write_count_close++;
        }
        valid_count_close = 0;
    }

    cv::resize(src, src, cv::Size(), 0.7, 0.7, cv::INTER_LANCZOS4);
    cv::imshow("sensor_close_view", src);
}

