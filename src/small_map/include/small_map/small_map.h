#ifndef SMALL_MAP_H
#define SMALL_MAP_H

#include <cstdio>
#include <opencv2/opencv.hpp>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "radar_interfaces/msg/points.hpp"
#include "radar_interfaces/msg/dist_points.hpp"
#include "radar_interfaces/msg/pnp_result.hpp"
#include "radar_interfaces/msg/battle_color.hpp"
#include "radar_interfaces/msg/mark_data.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;
using std::placeholders::_1;

// BGR
map<string, Scalar> color_table = {
        {"White", Scalar(0xfa, 0xfa, 0xff)},
        {"Red", Scalar(0x00, 0x00, 0xff)},
        {"Green_light", Scalar(0xcc, 0xff, 0xcc)},
        {"Orange", Scalar(0x00, 0x8c, 0xff)},
        {"Blue", Scalar(0xff, 0x90, 0x1e)},
        {"Yellow", Scalar(0x11, 0xff, 0xff)}
};


class SmallMap : public rclcpp::Node {
public:
    SmallMap(string name);
    ~SmallMap();

private:
    rclcpp::Subscription<radar_interfaces::msg::DistPoints>::SharedPtr far_distant_point_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::DistPoints>::SharedPtr close_distant_point_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::PnpResult>::SharedPtr Pnp_result_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::PnpResult>::SharedPtr Icp_result_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr world_point_publisher_;


    void TimerCallback();
    void far_distPointCallback(radar_interfaces::msg::DistPoints::SharedPtr);
    void close_distPointCallback(radar_interfaces::msg::DistPoints::SharedPtr);
    void Pnp_resultCallback(radar_interfaces::msg::PnpResult::SharedPtr);
    void Icp_resultCallback(radar_interfaces::msg::PnpResult::SharedPtr);

    void load_param();

    void draw_point_on_map(const radar_interfaces::msg::Point &point, Mat &image);
    void remove_duplicate();
    bool is_enemy_car(uint8_t);
    double calculate_dist(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b);
    double Point2PointDist(const radar_interfaces::msg::Point &a, const Point3f &b);
    radar_interfaces::msg::Point calculate_relative_codi(const Point3f &guard, const radar_interfaces::msg::Point &enemy, uint8_t priority_id);
    Point2f calculate_pixel_codi(const radar_interfaces::msg::Point &point);
    Point2f calculate_pixel_text_codi(const radar_interfaces::msg::Point &point);
    void add_grid(cv::Mat &src);

    cv::Mat far_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
    Mat far_R = Mat::eye(3, 3, CV_64FC1);
    Mat far_T = Mat::ones(3, 1, CV_64FC1);
    cv::Mat close_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
    Mat close_R = Mat::eye(3, 3, CV_64FC1);
    Mat close_T = Mat::zeros(3, 1, CV_64FC1);
    Mat far_invR, close_invR, far_invM, close_invM;
    int X_shift = 0, Y_shift = 0, red_or_blue = 0; // 0 : red, 1 : blue
    vector<Point2f> left_region = {Point(0, 0), Point(0, img_show_height),
                                   Point(168, img_show_height), Point(278, 0)};
    vector<Point2f> right_region = {Point(278, 0), Point(168, img_show_height),
                                    Point(img_show_width, img_show_height), Point(img_show_width, 0)};

    double field_width = 15, field_height = 28;
    int img_show_width = 15*30, img_show_height = 28*30;
    double imgCols = 1920.0, imgRows = 1200.0;
    double dist_threshold = 0.8;

    Mat small_map, small_map_copy;
    radar_interfaces::msg::Points far_points, close_points, result_points, filtered_result_points;
    int tracker_id_lock_list[6] = {0};
    bool mouse_click_flag = false;
    cv::Point2f mouse_point = cv::Point2f(0.0, 0.0);
};

#endif