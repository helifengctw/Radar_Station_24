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
#include "radar_interfaces/srv/pnp_result.hpp"
#include "robot_serial/msg/map_points.hpp"
#include "robot_serial/msg/double_hurt.hpp"
#include "robot_serial/msg/gamestatus.hpp"
#include "robot_serial/msg/double_info.hpp"
#include "robot_serial/msg/event.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;
using std::placeholders::_1;

// BGR
map<string, Scalar> color_table = {
        {"White", Scalar(0xfa, 0xfa, 0xff)},
        {"Black", Scalar(0x00, 0x06, 0x06)},
        {"Red", Scalar(0x00, 0x00, 0xff)},
        {"Green", Scalar(0x06, 0xff, 0x06)},
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
    rclcpp::Subscription<radar_interfaces::msg::Point>::SharedPtr pickup_information_subscription_;
    rclcpp::Subscription<robot_serial::msg::Gamestatus>::SharedPtr game_status_subscription_;
    rclcpp::Subscription<robot_serial::msg::DoubleInfo>::SharedPtr double_info_subscription_;
    rclcpp::Subscription<robot_serial::msg::Event>::SharedPtr event_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr world_point_publisher_;
    rclcpp::Publisher<robot_serial::msg::MapPoints>::SharedPtr serial_world_point_publisher_;
    rclcpp::Publisher<robot_serial::msg::DoubleHurt>::SharedPtr double_hurt_cmd_publisher_;
    rclcpp::Client<radar_interfaces::srv::PnpResult>::SharedPtr Pnp_result_client_;
    std::shared_ptr<radar_interfaces::srv::PnpResult::Request> pnp_request;

    void Pnp_resultCallback(rclcpp::Client<radar_interfaces::srv::PnpResult>::SharedFuture response);

    void TimerCallback();
    void far_distPointCallback(radar_interfaces::msg::DistPoints::SharedPtr);
    void close_distPointCallback(radar_interfaces::msg::DistPoints::SharedPtr);
    void game_status_Callback(robot_serial::msg::Gamestatus::SharedPtr);
    void double_info_Callback(robot_serial::msg::DoubleInfo::SharedPtr);
    void event_Callback(robot_serial::msg::Event::SharedPtr);
    void pickup_infoCallback(radar_interfaces::msg::Point::SharedPtr);
    void load_param();

    void draw_point_on_map(const radar_interfaces::msg::Point &point, Mat &image, const string& txt_color);
    void remove_duplicate();
    bool is_enemy_car(uint8_t);
    double calculate_dist(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b);
    double Point2PointDist(const radar_interfaces::msg::Point &a, const Point2d &b);
    radar_interfaces::msg::Point calculate_relative_codi(const Point3f &guard, const radar_interfaces::msg::Point &enemy, uint8_t priority_id);
    Point2d calculate_pixel_codi(const radar_interfaces::msg::Point &point);
    Point2d calculate_pixel_text_codi(const radar_interfaces::msg::Point &point);
    void add_grid(cv::Mat &src);
    bool check_same_color(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b);

    cv::Mat far_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
    Mat far_R = Mat::eye(3, 3, CV_64FC1);
    Mat far_T = Mat::ones(3, 1, CV_64FC1);
    cv::Mat close_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
    Mat close_R = Mat::eye(3, 3, CV_64FC1);
    Mat close_T = Mat::zeros(3, 1, CV_64FC1);
    Mat far_invR = Mat::zeros(3, 3, CV_64FC1), far_invM = Mat::zeros(3, 3, CV_64FC1);
    Mat close_invR = Mat::zeros(3, 3, CV_64FC1), close_invM = Mat::zeros(3, 3, CV_64FC1);
    int X_shift = 0, Y_shift = 0, red_or_blue = 0; // 0 : red, 1 : blue
    vector<Point2f> left_region = {Point(0, 0), Point(0, img_show_height),
                                   Point(168, img_show_height), Point(278, 0)};
    vector<Point2f> right_region = {Point(278, 0), Point(168, img_show_height),
                                    Point(img_show_width, img_show_height), Point(img_show_width, 0)};

    double field_width, field_height, imgCols, imgRows, dist_threshold;
    cv::Point2d center_far, center_close;
    int img_show_width, img_show_height;
    std::string small_map_png_path;

    Mat small_map, small_map_copy;
    radar_interfaces::msg::Points far_points, close_points, result_points, filtered_result_points;
    robot_serial::msg::MapPoints serial_world_points;
    robot_serial::msg::DoubleHurt double_hurt_msg;
    bool trigger_once = false, small_energy_enable = false, big_energy_enable = false;
    int detected_dangerous_enemy_count = 0;
    uint16_t remain_time = 1000, trigger_time = 0;
    uint8_t double_hurt_chance = 0x00, exerting = 0x00, used_chance = 0x00;
    uint8_t last_game_progress = 0x00, game_progress = 0x00;
    cv::Point2f mouse_point = cv::Point2f(0.0, 0.0);
};

#endif
