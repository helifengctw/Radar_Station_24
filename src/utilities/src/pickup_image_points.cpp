#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "radar_interfaces/msg/yolo_point.hpp"
#include "radar_interfaces/msg/yolo_points.hpp"
#include "radar_interfaces/msg/point.hpp"
#include "radar_interfaces/msg/points.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

using namespace std;
using namespace cv;
using std::placeholders::_1;

class PointsPickUp : public rclcpp::Node {
public:
    PointsPickUp(const std::string& name) : Node(name) {
        far_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/sensor_far/raw/image", 10, std::bind(&PointsPickUp::FarImgCallback, this, _1));
        close_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/sensor_close/raw/image", 10, std::bind(&PointsPickUp::CloseImgCallback, this, _1));
        far_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>(
                "/sensor_far/calibration", 1);
        close_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>(
                "/sensor_close/calibration", 1);
        inform_small_map_publisher_ = this->create_publisher<radar_interfaces::msg::Point>(
                "pickup_information", 1);
    }
    void send_far_points();
    void send_close_points();
    void send_small_map_information();

        private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr far_img_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr close_img_subscription_;

    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr far_point_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr close_point_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Point>::SharedPtr inform_small_map_publisher_;


    void FarImgCallback(sensor_msgs::msg::Image::SharedPtr) const;
    void CloseImgCallback(sensor_msgs::msg::Image::SharedPtr) const;
};

void far_mouse_callback(int event, int x, int y, int flags, void* param);
void close_mouse_callback(int event, int x, int y, int flags, void* param);


Mat far_src, close_src, points_set_img;
float smaller = 0.6;
int pick_far_count = 0, pick_close_count = 0, total_count = 5;
Scalar GREEN(0, 255, 0);
bool far_receiving = true, close_receiving = true;
radar_interfaces::msg::Points far_points_msg, close_points_msg;
radar_interfaces::msg::Point small_map_info_msg;
vector<Point2f> far_points_list, close_points_list;

int main(int argc, char ** argv)
{
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto PPU_node = std::make_shared<PointsPickUp>("point_pick_up");

    points_set_img = imread("/home/hlf/Desktop/radar24_ws/src/utilities/data/points_set.png");
    namedWindow("far");
    namedWindow("close");
    namedWindow("points_set_show");
    setMouseCallback("far", far_mouse_callback, &far_src);
    setMouseCallback("close", close_mouse_callback, &close_src);
    cv::imshow("points_set_show", points_set_img);
    rclcpp::Rate loop_rate(100);

    while(true) {
        if (far_receiving) {
            rclcpp::spin_some(PPU_node);
        }
        if (!far_src.empty() && !close_src.empty()) {
            imshow("far", far_src);
            int k = waitKey(1);
            if (k == 27) {
                break;  // press esc to shut down the window
            }
        }
    }
    cout << "please enter the number of wanted far " << total_count << " image points and world points number:" << endl;
    for (int i = 0; i < total_count; i++) {
        int j = 0, k = 0;
        cout << "----the " << i << "th one:" << endl << "image--";
        cin >> j;
        cout << "world--";
        cin >> k;
        cout << "pick : " << far_points_list[j] << " and accordingly " << k << "th world points" << endl;
        radar_interfaces::msg::Point point;
        point.x = (float)far_points_list[j].x;
        point.y = (float)far_points_list[j].y;
        point.id = k;
        far_points_msg.data.push_back(point);
    }
    PPU_node->send_far_points();

    while(true) {
        if (close_receiving) {
            rclcpp::spin_some(PPU_node);
        }
        if (!far_src.empty() && !close_src.empty()) {
            imshow("close", close_src);
            int k = waitKey(1);
            if (k == 27) {
                break;  // press esc to shut down the window
            }
        }
    }
    cout << "please enter the number of wanted close " << total_count << " points :" << endl;
    for (int i = 0; i < total_count; i++) {
        int j = 0, k = 0;
        cout << "----the " << i << "th one:" << endl << "image--";
        cin >> j;
        cout << "world--";
        cin >> k;
        cout << "pick : " << close_points_list[j] << " and accordingly " << k << "th world points" << endl;
        radar_interfaces::msg::Point point;
        point.x = (float)close_points_list[j].x;
        point.y = (float)close_points_list[j].y;
        point.id = k;
        close_points_msg.data.push_back(point);
    }
    PPU_node->send_close_points();
    PPU_node->send_small_map_information();

    destroyAllWindows();
}

void PointsPickUp::FarImgCallback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat src;
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
    cv::resize(src, src, cv::Size(src.cols*smaller, src.rows*smaller));
    src.copyTo(far_src);
}

void PointsPickUp::CloseImgCallback(sensor_msgs::msg::Image::SharedPtr msg) const {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat src;
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
    cv::resize(src, src, cv::Size(src.cols*smaller, src.rows*smaller));
    src.copyTo(close_src);
}

void far_mouse_callback(int event, int x, int y, int flags, void* param) {
    if (event == EVENT_LBUTTONDOWN) {
        far_receiving = false;
        cout << "far count --" << pick_far_count <<"-- : ( " << x/smaller << " , " << y/smaller << " ) " << endl;
        far_points_list.emplace_back(x/smaller, y/smaller);
        circle(far_src, Point2i(x, y), 2, GREEN, -1);
        putText(far_src, String(to_string(pick_far_count)),
                Point(x+4, y+4), 1, 3, GREEN, 1, LINE_AA);
        pick_far_count++;
    }
}

void close_mouse_callback(int event, int x, int y, int flags, void* param) {
    if (event == EVENT_LBUTTONDOWN) {
        close_receiving = false;
        cout << "close count --" << pick_close_count << "-- : ( " << x/smaller << " , " << y/smaller << " ) " << endl;
        close_points_list.emplace_back(x/smaller, y/smaller);
        circle(close_src, Point2d(x, y), 2, GREEN, -1);
        putText(close_src, String(to_string(pick_close_count)),
                Point(x+4, y+4), 1, 3, GREEN, 1, LINE_AA);
        pick_close_count++;
    }
}


void PointsPickUp::send_far_points() {
    far_point_publisher_->publish(far_points_msg);
    cout << "publish far points---size: " << far_points_msg.data.size() << endl;
}
void PointsPickUp::send_close_points() {
    close_point_publisher_->publish(close_points_msg);
    cout << "publish close points---size: " << close_points_msg.data.size() << endl;
}

void PointsPickUp::send_small_map_information() {
    small_map_info_msg.id = 66;
    inform_small_map_publisher_->publish(small_map_info_msg);
    cout << "informed small_map to update params" << endl;
}