#ifndef PICKUP_IMAGE_POINTS_H
#define PICKUP_IMAGE_POINTS_H

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

namespace utilities {
    class PointsPicker : public rclcpp::Node {
    public:
        PointsPicker(const rclcpp::NodeOptions &options);
        ~PointsPicker();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
        void ImgCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);

        rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr point_publisher_;

        rclcpp::TimerBase::SharedPtr timer_receive, timer_;
        void timer_receive_callback();
        void timer_callback();
        void load_params();

        void send_points();

        Mat src;
        std::string camera_name = "camera_init", win_name;
        int pick_count = 0, valid_img_count = 0, send_count = 0;
        Scalar GREEN = Scalar (0, 255, 0);
        bool receiving = true;
        radar_interfaces::msg::Points points_msg;
        vector<Point2i> points_list;
    };

}


void mouse_callback(int event, int x, int y, int flags, void* param);



int main(int argc, char ** argv)
{
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto PPU_node = std::make_shared<PointsPickUp>("point_pick_up");

    namedWindow("far_pnp_points_picker");
    namedWindow("close_pnp_points_picker");
    setMouseCallback("far_pnp_points_picker", far_mouse_callback, &far_src);
    setMouseCallback("close_pnp_points_picker", close_mouse_callback, &close_src);
    rclcpp::Rate loop_rate(10);

    while(true) {
        if (far_receiving) {
            rclcpp::spin_some(PPU_node);
        }
        if (!far_src.empty() && !close_src.empty()) {
            imshow("far_pnp_points_picker", far_src);
            int k = waitKey(1);
            if (k == 27) {
                break;  // press esc to shut down the window
            }
        }
    }
    cout << "please enter the number of wanted far four points :" << endl;
    for (int i = 0; i < 4; i++) {
        int j = 0;
        cout << "the " << i << "th one:" << endl;
        cin >> j;
        cout << "pick : " << far_points_list[j] << endl;
        radar_interfaces::msg::Point point;
        point.x = (float) far_points_list[j].x;
        point.y = (float) far_points_list[j].y;
        far_points_msg.data.push_back(point);
    }
    PPU_node->send_far_points();
    destroyWindow("far_pnp_points_picker");

//    while(true) {
//        if (close_receiving) {
//            rclcpp::spin_some(PPU_node);
//        }
//        if (!far_src.empty() && !close_src.empty()) {
//            imshow("close_pnp_points_picker", close_src);
//            int k = waitKey(1);
//            if (k == 27) {
//                break;  // press esc to shut down the window
//            }
//        }
//    }
//    cout << "please enter the number of wanted close four points :" << endl;
//    for (int i = 0; i < 4; i++) {
//        int j = 0;
//        cout << "the " << i << "th one:" << endl;
//        cin >> j;
//        cout << "pick : " << far_points_list[j] << endl;
//        radar_interfaces::msg::Point point;
//        point.x = (float) close_points_list[j].x;
//        point.y = (float) close_points_list[j].y;
//        close_points_msg.data.push_back(point);
//    }
//    PPU_node->send_close_points();
//    destroyWindow("close_pnp_points_picker");
    destroyAllWindows();

    rclcpp::spin(PPU_node);
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
    }    src.copyTo(close_src);
}

void PointsPickUp::timer_callback() {
    this->send_far_points();
    this->send_close_points();
}

void far_mouse_callback(int event, int x, int y, int flags, void* param) {
    if (event == EVENT_LBUTTONDOWN) {
        far_receiving = false;
        cout << "far count --" << pick_far_count <<"-- : ( " << x << " , " << y << " ) " << endl;
        far_points_list.emplace_back(x, y);
        circle(far_src, Point2i(x, y), 2, GREEN, -1);
        putText(far_src, String(to_string(pick_far_count)),
                Point(x+4, y+4), 1, 3, GREEN, 1, LINE_AA);
        pick_far_count++;
    }
}

void close_mouse_callback(int event, int x, int y, int flags, void* param) {
    if (event == EVENT_LBUTTONDOWN) {
        close_receiving = false;
        cout << "close count --" << pick_close_count << "-- : ( " << x << " , " << y << " ) " << endl;
        close_points_list.emplace_back(x, y);
        circle(close_src, Point2i(x, y), 2, GREEN, -1);
        putText(close_src, String(to_string(pick_close_count)),
                Point(x+4, y+4), 1, 3, GREEN, 1, LINE_AA);
        pick_close_count++;
    }
}

void PointsPickUp::send_far_points() {
    for (int i = 0; i < 3; i++) {
        this->far_point_publisher_->publish(far_points_msg);
        cout << "publish far points for the <<" << far_send_count++ << "th>>" << endl;
    }
}
void PointsPickUp::send_close_points() {
    for (int i = 0; i < 3; i++) {
        this->close_point_publisher_->publish(close_points_msg);
        cout << "publish close points for the <<" << close_send_count++ << "th>>" << endl;
    }
}