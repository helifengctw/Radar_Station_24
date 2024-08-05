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
                "/sensor_far/raw/image", 1, std::bind(&PointsPickUp::FarImgCallback, this, _1));
        close_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/sensor_close/raw/image", 1, std::bind(&PointsPickUp::CloseImgCallback, this, _1));

        far_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>(
                "/sensor_far/calibration", 1);
        close_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>(
                "/sensor_close/calibration", 1);

        timer_ = this->create_wall_timer(
                5000ms, std::bind(&PointsPickUp::timer_callback, this));
    }
    void send_far_points();
    void send_close_points();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr far_img_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr close_img_subscription_;

    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr far_point_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr close_point_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    void FarImgCallback(sensor_msgs::msg::Image::SharedPtr) const;
    bool CloseImgCallback(sensor_msgs::msg::Image::SharedPtr) const;
    void timer_callback();
};

void far_mouse_callback(int event, int x, int y, int flags, void* param);
void close_mouse_callback(int event, int x, int y, int flags, void* param);

Mat far_src, close_src;
int pick_far_count = 0, pick_close_count = 0, valid_far_img_count = 0, valid_close_img_count = 0,
        far_send_count = 0, close_send_count = 0;
Scalar GREEN(0, 255, 0);
bool far_receiving = true, close_receiving = true;
radar_interfaces::msg::Points far_points_msg, close_points_msg;
vector<Point2i> far_points_list, close_points_list;

int main(int argc, char ** argv)
{
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto PPU_node = std::make_shared<PointsPickUp>("point_pick_up");

    namedWindow("far_pnp_points_picker");
    namedWindow("close_pnp_points_picker");
    setMouseCallback("far_pnp_points_picker", far_mouse_callback, &far_src);
    setMouseCallback("close_pnp_points_picker", close_mouse_callback);
    rclcpp::Rate loop_rate(10);

    while(true) {
        if (far_receiving) {
            rclcpp::spin_some(PPU_node);
        }
        if (!far_src.empty()) {
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

    while(true) {
        if (close_receiving) {
            rclcpp::spin_some(PPU_node);
        }
        if (!far_src.empty() && !close_src.empty()) {
            imshow("close_pnp_points_picker", close_src);
            int k = waitKey(1);
            if (k == 27) {
                break;  // press esc to shut down the window
            }
        }
    }
    cout << "please enter the number of wanted close four points :" << endl;
    for (int i = 0; i < 4; i++) {
        int j = 0;
        cout << "the " << i << "th one:" << endl;
        cin >> j;
        cout << "pick : " << far_points_list[j] << endl;
        radar_interfaces::msg::Point point;
        point.x = (float) close_points_list[j].x;
        point.y = (float) close_points_list[j].y;
        close_points_msg.data.push_back(point);
    }
    PPU_node->send_close_points();
    destroyWindow("close_pnp_points_picker");
    destroyAllWindows();

    rclcpp::spin(PPU_node);
}

void PointsPickUp::FarImgCallback(const sensor_msgs::msg::Image::SharedPtr img) const {
    Mat src = cv_bridge::toCvShare(img, "bgr8")->image;
    src.copyTo(far_src);
}

bool PointsPickUp::CloseImgCallback(sensor_msgs::msg::Image::SharedPtr img) const {
    Mat src = cv_bridge::toCvShare(img, "bgr8")->image;
    src.copyTo(close_src);
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