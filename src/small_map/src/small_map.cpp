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


cv::Mat far_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
Mat far_R = Mat::eye(3, 3, CV_64FC1);
Mat far_T = Mat::ones(3, 1, CV_64FC1);
cv::Mat close_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
Mat close_R = Mat::eye(3, 3, CV_64FC1);
Mat close_T = Mat::zeros(3, 1, CV_64FC1);

// BGR
map<string, Scalar> color_table = {
        {"White", Scalar(0xfa, 0xfa, 0xff)},
        {"Red", Scalar(0x00, 0x00, 0xff)},
        {"Green_light", Scalar(0xcc, 0xff, 0xcc)},
        {"Orange", Scalar(0x00, 0x8c, 0xff)},
        {"Blue", Scalar(0xff, 0x90, 0x1e)},
        {"Yellow", Scalar(0x11, 0xff, 0xff)}
};

int X_shift = 0;
int Y_shift = 0;
int red_or_blue = 0; // 0 stands for red, 1 stands for blue

Mat small_map, small_map_copy;
int outpost_calc_flag = 0;

radar_interfaces::msg::Points far_points, close_points, result_points, filtered_result_points;
int tracker_id_lock_list[6] = {0};

uint8_t in_our_base_cnt;
uint16_t warn_region_state = 0x0000;

bool mouse_click_flag = false;
cv::Point2f mouse_point = cv::Point2f(0.0, 0.0);


// 24*14 (*600mm) 
double field_width = 15, field_height = 28;
int img_show_width = 15*30, img_show_height = 28*30;
double imgCols = 1440.0, imgRows = 1080.0;

class SmallMap : public rclcpp::Node {
public:
    SmallMap(string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "small map node initialize!!!");

        far_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoints>(
                "/sensor_far/distance_point", 1, std::bind(&SmallMap::far_distPointCallback, this, _1));
        close_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoints>(
                "/sensor_close/distance_point", 1, std::bind(&SmallMap::close_distPointCallback, this, _1));
        Pnp_result_subscription_ = this->create_subscription<radar_interfaces::msg::PnpResult>(
                "pnp_result", 1, std::bind(&SmallMap::Pnp_resultCallback, this, _1));
        Icp_result_subscription_ = this->create_subscription<radar_interfaces::msg::PnpResult>(
                "icp_result", 1, std::bind(&SmallMap::Icp_resultCallback, this, _1));

        hero_publisher_ = this->create_publisher<radar_interfaces::msg::Point>("/hero_pub", 1);
        world_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/world_point", 1);
        guard_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/guard_pub", 1);

        this->load_param();
    }

    void SendWorldPoint(radar_interfaces::msg::Points &msg) {
        this->world_point_publisher_->publish(msg);
    }
    void SendHero(radar_interfaces::msg::Point &msg) {
        this->hero_publisher_->publish(msg);
    }
    void SendGuard(radar_interfaces::msg::Points &msg) {
        this->guard_publisher_->publish(msg);
    }

private:
    rclcpp::Subscription<radar_interfaces::msg::DistPoints>::SharedPtr far_distant_point_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::DistPoints>::SharedPtr close_distant_point_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::PnpResult>::SharedPtr Pnp_result_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::PnpResult>::SharedPtr Icp_result_subscription_;

    void far_distPointCallback(radar_interfaces::msg::DistPoints::SharedPtr) const;
    void close_distPointCallback(radar_interfaces::msg::DistPoints::SharedPtr) const;
    void Pnp_resultCallback(radar_interfaces::msg::PnpResult::SharedPtr);
    void Icp_resultCallback(radar_interfaces::msg::PnpResult::SharedPtr);

    rclcpp::Publisher<radar_interfaces::msg::Point>::SharedPtr hero_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr world_point_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr guard_publisher_;

    void load_param();
};


void add_grid(cv::Mat &src);
void draw_point_on_map(const radar_interfaces::msg::Point &point, Mat &image);
void remove_duplicate();
bool is_enemy_car(uint8_t);
double calculate_dist(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b);
double Point2PointDist(const radar_interfaces::msg::Point &a, const Point3f &b);
radar_interfaces::msg::Point calculate_relative_codi(const Point3f &guard, const radar_interfaces::msg::Point &enemy, uint8_t priority_id);
Point2f calculate_pixel_codi(const radar_interfaces::msg::Point &point);
Point2f calculate_pixel_text_codi(const radar_interfaces::msg::Point &point);

// 1, 7:工程  *  0, 6:英雄  *
int main(int argc, char **argv){
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto SM_node = std::make_shared<SmallMap>("small_map");

    if (red_or_blue == 0) {
        std::string small_map_png = std::string("/home/hlf/Desktop/radar24_ws/src/small_map/images_24") + "/red_smallmap.png";
        small_map = imread(small_map_png);
    } else {
        std::string small_map_png = std::string("/home/hlf/Desktop/radar24_ws/src/small_map/images_24") + "/blue_smallmap.png";
        small_map = imread(small_map_png);
    }
    resize(small_map, small_map, Size(img_show_width, img_show_height));
    small_map.copyTo(small_map_copy);
    namedWindow("small_map");

    rclcpp::Rate loop_rate(40);
    while (rclcpp::ok()) {
        rclcpp::spin_some(SM_node);
        small_map.copyTo(small_map_copy);
        remove_duplicate();
        vector<radar_interfaces::msg::Point>().swap(filtered_result_points.data);
        //根据裁判系统的标记进度，确定哪些tracker_id需要锁定
        for (auto &i: result_points.data) {
            if (is_enemy_car(i.id)) {
                if (red_or_blue == 0 && i.id <= 5) {
                    filtered_result_points.data.emplace_back(i);
                    // TODO : cancel debug out
                    std::cout << "id : " << (int) i.id << ", [ " << i.x << " , " << i.y << " , " << i.z << " ]" << std::endl;
                    // TODO : end at here
                } else if (red_or_blue == 1 && i.id >= 6 && i.id <= 11) { // id >= 6 || id == 13
                    filtered_result_points.data.emplace_back(i);
                    // TODO : cancel debug out
                    std::cout << "id : " << (int) i.id << ", [ " << i.x << " , " << i.y << " , " << i.z << " ]" << std::endl;
                    // TODO : end at here
                }
            }
            draw_point_on_map(i, small_map_copy);
        }
        SM_node->SendWorldPoint(filtered_result_points); // filtered_result_points

        if (mouse_click_flag) {
            mouse_click_flag = false;
            std::string toshow = "at : " + to_string(mouse_point.x) + " , " + to_string(mouse_point.y);
            cv::putText(small_map_copy, toshow, cv::Point(10, 400), cv::FONT_HERSHEY_SIMPLEX, 0.7, color_table["Red"], 2);
        }

        imshow("small_map", small_map_copy);
        waitKey(1);
        loop_rate.sleep();
    }
//    rclcpp::spin(SM_node);
//    rclcpp::shutdown();
    return 0;
}


void SmallMap::far_distPointCallback(const radar_interfaces::msg::DistPoints::SharedPtr input) const {
    Mat invR;
    Mat invM;
    invert(far_CamMatrix_, invM);
    invert(far_R, invR);
    std::vector<radar_interfaces::msg::Point>().swap(far_points.data);
    for (int i = 0; i < input->data.size(); i++) {
        if (input->data[i].dist > 0) {
            Mat x8_pixel;
            x8_pixel = (Mat_<double>(3, 1) << (double) input->data[i].x, (double) input->data[i].y, 1);
            x8_pixel *= (1000 * input->data[i].dist);  // input->data[i].dist -- *10m
            Mat calcWorld = invR * (invM * x8_pixel - far_T); //2D-3D变换
            calcWorld /= 1000;
            double x = calcWorld.at<double>(0, 0);
            double y = calcWorld.at<double>(1, 0);
            x /= field_width;
            y /= field_height;
            radar_interfaces::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = calcWorld.at<double>(2, 0);
            point.id = input->data[i].id;  // point.z -> *10m
            point.tracker_id = input->data[i].tracker_id;
            if (point.id == 66) {
                mouse_click_flag = true;
                mouse_point = Point2f(x*field_width, y*field_height);
            }
//            std::cout << x << " : " << y << " id: " << point.id << endl;
            far_points.data.push_back(point);
//            if (input->data[i].id == 666) {
//
//            }
        }
    }
}

void SmallMap::close_distPointCallback(const radar_interfaces::msg::DistPoints::SharedPtr input) const {
    Mat invR;
    Mat invM;
    invert(close_CamMatrix_, invM);
    invert(close_R, invR);
    std::vector<radar_interfaces::msg::Point>().swap(close_points.data);
    for (int i = 0; i < input->data.size(); i++) {
        if (input->data[i].dist > 0) {
            Mat x8_pixel;
            x8_pixel = (Mat_<double>(3, 1) << (double) input->data[i].x, (double) input->data[i].y, 1);
            x8_pixel *= (1000 * input->data[i].dist);  // input->data[i].dist -- *10m
            Mat calcWorld = invR * (invM * x8_pixel - close_T); //2D-3D变换
            calcWorld /= 1000;
            double x = calcWorld.at<double>(0, 0);
            double y = calcWorld.at<double>(1, 0);
            x /= field_width;
            y /= field_height;
            radar_interfaces::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = calcWorld.at<double>(2, 0);
            point.id = input->data[i].id;  // point.z -> *10m
            point.tracker_id = input->data[i].tracker_id;
//            std::cout << x << " : " << y << " id: " << point.id << endl;
            close_points.data.push_back(point);
//            if (input->data[i].id == 666) {
//
//            }
        }
    }
}


void SmallMap::Pnp_resultCallback(const radar_interfaces::msg::PnpResult::SharedPtr msg) {
    for (int i = 0; i < 3; i++) {
        far_T.at<double>(i) = msg->far_t[i];
        close_T.at<double>(i) = msg->close_t[i];
        for (int j = 0; j < 3; j++) {
            far_CamMatrix_.at<double>(i, j) = msg->far_cam_matrix[3*i + j];
            far_R.at<double>(i, j) = msg->far_r[3*i + j];
            close_CamMatrix_.at<double>(i, j) = msg->close_cam_matrix[3*i + j];
            close_R.at<double>(i, j) = msg->close_r[3*i + j];
        }
    }
    cout << "pnp result received" << endl;
    cout << endl << "far R matrix load done!" << endl << far_R << endl;
    cout << endl << "far T matrix load done!" << endl << far_T << endl;
    cout << endl << "close R matrix load done!" << endl << close_R << endl;
    cout << endl << "close T matrix load done!" << endl << close_T << endl;
    cout << endl << "far_Camera matrix load done!" << endl << far_CamMatrix_ << endl;
    cout << endl << "close_Camera matrix load done!" << endl << close_CamMatrix_ << endl;
}

void SmallMap::Icp_resultCallback(radar_interfaces::msg::PnpResult::SharedPtr msg) {
    for (int i = 0; i < 3; i++) {
        far_T.at<double>(i) = msg->far_t[i];
        close_T.at<double>(i) = msg->close_t[i];
        for (int j = 0; j < 3; j++) {
            far_R.at<double>(i, j) = msg->far_r[3*i + j];
            close_R.at<double>(i, j) = msg->close_r[3*i + j];
        }
    }
    cout << "icp result received" << endl;
    cout << endl << "far R matrix load done!" << endl << far_R << endl;
    cout << endl << "far T matrix load done!" << endl << far_T << endl;
    cout << endl << "close R matrix load done!" << endl << close_R << endl;
    cout << endl << "close T matrix load done!" << endl << close_T << endl;
}

void add_grid(cv::Mat &src) {
    for (int i = 1; i < 14; i++) {
        cv::line(src, cv::Point(30*i, 0), cv::Point(30*i, img_show_height), color_table["Orange"], 1);
    }
    for (int i = 1; i < 24; i++) {
        cv::line(src, cv::Point(0, 30*i), cv::Point(img_show_width, 30*i), color_table["Orange"], 1);
    }
}

void draw_point_on_map(const radar_interfaces::msg::Point &point, Mat &image) {
    Scalar scalar;
    string id;
    if (point.id <= 5 || point.id == 12) scalar = color_table["Red"];//Scalar(0, 0, 255);
    else scalar = color_table["Blue"];
    circle(image, calculate_pixel_codi(point), 10,
           scalar, -1, LINE_8, 0);
    if (point.id != 12 && point.id != 13) {
        if (point.id <= 5) id = to_string(point.id + 1);
        if (point.id == 5) id = "G";
        if (point.id >= 6) id = to_string(point.id - 5);
        if (point.id == 11) id = "G";
        putText(image, id, calculate_pixel_text_codi(point), cv::FONT_HERSHEY_SIMPLEX,
                0.7, color_table["Withe"], 2);
    }
}

void remove_duplicate() {
    vector<radar_interfaces::msg::Point>().swap(result_points.data);
    radar_interfaces::msg::Points red_no_id_cars;
    radar_interfaces::msg::Points blue_no_id_cars;
    radar_interfaces::msg::Points left_may_overlap_points;
    radar_interfaces::msg::Points right_may_overlap_points;
    vector<Point2f> left_region = {Point(0, 0), Point(0, img_show_height),
                                   Point(168, img_show_height), Point(278, 0)};
    vector<Point2f> right_region = {Point(278, 0), Point(168, img_show_height),
                                    Point(img_show_width, img_show_height), Point(img_show_width, 0)};
    for (auto &i: far_points.data) {
        int test = pointPolygonTest(left_region, calculate_pixel_codi(i), false);
        int test2 = pointPolygonTest(right_region, calculate_pixel_codi(i), false);
        if (test > 0) {
            result_points.data.emplace_back(i);
        } else if (test == 0 && i.x != 200) {
            result_points.data.emplace_back(i);
        } else if (test2 <= 0) {
            left_may_overlap_points.data.emplace_back(i);
        }
    }
    for (auto &i: close_points.data) {
        int test = pointPolygonTest(right_region, calculate_pixel_codi(i), false);
        int test2 = pointPolygonTest(left_region, calculate_pixel_codi(i), false);
        if (test > 0) {
            result_points.data.emplace_back(i);
        } else if (test == 0 && i.x != 255) {
            result_points.data.emplace_back(i);
        } else if (test2 <= 0) {
            right_may_overlap_points.data.emplace_back(i);
        }
    }

    //    for (auto it_left_a = left_may_overlap_points.data.begin(); it_left < left_may_overlap_points.data.end();) {
//        for (auto it_left_b = right_may_overlap_points.data.begin(); it_right < right_may_overlap_points.data.end();) {
//            if (it_left->id == it_right->id && it_left->id == 12 && calculate_dist(*it_left, *it_right) < 5) {
//                radar_interfaces::msg::Point center;
//                center.id = 12;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty()) left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty()) right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            } else if (it_left->id == it_right->id && it_left->id == 13 && calculate_dist(*it_left, *it_right) < 5) {
//                radar_interfaces::msg::Point center;
//                center.id = 13;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            } else if (it_left->id == it_right->id && it_left->id < 12 && it_left->id > 0) {
//                radar_interfaces::msg::Point center;
//                center.id = it_left->id;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            }
//            if (inner_erase_flag == 1) {
//                inner_erase_flag = 0;
//                continue;
//            }
//            it_right++;
//        }
//        if (outer_erase_flag == 1) {
//            outer_erase_flag = 0;
//            continue;
//        }
//        it_left++;
//    }

    uint8_t inner_erase_flag = 0;
    uint8_t outer_erase_flag = 0;
    for (auto it_left = left_may_overlap_points.data.begin(); it_left < left_may_overlap_points.data.end();) {
        for (auto it_right = right_may_overlap_points.data.begin(); it_right < right_may_overlap_points.data.end();) {
            if (it_left->id == it_right->id && it_left->id == 12 && calculate_dist(*it_left, *it_right) < 5) {
                radar_interfaces::msg::Point center;
                center.id = 12;
                center.x = (it_left->x + it_right->x) / 2;
                center.y = (it_left->y + it_right->y) / 2;
                result_points.data.emplace_back(center);
                if (!left_may_overlap_points.data.empty()) left_may_overlap_points.data.erase(it_left);
                if (!right_may_overlap_points.data.empty()) right_may_overlap_points.data.erase(it_right);
                inner_erase_flag = 1;
                outer_erase_flag = 1;
            } else if (it_left->id == it_right->id && it_left->id == 13 && calculate_dist(*it_left, *it_right) < 5) {
                radar_interfaces::msg::Point center;
                center.id = 13;
                center.x = (it_left->x + it_right->x) / 2;
                center.y = (it_left->y + it_right->y) / 2;
                result_points.data.emplace_back(center);
                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
                inner_erase_flag = 1;
                outer_erase_flag = 1;
            } else if (it_left->id == it_right->id && it_left->id < 12 && it_left->id > 0) {
                radar_interfaces::msg::Point center;
                center.id = it_left->id;
                center.x = (it_left->x + it_right->x) / 2;
                center.y = (it_left->y + it_right->y) / 2;
                result_points.data.emplace_back(center);
                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
                inner_erase_flag = 1;
                outer_erase_flag = 1;
            }
            if (inner_erase_flag == 1) {
                inner_erase_flag = 0;
                continue;
            }
            it_right++;
        }
        if (outer_erase_flag == 1) {
            outer_erase_flag = 0;
            continue;
        }
        it_left++;
    }
    for (auto &i: left_may_overlap_points.data) {
        result_points.data.emplace_back(i);
    }
    for (auto &i: right_may_overlap_points.data) {
        result_points.data.emplace_back(i);
    }
}

radar_interfaces::msg::Point calculate_relative_codi(const Point3f &guard, const radar_interfaces::msg::Point &enemy, uint8_t priority_id) {
    radar_interfaces::msg::Point re_codi;
    re_codi.x = enemy.x * field_height - guard.x;
    re_codi.y = enemy.y * field_width - guard.y;
    re_codi.z = enemy.z * 1000 - guard.z;
    re_codi.id = priority_id;
    return re_codi;
}

/*
 * 把result_points 里面归一化的点转换成小地图(450*840)图像中的像素座标
 */
Point2f calculate_pixel_codi(const radar_interfaces::msg::Point &point) {
    Point2f res;
    res.x = point.x * img_show_width - (float) X_shift;
    res.y = (1 - point.y) * img_show_height- (float) Y_shift;
    return res;
}

Point2f calculate_pixel_text_codi(const radar_interfaces::msg::Point &point) {
    Point2f res;
    res.x = point.x * img_show_width - (float) X_shift - 7;
    res.y = (1 - point.y) * img_show_height - (float) Y_shift + 7;
    return res;
}

double Point2PointDist(const radar_interfaces::msg::Point &a, const Point3f &b) {
    double res = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return res;
}

double calculate_dist(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b) {
    double res = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return res;
}

bool is_enemy_car(uint8_t id) {
    if (red_or_blue == 0) {
        if ((id >= 6 && id <= 11) || id == 13)
            return true;
        else return false;
    } else {
        if (id <= 5 || id == 12)
            return true;
        else return false;
    }
}

void SmallMap::load_param() {
    this->declare_parameter<int>("small_map_params.small_map_shift_X", 0);
    this->declare_parameter<int>("small_map_params.small_map_shift_Y", 0);
    this->declare_parameter<std::string>("battle_state.battle_color", "empty");

    X_shift = this->get_parameter("small_map_params.small_map_shift_X").as_int();  // =30
    Y_shift = this->get_parameter("small_map_params.small_map_shift_Y").as_int();  // =5
    string btlcolor = this->get_parameter("battle_state.battle_color").as_string();
    cout << endl << "Load X_shift, Y_shift, red_or_blue : " << endl;
    cout << "\t" << X_shift << "\t" << Y_shift << "\t" << btlcolor << endl;
    if (btlcolor == "red") red_or_blue = 0;
    else if (btlcolor == "blue") red_or_blue = 1;
}