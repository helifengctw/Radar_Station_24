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
        {"White", Scalar(0xfa, 0xfa, 0xff)}, {"Red", Scalar(0x00, 0x00, 0xff)},
        {"Green_light", Scalar(0xcc, 0xff, 0xcc)}, {"Orange", Scalar(0x00, 0x8c, 0xff)},
        {"Blue", Scalar(0xff, 0x90, 0x1e)}, {"Yellow", Scalar(0x11, 0xff, 0xff)}
};

int X_shift = 0;
int Y_shift = 0;
int red_or_blue = 0; // 0 stands for red, 1 stands for blue

Mat small_map, small_map_copy;
int outpost_calc_flag = 0;

radar_interfaces::msg::Points far_points, close_points, result_points, relative_coordinates, guard_relative;
radar_interfaces::msg::DistPoint outpost_dist;
Point3f our_guard, enemy_guard, our_hero, outpost_3d_armour;

uint8_t in_our_base_cnt;
uint16_t warn_region_state = 0x0000;

// 24*14 (*600mm) 
double field_width = 24*0.6, field_height = 14*0.6;
int img_show_width = 14*30, img_show_height = 24*30;
double imgCols = 1280.0, imgRows = 1024.0;

class SmallMap : public rclcpp::Node {
public:
    SmallMap(string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "small map node initialize!!!");

        far_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoints>(
                "/sensor_far/distance_point", 1, std::bind(&SmallMap::far_distPointCallback, this, _1));
        close_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoints>(
                "/sensor_close/distance_point", 1, std::bind(&SmallMap::close_distPointCallback, this, _1));
        outpost_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoint>(
                "/sensor_far/outpost", 1, std::bind(&SmallMap::outpost_Callback, this, _1));
        Pnp_result_subscription_ = this->create_subscription<radar_interfaces::msg::PnpResult>(
                "pnp_result", 10, std::bind(&SmallMap::Pnp_resultCallback, this, _1));

        hero_publisher_ = this->create_publisher<radar_interfaces::msg::Point>("/hero_pub", 1);
        world_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/world_point", 10);
        guard_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/guard_pub", 10);

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
    rclcpp::Subscription<radar_interfaces::msg::DistPoint>::SharedPtr outpost_distant_point_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::PnpResult>::SharedPtr Pnp_result_subscription_;

    void far_distPointCallback(radar_interfaces::msg::DistPoints::SharedPtr) const;
    void close_distPointCallback(radar_interfaces::msg::DistPoints::SharedPtr) const;
    void outpost_Callback(radar_interfaces::msg::DistPoint::SharedPtr) const;
    void Pnp_resultCallback(radar_interfaces::msg::PnpResult::SharedPtr);

    rclcpp::Publisher<radar_interfaces::msg::Point>::SharedPtr hero_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr world_point_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr guard_publisher_;

    void load_param();
};


//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
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

    cv::Mat small_map(Size(img_show_width, img_show_height), CV_8UC3, color_table["White"]);
    add_grid(small_map);
    small_map.copyTo(small_map_copy);
    namedWindow("small_map");

    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok()) {
        rclcpp::spin_some(SM_node);
        small_map.copyTo(small_map_copy);
        remove_duplicate();
        for (auto &i: result_points.data) {
//            std::cout << i.x << " : " << i.y << ' id: ' << i.id << endl;
            draw_point_on_map(i, small_map_copy);
        }
        SM_node->SendWorldPoint(result_points);
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
            x8_pixel *= (1000 * input->data[i].dist);
            Mat calcWorld = invR * (invM * x8_pixel - far_T);//2D-3D变换
            calcWorld /= 1000;
            double x = calcWorld.at<double>(0, 0);
            double y = calcWorld.at<double>(1, 0);
            x /= field_height;
            y /= field_width;
            radar_interfaces::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = calcWorld.at<double>(2, 0);
            if (red_or_blue == 0) {
                if (input->data[i].id == 5) {
                    our_guard.x = x * field_height;
                }
                if (input->data[i].id == 11) {
                    enemy_guard.x = x * field_height;
                }
                if (input->data[i].id == 0) {
                    our_hero.x = point.x * field_height;
                    our_hero.y = point.y * field_width;
                    our_hero.z = point.z * 1000;
                }
            } else {
                if (input->data[i].id == 11) {
                    our_guard.x = x * field_height;
                }
                if (input->data[i].id == 5) {
                    enemy_guard.x = x * field_height;
                }
                if (input->data[i].id == 6) {
                    our_hero.x = point.x * field_height;
                    our_hero.y = point.y * field_width;
                    our_hero.z = point.z * 1000;
                }
            }
            point.id = input->data[i].id;
//            std::cout << x << " : " << y << " id: " << point.id << endl;
            far_points.data.push_back(point);
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
            x8_pixel *= (1000 * input->data[i].dist);
            Mat calcWorld = invR * (invM * x8_pixel - close_T);//2D-3D变换
            calcWorld /= 1000;
            double x = calcWorld.at<double>(0, 0);
            double y = calcWorld.at<double>(1, 0);
            x /= field_height;
            y /= field_width;
            radar_interfaces::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = calcWorld.at<double>(2, 0);
            if (red_or_blue == 0) {
                if (input->data[i].id == 5) {
                    our_guard.x = x * field_height;
                }
                if (input->data[i].id == 11) {
                    enemy_guard.x = x * field_height;
                }
                if (input->data[i].id == 0) {
                    our_hero.x = point.x * field_height;
                    our_hero.y = point.y * field_width;
                    our_hero.z = point.z * 1000;
                }
            } else {
                if (input->data[i].id == 11) {
                    our_guard.x = x * field_height;
                }
                if (input->data[i].id == 5) {
                    enemy_guard.x = x * field_height;
                }
                if (input->data[i].id == 6) {
                    our_hero.x = point.x * field_height;
                    our_hero.y = point.y * field_width;
                    our_hero.z = point.z * 1000;
                }
            }
            point.id = input->data[i].id;
            close_points.data.push_back(point);
        }
    }
}

void SmallMap::outpost_Callback(const radar_interfaces::msg::DistPoint::SharedPtr msg) const {
    outpost_dist.dist = msg->dist;
    outpost_dist.x = msg->x;
    outpost_dist.y = msg->y;
    outpost_calc_flag=1;
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
    cout << endl << "far R matrix load done!" << endl << far_R << endl;
    cout << endl << "far T matrix load done!" << endl << far_T << endl;
    cout << endl << "close R matrix load done!" << endl << close_R << endl;
    cout << endl << "close T matrix load done!" << endl << close_T << endl;
    cout << endl << "far_Camera matrix load done!" << endl << far_CamMatrix_ << endl;
    cout << endl << "close_Camera matrix load done!" << endl << close_CamMatrix_ << endl;
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
    res.x = point.x * 450 - (float) X_shift;
    res.y = (1 - point.y) * 840 - (float) Y_shift;
    return res;
}

Point2f calculate_pixel_text_codi(const radar_interfaces::msg::Point &point) {
    Point2f res;
    res.x = point.x * 450 - (float) X_shift - 7;
    res.y = (1 - point.y) * 840 - (float) Y_shift + 7;
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
    cout << X_shift << "\t" << Y_shift << "\t" << btlcolor << endl;
    if (btlcolor == "red") red_or_blue = 0;
    if (btlcolor == "blue") red_or_blue = 1;
}