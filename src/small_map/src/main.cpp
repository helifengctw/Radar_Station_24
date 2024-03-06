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


// various region position in small map coordinate, calculated accord the rule
vector<vector<Point>> our_warn_regions, enemy_warn_regions;
vector<Point> our_R1 = {Point(0, 395), Point(0, 562), Point(33, 562), Point(33, 395)};
vector<Point> our_R2 = {Point(76, 511), Point(160, 569), Point(247, 569), Point(259, 562), Point(235, 532),
                        Point(172, 530), Point(100, 477)};
vector<Point> our_R3 = {Point(0, 572), Point(0, 705), Point(127, 705), Point(127, 639), Point(31, 572)};
vector<Point> our_dafu = {Point(370, 558), Point(370, 609), Point(416, 609), Point(416, 558)};
vector<Point> our_highway = {Point(415, 464), Point(415, 644), Point(450, 644), Point(450, 464)};
vector<Point> our_outpost = {Point(414, 558), Point(414, 445), Point(317, 445), Point(317, 558)};
vector<Point> our_half_field = {Point(0, 420), Point(0, 840), Point(450, 840), Point(450, 420)};
vector<Point> enemy_highway = {Point(35, 376), Point(35, 196), Point(0, 196), Point(0, 376)};
vector<Point> enemy_dafu = {Point(80, 282), Point(80, 231), Point(34, 231), Point(34, 282)};
vector<Point> enemy_outpost = {Point(36, 282), Point(36, 395), Point(133, 395), Point(133, 282)};
vector<Point> enemy_hero_hide = {Point(417, 333), Point(417, 445), Point(450, 445), Point(450, 333)};
vector<Point> enemy_R3 = {Point(450, 268), Point(450, 135), Point(323, 135), Point(323, 201), Point(419, 268)};
vector<Point> guard_forbidden_zone{Point(160, 647), Point(160, 705), Point(287, 705), Point(287, 647)};
vector<Point> guidao_houbian{Point(0, 676), Point(0, 840), Point(450, 840), Point(450, 676)};

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
        {"Blue", Scalar(0xff, 0x90, 0x1e)}, {"Yellow", Scalar(0x00, 0xff, 0xff)}
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

int field_width = 28, field_height = 15;
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
                "pnp_result", 1, std::bind(&SmallMap::Pnp_resultCallback, this, _1));

        hero_publisher_ = this->create_publisher<radar_interfaces::msg::Point>("/hero_pub", 1);
        world_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/world_point", 10);
        guard_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/guard_pub", 10);

        this->declare_parameter<int>("small_map_param.small_map_shift_X", 0);
        this->declare_parameter<int>("small_map_param.small_map_shift_Y", 0);
        this->declare_parameter<std::string>(".battle_state.battle_color", "empty");

        X_shift = this->get_parameter("small_map_param.small_map_shift_X").as_int();  // =30
        Y_shift = this->get_parameter("small_map_param.small_map_shift_Y").as_int();  // =5
        string btlcolor = this->get_parameter(".battle_state.battle_color").as_string();
        cout << endl << X_shift << endl;
        if (btlcolor == "red") red_or_blue = 0;
        if (btlcolor == "blue") red_or_blue = 1;

//        this->load_param();
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
void onMouse(int event, int x, int y, int flags, void *userdata);
void draw_point_on_map(const radar_interfaces::msg::Point &point, Mat &image);
void remove_duplicate();
void draw_warn_region(Mat &image, const vector<vector<Point>> &our_regions, const vector<vector<Point>> &enemy_regions);
void warn_on_map(const radar_interfaces::msg::Points &points, Mat &image);
bool is_enemy_car(uint8_t);
double calculate_dist(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b);
double Point2PointDist(const radar_interfaces::msg::Point &a, const Point3f &b);
radar_interfaces::msg::Point calculate_relative_codi(const Point3f &guard, const radar_interfaces::msg::Point &enemy, uint8_t priority_id);
Point2f calculate_pixel_codi(const radar_interfaces::msg::Point &point);
Point2f calculate_pixel_text_codi(const radar_interfaces::msg::Point &point);

// 1, 7:工程  *  0, 6:英雄  *
int main(int argc, char **argv){
    our_guard.x = 9200;
    our_guard.y = 5333.44;
    enemy_guard.y = 22666.56;
    our_guard.z = 1300.0;
    enemy_guard.z = 1300.0;
    our_hero = Point3f(0, 0, 0);
    outpost_3d_armour.x=0;
    outpost_3d_armour.y=0;
    outpost_3d_armour.z=0;

    our_warn_regions.emplace_back(our_R1);
    our_warn_regions.emplace_back(our_R2);
    our_warn_regions.emplace_back(our_R3);
    our_warn_regions.emplace_back(our_dafu);
    our_warn_regions.emplace_back(our_highway);
    our_warn_regions.emplace_back(our_outpost);
    enemy_warn_regions.emplace_back(enemy_highway);
    enemy_warn_regions.emplace_back(enemy_dafu);
    enemy_warn_regions.emplace_back(enemy_outpost);
    enemy_warn_regions.emplace_back(enemy_hero_hide);
    enemy_warn_regions.emplace_back(enemy_R3);

    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto SM_node = std::make_shared<SmallMap>("small_map");

    if (red_or_blue == 0)
    {
        std::string small_map_png = std::string("/home/hlf/Desktop/radar24_ws/src/small_map/image_23") + "/red_minimap.png";
        small_map = imread(small_map_png);
    } else {
        std::string small_map_png = std::string("/home/hlf/Desktop/radar24_ws/src/small_map/image_23") + "/blue_minimap.png";
        small_map = imread(small_map_png);
    }
    resize(small_map, small_map, Size(450, 840));
    small_map.copyTo(small_map_copy);
    namedWindow("small_map");
    setMouseCallback("small_map", onMouse, &small_map_copy);

    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok()) {
        rclcpp::spin_some(SM_node);
        small_map.copyTo(small_map_copy);
        draw_warn_region(small_map_copy, our_warn_regions, enemy_warn_regions);
        warn_on_map(result_points, small_map_copy);
        remove_duplicate();
        warn_on_map(result_points, small_map_copy);
        for (auto &i: result_points.data) {
//            std::cout << i.x << " : " << i.y << ' id: ' << i.id << endl;
            draw_point_on_map(i, small_map_copy);
        }
        SM_node->SendWorldPoint(result_points);
        if (!guard_relative.data.empty()) {
            Point2f ab;
            ab.x = (guard_relative.data[0].x + our_guard.x) / 15000 * 450 - X_shift;
            ab.y = 840 - (guard_relative.data[0].y + our_guard.y) / 28000 * 840 - Y_shift;

            //cout << (int) in_our_base_cnt << endl;
            if (pointPolygonTest(guidao_houbian, ab, false) >= 0) {
                in_our_base_cnt++;
                if (in_our_base_cnt > 10) {
                    in_our_base_cnt = 10;
                    SM_node->SendGuard(guard_relative);
                }
                circle(small_map_copy, ab, 10, Scalar(255, 255, 255), -1, LINE_8, 0);
            } else if (pointPolygonTest(guard_forbidden_zone, ab, false) <= 0) {
                in_our_base_cnt = 0;
                guard_relative.data[0].x+=300;
                guard_relative.data[0].y+=250;
                guard_relative.data[0].z-=300;
                SM_node->SendGuard(guard_relative);
                circle(small_map_copy, ab, 10, Scalar(255, 255, 255), -1, LINE_8, 0);
            } else {
                circle(small_map_copy, ab, 10, Scalar(0, 255, 255), -1, LINE_8, 0);
            }
        } else {
            radar_interfaces::msg::Point abc;
            abc.x = 30000;
            abc.y = 30000;
            guard_relative.data.emplace_back(abc);
            SM_node->SendGuard(guard_relative);
        }
        if(outpost_3d_armour.x>0 && our_hero.x>0){
            float h_o_dist = sqrt(pow(outpost_3d_armour.x - our_hero.x, 2) + pow(outpost_3d_armour.y - our_hero.y, 2)+ pow(outpost_3d_armour.z-our_hero.z,2));
            h_o_dist-=609;
            radar_interfaces::msg::Point hero;
            hero.x=h_o_dist;
            SM_node->SendHero(hero);
        } else {
            radar_interfaces::msg::Point hero;
            hero.x=30000;
            SM_node->SendHero(hero);
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
                    point.y = 0.19048;
                    our_guard.x = x * 15000;
                }
                if (input->data[i].id == 11) {
                    point.y = 0.80952;
                    enemy_guard.x = x * 15000;
                }
                if (input->data[i].id == 0) {
                    our_hero.x = point.x * 15000;
                    our_hero.y = point.y * 28000;
                    our_hero.z = point.z * 1000;
                }
            } else {
                if (input->data[i].id == 11) {
                    point.y = 0.19048;
                    our_guard.x = x * 15000;
                }
                if (input->data[i].id == 5) {
                    point.y = 0.80952;
                    enemy_guard.x = x * 15000;
                }
                if (input->data[i].id == 6) {
                    our_hero.x = point.x * 15000;
                    our_hero.y = point.y * 28000;
                    our_hero.z = point.z * 1000;
                }
            }
            point.id = input->data[i].id;
            std::cout << x << " : " << y << " id: " << point.id << endl;
            far_points.data.push_back(point);
        }
    }
    if (outpost_calc_flag==1 && outpost_dist.dist>0 && outpost_dist.dist<24) {
        Mat x8_pixel;
        x8_pixel = (Mat_<double>(3, 1) << (double) outpost_dist.x, (double) outpost_dist.y, 1);
        x8_pixel *= (1000 * outpost_dist.dist);
        Mat calcWorld = invR * (invM * x8_pixel - far_T);//2D-3D变换
        outpost_3d_armour.x = calcWorld.at<double>(0, 0);
        outpost_3d_armour.y = calcWorld.at<double>(1, 0);
        outpost_3d_armour.z = calcWorld.at<double>(2, 0);
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
                    point.y = 0.19048;
                    our_guard.x = x * 15000;
                }
                if (input->data[i].id == 11) {
                    point.y = 0.80952;
                    enemy_guard.x = x * 15000;
                }
                if (input->data[i].id == 0) {
                    our_hero.x = point.x * 15000;
                    our_hero.y = point.y * 28000;
                    our_hero.z = point.z * 1000;
                }
            } else {
                if (input->data[i].id == 11) {
                    point.y = 0.19048;
                    our_guard.x = x * 15000;
                }
                if (input->data[i].id == 5) {
                    point.y = 0.80952;
                    enemy_guard.x = x * 15000;
                }
                if (input->data[i].id == 6) {
                    our_hero.x = point.x * 15000;
                    our_hero.y = point.y * 28000;
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

//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号 
void onMouse(int event, int x, int y, int flags, void *userdata){
    if (event == cv::EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆
    {
        circle(*(Mat *) userdata, Point2i(x, y), 5, Scalar(0x27, 0xc1, 0x36), -1);
        int is_in_poly = pointPolygonTest(our_R3, Point2f(x, y), false);
        cout << is_in_poly << "  ";
        if (is_in_poly > 0)cout << "in" << endl;
        else if (is_in_poly < 0)cout << "out" << endl;
        else if (is_in_poly == 0)cout << "edge" << endl;
        imshow("small_map", *(Mat *) userdata);
        cout << Point(x, y) << endl;
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
    vector<Point2f> left_region = {Point(0, 0), Point(0, 840), Point(168, 840), Point(278, 0)};
    vector<Point2f> right_region = {Point(278, 0), Point(168, 840), Point(450, 840), Point(450, 0)};
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

void draw_warn_region(Mat &image, const vector<vector<Point>> &our_regions, 
                      const vector<vector<Point>> &enemy_regions) {
    Scalar our_scalar;
    Scalar enemy_scalar;
    if (red_or_blue == 0) {
        our_scalar = Scalar(0, 0, 255);
        enemy_scalar = Scalar(255, 0, 0);
    } else {
        enemy_scalar = Scalar(0, 0, 255);
        our_scalar = Scalar(255, 0, 0);
    }
    for (const vector<Point> &region: our_regions) {
        polylines(image, our_regions, 1, our_scalar, 2);
    }
    for (const vector<Point> &region: enemy_regions) {
        polylines(image, enemy_regions, 1, enemy_scalar, 2);
    }
}

void warn_on_map(const radar_interfaces::msg::Points &points, Mat &image) {
    vector<radar_interfaces::msg::Point>().swap(guard_relative.data);  //离哨兵最近的车的坐标
    vector<radar_interfaces::msg::Point>().swap(relative_coordinates.data);  //识别出来的车辆相对于哨兵的坐标
    warn_region_state = 0x0000;
    far_points.id = 0;
    for (int i = 0; i < our_warn_regions.size(); i++) {
        for (radar_interfaces::msg::Point car: points.data) {
            if (is_enemy_car(car.id) && pointPolygonTest(our_half_field,
                                                         calculate_pixel_codi(car),
                                                         false) > 0) {
                relative_coordinates.data.push_back(calculate_relative_codi(our_guard, car, 0));
                if (pointPolygonTest(our_warn_regions[i], calculate_pixel_codi(car), false) > 0) {
                    warn_region_state |= (0x01 << (i + 5));  //???为什么是5
                    drawContours(image, our_warn_regions, i, color_table["Green_light"], -1);
                }
            }
        }
    }
    double nearest = 50000.0;
    double dist;
    uint8_t position = 0;
    for (int i = 0; i < relative_coordinates.data.size(); i++) {
        dist = Point2PointDist(relative_coordinates.data[i], Point3f(0, 0, 0));
        if (dist < nearest) {
            nearest = dist;
            position = (uint8_t) i;
        }
    }
    if (!relative_coordinates.data.empty()) {
        guard_relative.data.emplace_back(relative_coordinates.data[position]);
    }
    for (int i = 0; i < enemy_warn_regions.size(); i++) {
        for (radar_interfaces::msg::Point car: points.data) {
            if (is_enemy_car(car.id) && pointPolygonTest(enemy_warn_regions[i],
                                                         calculate_pixel_codi(car),
                                                         false) > 0) {
                warn_region_state |= (0x01 << (i));
                drawContours(image, enemy_warn_regions, i, color_table["Green_light"], -1);
                if (red_or_blue == 0 && guard_relative.data.empty()) {
                    if (i == 4 && car.id == 6) {
                        guard_relative.data.emplace_back(
                                calculate_relative_codi(our_guard, car, 1));//敌方英雄到达敌方公路区，可能会打前哨站
                    } else if (i == 2 && car.id != 7 && guard_relative.data.empty()) {
                        guard_relative.data.emplace_back(
                                calculate_relative_codi(our_guard, car, 2));//敌方车辆到达敌方前哨站(工程除外)
                    }
                } else {
                    if (i == 4 && car.id == 0 && guard_relative.data.empty()) {
                        guard_relative.data.emplace_back(
                                calculate_relative_codi(our_guard, car, 1));//敌方英雄到达敌方公路区，可能会打前哨站
                    } else if (i == 2 && car.id != 1 && guard_relative.data.empty()) {
                        guard_relative.data.emplace_back(
                                calculate_relative_codi(our_guard, car, 2));//敌方车辆到达敌方前哨站(工程除外)
                    }
                }
            }
        }
    }
    unsigned char state[2];
    memcpy(state, &warn_region_state, 2);
    result_points.text = string(state, state + 2);
//    cout<<bitset<8>(guard_relative.text[1])<<bitset<8>(guard_relative.text[0])<<endl;
}

radar_interfaces::msg::Point calculate_relative_codi(const Point3f &guard, const radar_interfaces::msg::Point &enemy, uint8_t priority_id) {
    radar_interfaces::msg::Point re_codi;
    re_codi.x = enemy.x * 15000 - guard.x;
    re_codi.y = enemy.y * 28000 - guard.y;
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
    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this, "pnp_solver");
    std::vector<rclcpp::Parameter> param_vector;

    while (!parameter_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
//            return -1;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for the parameter service to start...");
    }

    param_vector = parameter_client->get_parameters(
            {"sensor_far.uni_matrix.zerozero", "sensor_far.uni_matrix.zeroone", "sensor_far.uni_matrix.zerotwo",
             "sensor_far.uni_matrix.zerothree", "sensor_far.uni_matrix.onezero", "sensor_far.uni_matrix.oneone",
             "sensor_far.uni_matrix.onetwo", "sensor_far.uni_matrix.onethree", "sensor_far.uni_matrix.twozero",
             "sensor_far.uni_matrix.twoone", "sensor_far.uni_matrix.twotwo","sensor_far.uni_matrix.twothree"}
    );
//    far_R.at<double>(0, 0) = param_vector[0].as_double();
//    far_R.at<double>(0, 1) = param_vector[1].as_double();
//    far_R.at<double>(0, 2) = param_vector[2].as_double();
//    far_T.at<double>(0, 0) = param_vector[3].as_double();
//    far_R.at<double>(1, 0) = param_vector[4].as_double();
//    far_R.at<double>(1, 1) = param_vector[5].as_double();
//    far_R.at<double>(1, 2) = param_vector[6].as_double();
//    far_T.at<double>(1, 0) = param_vector[7].as_double();
//    far_R.at<double>(2, 0) = param_vector[8].as_double();
//    far_R.at<double>(2, 1) = param_vector[9].as_double();
//    far_R.at<double>(2, 2) = param_vector[10].as_double();
//    far_T.at<double>(2, 0) = param_vector[11].as_double();
    far_R.at<double>(0, 0) = 0.99826;
    far_R.at<double>(0, 1) = -0.0558975;
    far_R.at<double>(0, 2) = -0.01865807492;
    far_T.at<double>(0, 0) = -6523.262105853722;
    far_R.at<double>(1, 0) = 0.002979243252566429;
    far_R.at<double>(1, 1) = -0.3640851585657675;
    far_R.at<double>(1, 2) = -0.9313608975159853;
    far_T.at<double>(1, 0) = 3140.755533853394;
    far_R.at<double>(2, 0) = -0.0588539175837365;
    far_R.at<double>(2, 1) = 0.9296867545397987;
    far_R.at<double>(2, 2) = -0.3636189692773227;
    far_T.at<double>(2, 0) = 3150.696588184383;
    cout << "far R matrix load done!" << endl << far_R << endl;
    cout << "far T matrix load done!" << endl << far_T << endl;


    param_vector = parameter_client->get_parameters(
            {"sensor_close.uni_matrix.zerozero", "sensor_close.uni_matrix.zeroone", "sensor_close.uni_matrix.zerotwo",
             "sensor_close.uni_matrix.zerothree", "sensor_close.uni_matrix.onezero", "sensor_close.uni_matrix.oneone",
             "sensor_close.uni_matrix.onetwo", "sensor_close.uni_matrix.onethree", "sensor_close.uni_matrix.twozero",
             "sensor_close.uni_matrix.twoone", "sensor_close.uni_matrix.twotwo","sensor_close.uni_matrix.twothree"}
    );
//    close_R.at<double>(0, 0) = param_vector[0].as_double();
//    close_R.at<double>(0, 1) = param_vector[1].as_double();
//    close_R.at<double>(0, 2) = param_vector[2].as_double();
//    close_T.at<double>(0, 0) = param_vector[3].as_double();
//    close_R.at<double>(1, 0) = param_vector[4].as_double();
//    close_R.at<double>(1, 1) = param_vector[5].as_double();
//    close_R.at<double>(1, 2) = param_vector[6].as_double();
//    close_T.at<double>(1, 0) = param_vector[7].as_double();
//    close_R.at<double>(2, 0) = param_vector[8].as_double();
//    close_R.at<double>(2, 1) = param_vector[9].as_double();
//    close_R.at<double>(2, 2) = param_vector[10].as_double();
//    close_T.at<double>(2, 0) = param_vector[11].as_double();
    close_R.at<double>(0, 0) = 0.999337396352221;
    close_R.at<double>(0, 1) = -0.03525103316746379;
    close_R.at<double>(0, 2) = 0.009062721036772645;
    close_T.at<double>(0, 0) = -7023.804783881676;
    close_R.at<double>(1, 0) = -0.003284081884404988;
    close_R.at<double>(1, 1) =  -0.3353071587501599;
    close_R.at<double>(1, 2) = -0.9421031387789086;
    close_T.at<double>(1, 0) = 971.9981536467531;
    close_R.at<double>(2, 0) = 0.03624890423365259;
    close_R.at<double>(2, 1) =  0.9414491350845894;
    close_R.at<double>(2, 2) = -0.3352007502830774;
    close_T.at<double>(2, 0) = 12897.46947588635;
    cout << "close R matrix load done!" << endl << close_R << endl;
    cout << "close T matrix load done!" << endl << close_T << endl;

    param_vector = parameter_client->get_parameters(
            {"sensor_far.camera_matrix.zerozero", "sensor_far.camera_matrix.zerotwo",
             "sensor_far.camera_matrix.oneone", "sensor_far.camera_matrix.onetwo", "sensor_far.camera_matrix.twotwo"});
//    far_CamMatrix_.at<double>(0, 0) = param_vector[0].as_double();
//    far_CamMatrix_.at<double>(0, 2) = param_vector[1].as_double();
//    far_CamMatrix_.at<double>(1, 1) = param_vector[2].as_double();
//    far_CamMatrix_.at<double>(1, 2) = param_vector[3].as_double();
//    far_CamMatrix_.at<double>(2, 2) = param_vector[4].as_double();
    far_CamMatrix_.at<double>(0, 0) = 1890.751574;
    far_CamMatrix_.at<double>(0, 2) = 645.397327;
    far_CamMatrix_.at<double>(1, 1) = 1883.882471;
    far_CamMatrix_.at<double>(1, 2) = 529.4028520000001;
    far_CamMatrix_.at<double>(2, 2) = 1;
    far_CamMatrix_.at<double>(0, 1) = 0;
    far_CamMatrix_.at<double>(1, 0) = 0;
    far_CamMatrix_.at<double>(2, 0) = 0;
    far_CamMatrix_.at<double>(2, 1) = 0;
    cout << "far_Camera matrix load done!" << endl << far_CamMatrix_ << endl;

    param_vector = parameter_client->get_parameters(
            {"sensor_close.camera_matrix.zerozero", "sensor_close.camera_matrix.zerotwo",
             "sensor_close.camera_matrix.oneone", "sensor_close.camera_matrix.onetwo", "sensor_close.camera_matrix.twotwo"});
//    close_CamMatrix_.at<double>(0, 0) = param_vector[0].as_double();
//    close_CamMatrix_.at<double>(0, 2) = param_vector[1].as_double();
//    close_CamMatrix_.at<double>(1, 1) = param_vector[2].as_double();
//    close_CamMatrix_.at<double>(1, 2) = param_vector[3].as_double();
//    close_CamMatrix_.at<double>(2, 2) = param_vector[4].as_double();
    close_CamMatrix_.at<double>(0, 0) = 1854.70825;
    close_CamMatrix_.at<double>(0, 2) = 661.706219;
    close_CamMatrix_.at<double>(1, 1) = 1847.979719;
    close_CamMatrix_.at<double>(1, 2) = 512.214798;
    close_CamMatrix_.at<double>(2, 2) = 1;
    close_CamMatrix_.at<double>(0, 1) = 0;
    close_CamMatrix_.at<double>(1, 0) = 0;
    close_CamMatrix_.at<double>(2, 0) = 0;
    close_CamMatrix_.at<double>(2, 1) = 0;
    cout << "close_Camera matrix load done!" << endl << close_CamMatrix_ << endl;
}