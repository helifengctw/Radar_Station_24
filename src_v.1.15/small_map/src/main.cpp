#include <cstdio>
#include <opencv2/opencv.hpp>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "radar_interfaces/msg/points.hpp"
#include "radar_interfaces/msg/dist_points.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;
using std::placeholders::_1;


// various region position in small map coordinate, calculated accord the rule
vector<vector<Point>> our_warn_regions, enemy_warn_regions;
vector<Point> our_R1 = {Point(0, 395), Point(0, 562),Point(33, 562), Point(33, 395)};
vector<Point> our_R2 = {Point(76, 511), Point(160, 569), Point(247, 569),Point(259, 562),Point(235, 532),Point(172, 530), Point(100, 477)};
vector<Point> our_R3 = {Point(0, 572), Point(0, 705),Point(157, 705), Point(157, 654), Point(31, 572)};
vector<Point> our_dafu = {Point(370, 558), Point(370, 609),Point(416, 609), Point(416, 558)};
vector<Point> our_highway = {Point(415, 464), Point(415, 644),Point(450, 644), Point(450, 464)};
vector<Point> our_outpost = {Point(414, 558), Point(414, 445),Point(317, 445), Point(317, 558)};
vector<Point> our_half_field = {Point(0, 420), Point(0, 840),Point(450, 840), Point(450, 420)};
vector<Point> enemy_highway = {Point(35, 376), Point(35, 196),Point(0, 196), Point(0, 376)};
vector<Point> enemy_dafu = {Point(80, 282), Point(80, 231),Point(34, 231), Point(34, 282)};
vector<Point> enemy_outpost = {Point(36, 282), Point(36, 395),Point(133, 395), Point(133, 282)};
vector<Point> enemy_hero_hide = {Point(417, 333), Point(417, 445),Point(450, 445), Point(450, 333)};
vector<Point> enemy_R3 = {Point(450, 268), Point(450, 135),Point(293, 135), Point(293, 186), Point(419, 268)};
vector<Point> guard_forbidden_zone{Point(160, 647), Point(160, 705),Point(287, 705), Point(287, 647)};
vector<Point> guidao_houbian{Point(0, 676), Point(0, 840),Point(450, 840), Point(450, 676)};

int X_shift = 0;
int Y_shift = 0;
vector<cv::Point3d> far_objectPoints(4);
vector<cv::Point2d> far_imagePoints(4);
cv::Mat far_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
cv::Mat far_distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
Mat far_Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat far_R = Mat::eye(3, 3, CV_64FC1);
Mat far_T = Mat::zeros(3, 1, CV_64FC1);
vector<cv::Point3d> close_objectPoints(4);
vector<cv::Point2d> close_imagePoints(4);
cv::Mat close_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
cv::Mat close_distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
Mat close_Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat close_R = Mat::eye(3, 3, CV_64FC1);
Mat close_T = Mat::zeros(3, 1, CV_64FC1);

Mat small_map, small_map_copy;
int red_or_blue = 0; // 0 stands for red, 1 stands for blue
int outpost_calc_flag = 0;

radar_interfaces::msg::Points far_points, close_points, result_points, relative_coordinates, guard_relative;
radar_interfaces::msg::DistPoint outpost_dist;
Point3f our_guard, enemy_guard, our_hero, outpost_2d_armour, outpost_3d_armour;

uint8_t in_our_base_cnt;
uint16_t warn_region_state = 0x0000;

int field_width = 28, field_height = 15;
double imgCols = 1280.0, imgRows = 1024.0;

class ParamReceiver : public rclcpp::Node {
public:
    ParamReceiver(): Node("param_receiver") {
        RCLCPP_INFO(this->get_logger(), "ParamReceiver node init!!!");
        // TODO declare_parameter()
    }
    void LoadCameraParams();
    void LoadPnpParams();

private:
};


class MsgSubscriber : public rclcpp::Node {
public:
    MsgSubscriber() : Node("msg_subscriber") {
        far_image_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
                "/sensor_far/calibration", 1, std::bind(&MsgSubscriber::far_calibration, this, _1));
        close_image_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
                "/sensor_close/calibration", 1, std::bind(&MsgSubscriber::close_calibration, this, _1));
        far_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoints>(
                "/sensor_far/distance_point", 1, std::bind(&MsgSubscriber::far_distPointCallback, this, _1));
        close_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoints>(
                "/sensor_close/distance_point", 1, std::bind(&MsgSubscriber::close_distPointCallback, this, _1));
        outpost_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoint>(
                "/sensor_far/outpost", 1, std::bind(&MsgSubscriber::outpost_Callback, this, _1));
    }

private:
    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr far_image_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr close_image_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::DistPoints>::SharedPtr far_distant_point_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::DistPoints>::SharedPtr close_distant_point_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::DistPoint>::SharedPtr outpost_distant_point_subscription_;

    void far_calibration(const radar_interfaces::msg::Points::SharedPtr) const;
    void close_calibration(const radar_interfaces::msg::Points::SharedPtr) const;
    void far_distPointCallback(const radar_interfaces::msg::DistPoints::SharedPtr) const;
    void close_distPointCallback(const radar_interfaces::msg::DistPoints::SharedPtr) const;
    void outpost_Callback(const radar_interfaces::msg::DistPoint::SharedPtr) const;
};


class MsgPublisher : public rclcpp::Node {
public:
    MsgPublisher() : Node("msg_publisher") {
        hero_publisher_ = this->create_publisher<radar_interfaces::msg::Point>("/hero_pub", 1);
        world_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/world_point", 10);
        guard_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/guard_pub", 10);

        timer_ = this->create_wall_timer(
                500ms, std::bind(&MsgPublisher::timer_callback, this));
    }

private:
    rclcpp::Publisher<radar_interfaces::msg::Point>::SharedPtr hero_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr world_point_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::Points>::SharedPtr guard_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
};


radar_interfaces::msg::Point calculate_relative_codi(const Point3f &guard, const radar_interfaces::msg::Point &enemy, uint8_t priority_id) {
    radar_interfaces::msg::Point re_codi;
    re_codi.x = enemy.x * 15000 - guard.x;
    re_codi.y = enemy.y * 28000 - guard.y;
    re_codi.z = enemy.z * 1000 - guard.z;
    re_codi.id = priority_id;
    return re_codi;
}

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

bool is_enemy_car(uint8_t id)
{
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


//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
void onMouse(int event, int x, int y, int flags, void *userdata);

void draw_point_on_map(const radar_interfaces::msg::Point &point, Mat &image);

void remove_duplicate();

void draw_warn_region(Mat &image, const vector<vector<Point>> &our_regions, const vector<vector<Point>> &enemy_regions);

void warn_on_map(const radar_interfaces::msg::Points &points, Mat &image);


int main(int argc, char **argv){
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
    ParamReceiver param_receiver;


    // Load required params
    param_receiver.LoadCameraParams();
    param_receiver.LoadPnpParams();

    // start calculating
    cout << "已读取到closeCam默认参数值!下面进行SolvePnP求解外参矩阵。" << endl;
    cout << "close obj points:" << close_objectPoints << endl;
    cv::solvePnPRansac(close_objectPoints, close_imagePoints, close_CamMatrix_, close_distCoeffs_,
                       close_Rjacob, close_T,cv::SOLVEPNP_AP3P);
    Rodrigues(close_Rjacob, close_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "旋转矩阵:" << close_R << endl;
    cout << "平移矩阵" << close_T << endl;

    cout << "已读取到farCam默认参数值!下面进行SolvePnP求解外参矩阵。" << endl;
    cout << "far obj points:" << far_objectPoints << endl;
    cv::solvePnPRansac(far_objectPoints, far_imagePoints, far_CamMatrix_, far_distCoeffs_,
                       far_Rjacob, far_T, cv::SOLVEPNP_AP3P);
    Rodrigues(far_Rjacob, far_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "旋转矩阵:" << far_R << endl;
    cout << "平移矩阵" << far_T << endl;


    if (red_or_blue == 0)
    {
        // TODO std::string small_map_png = std::string(PACK_PATH) + "/src/red_minimap.png";
        std::string small_map_png = "/src/red_minimap.png";
        small_map = imread(small_map_png);
    }
    else
    {
        // TODO std::string small_map_png = std::string(PACK_PATH) + "/src/blue_minimap.png";
        std::string small_map_png = "/src/blue_minimap.png";
        small_map = imread(small_map_png);
    }
    resize(small_map, small_map, Size(450, 840));
    small_map.copyTo(small_map_copy);
    namedWindow("small_map");
    setMouseCallback("small_map", onMouse, &small_map_copy);

    rclcpp::spin(make_shared<MsgSubscriber>());
    rclcpp::shutdown();

}


void ParamReceiver::LoadCameraParams() {
    std::string my_param = this->get_parameter("my_parameter").as_string();
    /*小地图解算偏移量*/
    X_shift = this->get_parameter("small_map_param.small_map_shift_X").as_int();
    Y_shift = this->get_parameter("small_map_param.small_map_shift_Y").as_int();
    /*相机标定 Camera Calibration*/
    // loop 4 times to get 4 corner points 
    for (int i = 0; i < 4; i++) {
        string parent_name_far = "sensor_far.point";
        string close_parent_name = "sensor_close.point";
        string param_name_far_x = parent_name_far + char(i+1) + ".x";
        string param_name_far_y = parent_name_far + char(i+1) + ".y";
        string param_name_far_z = parent_name_far + char(i+1) + ".z";
        string param_name_close_x = close_parent_name + char(i+1) + ".x";
        string param_name_close_y = close_parent_name + char(i+1) + ".y";
        string param_name_close_z = close_parent_name + char(i+1) + ".z";

        far_objectPoints[0].x = this->get_parameter(param_name_far_x).as_int();
        far_objectPoints[0].y = this->get_parameter(param_name_far_y).as_int();
        far_objectPoints[0].z = this->get_parameter(param_name_far_z).as_int();

        close_objectPoints[0].x = this->get_parameter(param_name_close_x).as_int();
        close_objectPoints[0].y = this->get_parameter(param_name_close_y).as_int();
        close_objectPoints[0].z = this->get_parameter(param_name_close_z).as_int();
    }
    
    string btlcolor = this->get_parameter(".battle_state.battle_color").as_string();
    if (btlcolor == "red") red_or_blue = 0;
    if (btlcolor == "blue") red_or_blue = 1;

    far_CamMatrix_.at<double>(0, 0) = this->get_parameter("sensor_far.camera_matrix.zerozero").as_double();
    far_CamMatrix_.at<double>(0, 2) = this->get_parameter("sensor_far.camera_matrix.zerotwo").as_double();
    far_CamMatrix_.at<double>(1, 1) = this->get_parameter("sensor_far.camera_matrix.oneone").as_double();
    far_CamMatrix_.at<double>(1, 2) = this->get_parameter("sensor_far.camera_matrix.onetwo").as_double();
    far_CamMatrix_.at<double>(2, 2) = this->get_parameter("sensor_far.camera_matrix.twotwo").as_double();
    far_CamMatrix_.at<double>(0, 1) = 0;
    far_CamMatrix_.at<double>(1, 0) = 0;
    far_CamMatrix_.at<double>(2, 0) = 0;
    far_CamMatrix_.at<double>(2, 1) = 0;
    cout << far_CamMatrix_ << endl;
    far_distCoeffs_.at<double>(0, 0) = this->get_parameter("sensor_far.distortion_coefficient.zero").as_double();
    far_distCoeffs_.at<double>(1, 0) = this->get_parameter("sensor_far.distortion_coefficient.one").as_double();
    far_distCoeffs_.at<double>(2, 0) = this->get_parameter("sensor_far.distortion_coefficient.two").as_double();
    far_distCoeffs_.at<double>(3, 0) = this->get_parameter("sensor_far.distortion_coefficient.three").as_double();
    far_distCoeffs_.at<double>(4, 0) = this->get_parameter("sensor_far.distortion_coefficient.four").as_double();
    cout << far_distCoeffs_ << endl;

    close_CamMatrix_.at<double>(0, 0) = this->get_parameter("sensor_close.camera_matrix.zerozero").as_double();
    close_CamMatrix_.at<double>(0, 2) = this->get_parameter("sensor_close.camera_matrix.zerotwo").as_double();
    close_CamMatrix_.at<double>(1, 1) = this->get_parameter("sensor_close.camera_matrix.oneone").as_double();
    close_CamMatrix_.at<double>(1, 2) = this->get_parameter("sensor_close.camera_matrix.onetwo").as_double();
    close_CamMatrix_.at<double>(2, 2) = this->get_parameter("sensor_close.camera_matrix.twotwo").as_double();
    close_CamMatrix_.at<double>(0, 1) = 0;
    close_CamMatrix_.at<double>(1, 0) = 0;
    close_CamMatrix_.at<double>(2, 0) = 0;
    close_CamMatrix_.at<double>(2, 1) = 0;
    cout << close_CamMatrix_ << endl;
    close_distCoeffs_.at<double>(0, 0) = this->get_parameter("sensor_close.distortion_coefficient.zero").as_double();
    close_distCoeffs_.at<double>(1, 0) = this->get_parameter("sensor_close.distortion_coefficient.one").as_double();
    close_distCoeffs_.at<double>(2, 0) = this->get_parameter("sensor_close.distortion_coefficient.two").as_double();
    close_distCoeffs_.at<double>(3, 0) = this->get_parameter("sensor_close.distortion_coefficient.three").as_double();
    close_distCoeffs_.at<double>(4, 0) = this->get_parameter("sensor_close.distortion_coefficient.four").as_double();
    cout << close_distCoeffs_ << endl;
}

void ParamReceiver::LoadPnpParams() {
    //dovejh 读取默认pnp四点的坐标，防止意外重启造成pnp数据丢失。
    double x = 0, y = 0;
    // loop 4 times to get 4 corner points 
    for (int i = 0; i < 4; i++) {
        string parent_name_far = "camera.list.farCam.calibrationDefault.point";
        string param_name_x_far = parent_name_far + char(i+1) + ".x";
        string param_name_y_far = parent_name_far + char(i+1) + ".y";

        string parent_name_close = "camera.list.closeCam.calibrationDefault.point";
        string param_name_x_close = parent_name_close + char(i+1) + ".x";
        string param_name_y_close = parent_name_close + char(i+1) + ".y";

        x = this->get_parameter(param_name_x_far).as_double();
        y = this->get_parameter(param_name_y_far).as_double();
        far_imagePoints[i] = cv::Point2d(x, y);
        far_imagePoints[i].x *= imgCols;
        far_imagePoints[i].y *= imgRows;
        cout << far_imagePoints[i] << endl;
        
        x = this->get_parameter(param_name_x_close).as_double();
        y = this->get_parameter(param_name_y_close).as_double();
        close_imagePoints[i] = cv::Point2d(x, y);
        close_imagePoints[i].x *= imgCols;
        close_imagePoints[i].y *= imgRows;
        cout << close_imagePoints[i] << endl;
    }
}

void MsgSubscriber::far_calibration(const radar_interfaces::msg::Points::SharedPtr msg) const {
    for (const auto &point: msg->data) {
        far_imagePoints[point.id] = cv::Point2d(point.x, point.y);
        far_imagePoints[point.id].x *= imgCols;
        far_imagePoints[point.id].y *= imgRows;
        cout << far_imagePoints[point.id] << endl;
    }
    cout << "已经选出了4个点!下面进行SolvePnP求解外参矩阵。" << endl;
    outpost_2d_armour.x = far_imagePoints[0].x;
    outpost_2d_armour.y = far_imagePoints[0].y;
    cv::Mat inlier;
    int suc = cv::solvePnPRansac(far_objectPoints, far_imagePoints, far_CamMatrix_, far_distCoeffs_, far_Rjacob,
                                 far_T,
                                 false, 100, 8.0, 0.99,
                                 inlier, cv::SOLVEPNP_AP3P);
    Rodrigues(far_Rjacob, far_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << far_R << endl;
    cout << "平移矩阵" << far_T << endl;
}

void MsgSubscriber::close_calibration(const radar_interfaces::msg::Points::SharedPtr msg) const {
    for (const auto &point: msg->data) {
        close_imagePoints[point.id] = cv::Point2d(point.x, point.y);
        close_imagePoints[point.id].x *= imgCols;
        close_imagePoints[point.id].y *= imgRows;
        cout << close_imagePoints[point.id] << endl;
    }
    cout << "已经选出了4个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat inlier;
    cout << "close obj points:" << close_objectPoints << endl;
    int suc = cv::solvePnPRansac(close_objectPoints, close_imagePoints, close_CamMatrix_, close_distCoeffs_,
                                 close_Rjacob, close_T, false, 100, 8.0, 0.99,
                                 inlier, cv::SOLVEPNP_AP3P);
    Rodrigues(close_Rjacob, close_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << close_R << endl;
    cout << "平移矩阵" << close_T << endl;
}

void MsgSubscriber::far_distPointCallback(const radar_interfaces::msg::DistPoints::SharedPtr input) const {
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

void MsgSubscriber::close_distPointCallback(const radar_interfaces::msg::DistPoints::SharedPtr input) const {
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

void MsgSubscriber::outpost_Callback(const radar_interfaces::msg::DistPoint::SharedPtr msg) const {
    outpost_dist.dist = msg->dist;
    outpost_dist.x = msg->x;
    outpost_dist.y = msg->y;
    outpost_calc_flag=1;
}

void MsgPublisher::timer_callback() {
    small_map.copyTo(small_map_copy);
    draw_warn_region(small_map_copy, our_warn_regions, enemy_warn_regions);
    warn_on_map(result_points, small_map_copy);
    remove_duplicate();
    warn_on_map(result_points, small_map_copy);
    for (auto &i: result_points.data) {
        draw_point_on_map(i, small_map_copy);
    }
    world_point_publisher_->publish(result_points);
    if (!guard_relative.data.empty()) {
        Point2f ab;
        ab.x = (guard_relative.data[0].x + our_guard.x) / 15000 * 450 - X_shift;
        ab.y = 840 - (guard_relative.data[0].y + our_guard.y) / 28000 * 840 - Y_shift;

        //cout << (int) in_our_base_cnt << endl;
        if (pointPolygonTest(guidao_houbian, ab, false) >= 0) {
            in_our_base_cnt++;
            if (in_our_base_cnt > 10) {
                in_our_base_cnt = 10;
                guard_publisher_->publish(guard_relative);
            }
            circle(small_map_copy, ab, 10, Scalar(255, 255, 255), -1, LINE_8, 0);
        } else if (pointPolygonTest(guard_forbidden_zone, ab, false) <= 0) {
            in_our_base_cnt = 0;
            guard_relative.data[0].x+=300;
            guard_relative.data[0].y+=250;
            guard_relative.data[0].z-=300;
            guard_publisher_->publish(guard_relative);
            circle(small_map_copy, ab, 10, Scalar(255, 255, 255), -1, LINE_8, 0);
        } else {
            circle(small_map_copy, ab, 10, Scalar(0, 255, 255), -1, LINE_8, 0);
        }
    } else {
        radar_interfaces::msg::Point abc;
        abc.x = 30000;
        abc.y = 30000;
        guard_relative.data.emplace_back(abc);
        guard_publisher_->publish(guard_relative);
    }
    if(outpost_3d_armour.x>0 && our_hero.x>0){
        float h_o_dist = sqrt(pow(outpost_3d_armour.x - our_hero.x, 2) + pow(outpost_3d_armour.y - our_hero.y, 2)+ pow(outpost_3d_armour.z-our_hero.z,2));
        h_o_dist-=609;
        radar_interfaces::msg::Point hero;
        hero.x=h_o_dist;
        hero_publisher_->publish(hero);
    }
    else {
        radar_interfaces::msg::Point hero;
        hero.x=30000;
        hero_publisher_->publish(hero);
    }
    imshow("small_map", small_map_copy);
    waitKey(1);
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
    if (point.id <= 5 || point.id == 12)scalar = Scalar(0, 0, 255);
    else scalar = Scalar(255, 0, 0);
    circle(image, calculate_pixel_codi(point), 10,
           scalar, -1, LINE_8, 0);
    if (point.id != 12 && point.id != 13) {
        if (point.id <= 5)id = to_string(point.id + 1);
        if (point.id == 5)id = "G";
        if (point.id >= 6)id = to_string(point.id - 5);
        if (point.id == 11)id = "G";
        putText(image, id,
                calculate_pixel_text_codi(point), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0xFF, 0xFF, 0xFF), 2);
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
        int test2=pointPolygonTest(right_region, calculate_pixel_codi(i), false);
        if (test > 0) {
            result_points.data.emplace_back(i);
        } else if (test == 0 && i.x != 200) {
            result_points.data.emplace_back(i);
        } else if (pointPolygonTest(right_region, calculate_pixel_codi(i), false)<=0) {
            left_may_overlap_points.data.emplace_back(i);
        }
    }
    for (auto &i: close_points.data) {
        int test = pointPolygonTest(right_region, calculate_pixel_codi(i), false);
        int test2=pointPolygonTest(left_region, calculate_pixel_codi(i), false);
        if (test > 0) {
            result_points.data.emplace_back(i);
        } else if (test == 0 && i.x != 255) {
            result_points.data.emplace_back(i);
        } else if (pointPolygonTest(left_region, calculate_pixel_codi(i), false)<=0) {
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
                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
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
    vector<radar_interfaces::msg::Point>().swap(guard_relative.data);
    vector<radar_interfaces::msg::Point>().swap(relative_coordinates.data);
    warn_region_state = 0x0000;
    Scalar light_green = Scalar(0xcc, 0xff, 0xcc);
    far_points.id = 0;
    for (int i = 0; i < our_warn_regions.size(); i++) {
        for (radar_interfaces::msg::Point car: points.data) {
            if (is_enemy_car(car.id) && pointPolygonTest(our_half_field,
                                                         calculate_pixel_codi(car),
                                                         false) > 0) {
                relative_coordinates.data.push_back(calculate_relative_codi(our_guard, car, 0));
                if (pointPolygonTest(our_warn_regions[i], calculate_pixel_codi(car), false) > 0) {
                    warn_region_state |= (0x01 << (i + 5));
                    drawContours(image, our_warn_regions, i, light_green, -1);
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
                drawContours(image, enemy_warn_regions, i, light_green, -1);
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