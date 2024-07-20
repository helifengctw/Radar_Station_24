#include <cstdio>
#include <opencv2/opencv.hpp>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "radar_interfaces/msg/points.hpp"
#include "radar_interfaces/msg/dist_points.hpp"
#include "radar_interfaces/msg/pnp_result.hpp"
#include "radar_interfaces/msg/battle_color.hpp"
#include "radar_interfaces/srv/pnp_result.hpp"

using namespace std;
using namespace cv;
using std::placeholders::_1;
using std::placeholders::_2;

int red_or_blue = 0; // 0 stands for red, 1 stands for blue
int imgCols = 1920, imgRows = 1200;
bool far_received_one = false, close_received_one = false;

int X_shift = 0, Y_shift = 0, total_count = 5;
vector<cv::Point3d> far_objectPoints(total_count);
vector<cv::Point2f> far_imagePoints(total_count);
cv::Mat far_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
cv::Mat far_distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
Mat far_Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat far_R = Mat::eye(3, 3, CV_64FC1);
Mat far_T = Mat::zeros(3, 1, CV_64FC1);

vector<cv::Point3d> close_objectPoints(total_count);
vector<cv::Point2f> close_imagePoints(total_count);
cv::Mat close_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
cv::Mat close_distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
Mat close_Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat close_R = Mat::eye(3, 3, CV_64FC1);
Mat close_T = Mat::zeros(3, 1, CV_64FC1);

Point3f outpost_2d_armour;

class PnpSolver : public rclcpp::Node {
public:
    PnpSolver(string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Pnp solver node initialize!!!");

        far_calib_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
                "/sensor_far/calibration", 1, std::bind(&PnpSolver::far_calibration, this, _1));
        close_calib_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
                "/sensor_close/calibration", 1, std::bind(&PnpSolver::close_calibration, this, _1));
        pnp_service_ = this->create_service<radar_interfaces::srv::PnpResult>(
                "pnp_results", std::bind(&PnpSolver::PnpResultCallback, this, _1, _2),
                rmw_qos_profile_services_default, callback_group_organization);
        pnp_result_response = std::make_shared<radar_interfaces::srv::PnpResult::Response>();
        RCLCPP_INFO(this->get_logger(), "pnp result server Ready!!!.");

        this->DeclareParams();
        this->LoadCameraParams();
        this->LoadPnpParams();

        // start calculating
        cout << endl << "已读取到closeCam默认参数值!下面进行SolvePnP求解外参矩阵。" << endl;
        cout << "close obj points:" << std::endl << close_objectPoints << endl;
        cv::solvePnPRansac(close_objectPoints, close_imagePoints, close_CamMatrix_, close_distCoeffs_,
                           close_Rjacob, close_T,cv::SOLVEPNP_AP3P);
        Rodrigues(close_Rjacob, close_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
        cout << "旋转矩阵:" << endl << close_R << endl;
        cout << "平移矩阵:" << endl << close_T << endl;

        cout << endl << "已读取到farCam默认参数值!下面进行SolvePnP求解外参矩阵。" << endl;
        cout << "far obj points:" << std::endl << far_objectPoints << endl;
        cv::solvePnPRansac(far_objectPoints, far_imagePoints, far_CamMatrix_, far_distCoeffs_,
                           far_Rjacob, far_T, cv::SOLVEPNP_AP3P);
        Rodrigues(far_Rjacob, far_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
        cout << "旋转矩阵:" << endl << far_R << endl;
        cout << "平移矩阵:" << endl << far_T << endl;

        write_far_Pnp_srv();
        write_close_Pnp_srv();
    }

private:

    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr far_calib_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr close_calib_subscription_;
    rclcpp::CallbackGroup::SharedPtr callback_group_organization;
    rclcpp::Service<radar_interfaces::srv::PnpResult>::SharedPtr pnp_service_;
    std::shared_ptr<radar_interfaces::srv::PnpResult::Response> pnp_result_response;

    void far_calibration(radar_interfaces::msg::Points::SharedPtr);
    void close_calibration(radar_interfaces::msg::Points::SharedPtr);
    void PnpResultCallback(const radar_interfaces::srv::PnpResult::Request::SharedPtr request,
                           radar_interfaces::srv::PnpResult::Response::SharedPtr response);
    void write_far_Pnp_srv();
    void write_close_Pnp_srv();
    void DeclareParams();
    void LoadCameraParams();
    void LoadPnpParams();
};


int main(int argc, char **argv) {
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto PS_node = std::make_shared<PnpSolver>("pnp_solver");
    // 把节点的执行器变成多线程执行器, 避免死锁
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(PS_node);
    executor.spin();
    rclcpp::shutdown();
}

void PnpSolver::LoadCameraParams() {
    imgCols = this->get_parameter("image_width").as_int();
    imgRows = this->get_parameter("image_height").as_int();

    std::vector<double> temp_array;
    temp_array = this->get_parameter("sensor_far.camera_matrix").as_double_array();
    for (int i = 0; i < far_CamMatrix_.rows; ++i) {
        for (int j = 0; j < far_CamMatrix_.cols; ++j) {
            far_CamMatrix_.at<double>(i, j) = temp_array[i * 3 + j];
        }
    }
    cout << endl << "far_CamMatrix_:" << endl << far_CamMatrix_ << endl;
    temp_array = this->get_parameter("sensor_far.distortion_coefficient").as_double_array();
    for (int i = 0; i < far_distCoeffs_.rows; ++i) {
        far_distCoeffs_.at<double>(i, 0) = temp_array[i];
    }
    cout << endl << "far_distCoeffs_:" << endl << far_distCoeffs_ << endl;

    temp_array = this->get_parameter("sensor_close.camera_matrix").as_double_array();
    for (int i = 0; i < close_CamMatrix_.rows; ++i) {
        for (int j = 0; j < close_CamMatrix_.cols; ++j) {
            close_CamMatrix_.at<double>(i, j) = temp_array[i * 3 + j];
        }
    }
    cout << endl << "close_CamMatrix_:" << endl << close_CamMatrix_ << endl;
    temp_array = this->get_parameter("sensor_close.distortion_coefficient").as_double_array();
    for (int i = 0; i < close_distCoeffs_.rows; ++i) {
        close_distCoeffs_.at<double>(i, 0) = temp_array[i];
    }
    cout << endl << "close_distCoeffs_:" << endl << close_distCoeffs_ << endl;
}

void PnpSolver::LoadPnpParams() {
    //读取默认pnp四点的坐标，保证求解所需参数值一直存在，防止意外重启造成pnp数据丢失。
    double x = 0, y = 0;
    cout << endl << "far_imagePoints and close_imagesPoints:" << endl;
    //世界坐标
    for (int i = 0; i < total_count; i++) {
        string parent_name_far = "calibrate_default_points.farCam.world_points.point";
        string parent_name_close = "calibrate_default_points.closeCam.world_points.point";
        string param_name_far_x = parent_name_far + to_string(i+1) + ".x";
        string param_name_far_y = parent_name_far + to_string(i+1) + ".y";
        string param_name_far_z = parent_name_far + to_string(i+1) + ".z";
        string param_name_close_x = parent_name_close + to_string(i+1) + ".x";
        string param_name_close_y = parent_name_close + to_string(i+1) + ".y";
        string param_name_close_z = parent_name_close + to_string(i+1) + ".z";

        // declare params
        this->declare_parameter<double>(param_name_far_x, 0);
        this->declare_parameter<double>(param_name_far_y, 0);
        this->declare_parameter<double>(param_name_far_z, 0);
        this->declare_parameter<double>(param_name_close_x, 0);
        this->declare_parameter<double>(param_name_close_y, 0);
        this->declare_parameter<double>(param_name_close_z, 0);

        // get params
        far_objectPoints[i].x = this->get_parameter(param_name_far_x).as_double();
        far_objectPoints[i].y = this->get_parameter(param_name_far_y).as_double();
        far_objectPoints[i].z = this->get_parameter(param_name_far_z).as_double();

        close_objectPoints[i].x = this->get_parameter(param_name_close_x).as_double();
        close_objectPoints[i].y = this->get_parameter(param_name_close_y).as_double();
        close_objectPoints[i].z = this->get_parameter(param_name_close_z).as_double();
    }

    // 图像坐标
    for (int i = 0; i < total_count; i++) {
        string parent_name_far = "calibrate_default_points.farCam.image_points.point";
        string param_name_x_far = parent_name_far + to_string(i+1) + ".x";
        string param_name_y_far = parent_name_far + to_string(i+1) + ".y";

        string parent_name_close = "calibrate_default_points.closeCam.image_points.point";
        string param_name_x_close = parent_name_close + to_string(i+1) + ".x";
        string param_name_y_close = parent_name_close + to_string(i+1) + ".y";

        // declare params
        this->declare_parameter<double>(param_name_x_far, 0);
        this->declare_parameter<double>(param_name_y_far, 0);
        this->declare_parameter<double>(param_name_x_close, 0);
        this->declare_parameter<double>(param_name_y_close, 0);

        //get params
        x = this->get_parameter(param_name_x_far).as_double();
        y = this->get_parameter(param_name_y_far).as_double();
        far_imagePoints[i] = cv::Point2d(x, y);
        far_imagePoints[i].x *= imgCols;
        far_imagePoints[i].y *= imgRows;
        cout << far_imagePoints[i] << "\t";

        x = this->get_parameter(param_name_x_close).as_double();
        y = this->get_parameter(param_name_y_close).as_double();
        close_imagePoints[i] = cv::Point2d(x, y);
        close_imagePoints[i].x *= imgCols;
        close_imagePoints[i].y *= imgRows;
        cout << close_imagePoints[i] << endl;
    }
}

void PnpSolver::DeclareParams() {
//    this->declare_parameter<int>("small_map_param.small_map_shift_X", 0);
//    this->declare_parameter<int>("small_map_param.small_map_shift_Y", 0);
//
    this->declare_parameter<std::string>("battle_state.battle_color", "empty");

    this->declare_parameter<int>("length_of_cloud_queue", 0);
    this->declare_parameter<int>("image_width", 0);
    this->declare_parameter<int>("image_height", 0);

    this->declare_parameter<std::vector<double>>("sensor_far.camera_matrix", {0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter<std::vector<double>>("sensor_far.distortion_coefficient", {0, 0, 0, 0, 0});
    this->declare_parameter<std::vector<double>>("sensor_far.uni_matrix", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    this->declare_parameter<std::vector<double>>("sensor_close.camera_matrix", {0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter<std::vector<double>>("sensor_close.distortion_coefficient", {0, 0, 0, 0, 0});
    this->declare_parameter<std::vector<double>>("sensor_close.uni_matrix", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
}

void PnpSolver::far_calibration(const radar_interfaces::msg::Points::SharedPtr msg) {
    std::cout << std::endl << "far 选点接收：" << std::endl;
    int count = 0;
    for (const auto &point: msg->data) {
        far_imagePoints[count] = cv::Point2f(point.x, point.y);
        cout << far_imagePoints[count].x << ", " << far_imagePoints[count].y << endl;
        count++;
    }
    cout << "已经选出了 " << total_count << " 个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat inlier;
    int suc = cv::solvePnPRansac(far_objectPoints, far_imagePoints, far_CamMatrix_, far_distCoeffs_, far_Rjacob,
                                 far_T,
                                 false, 100, 8.0, 0.99,
                                 inlier, cv::SOLVEPNP_AP3P);
    Rodrigues(far_Rjacob, far_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << far_R << endl;
    cout << "平移矩阵" << far_T << endl;

    write_far_Pnp_srv();
}

void PnpSolver::close_calibration(const radar_interfaces::msg::Points::SharedPtr msg) {
    std::cout << std::endl << "close 选点接收：" << std::endl;
    int count = 0;
    for (const auto &point: msg->data) {
        close_imagePoints[count] = cv::Point2f(point.x, point.y);
//        close_imagePoints[count].x *= imgCols;
//        close_imagePoints[count].y *= imgRows;
        cout << close_imagePoints[count] << endl;
        count++;
    }
    cout << "已经选出了 " << total_count << " 个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat inlier;
    cout << "close obj points:" << close_objectPoints << endl;
    int suc = cv::solvePnPRansac(close_objectPoints, close_imagePoints, close_CamMatrix_, close_distCoeffs_,
                                 close_Rjacob, close_T, false, 100, 8.0, 0.99,
                                 inlier, cv::SOLVEPNP_AP3P);
    Rodrigues(close_Rjacob, close_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << close_R << endl;
    cout << "平移矩阵" << close_T << endl;
    write_close_Pnp_srv();
}

void PnpSolver::write_far_Pnp_srv(){
    for (int i = 0; i < 3; i++) {
        pnp_result_response->far_t[i] = far_T.at<double>(i);
        for (int j = 0; j < 3; j++) {
            pnp_result_response->far_cam_matrix[3*i + j] = far_CamMatrix_.at<double>(i, j);
            pnp_result_response->far_r[3*i + j] = far_R.at<double>(i, j);
        }
    }
}

void PnpSolver::write_close_Pnp_srv(){
    for (int i = 0; i < 3; i++) {
        pnp_result_response->close_t[i] = close_T.at<double>(i);
        for (int j = 0; j < 3; j++) {
            pnp_result_response->close_cam_matrix[3*i + j] = close_CamMatrix_.at<double>(i, j);
            pnp_result_response->close_r[3*i + j] = close_R.at<double>(i, j);
        }
    }
}

void PnpSolver::PnpResultCallback(const radar_interfaces::srv::PnpResult::Request::SharedPtr request,
                                  radar_interfaces::srv::PnpResult::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Incoming request\nsmall_map_qidong: %d", request->small_map_qidong);
    if (request->small_map_qidong) {
        pnp_result_response->if_ok = true;
        response->if_ok = pnp_result_response->if_ok;
        response->far_t = pnp_result_response->far_t;
        response->far_cam_matrix = pnp_result_response->far_cam_matrix;
        response->far_r = pnp_result_response->far_r;
        response->close_t = pnp_result_response->close_t;
        response->close_cam_matrix = pnp_result_response->close_cam_matrix;
        response->close_r = pnp_result_response->close_r;
        RCLCPP_INFO(this->get_logger(), "sending back response:%lf", response->far_t[0]);
    }
}

