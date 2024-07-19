#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "pcl/point_cloud.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/segmentation/extract_clusters.h"
#include "radar_interfaces/msg/dist_point.hpp"
#include "radar_interfaces/msg/dist_points.hpp"
#include "radar_interfaces/msg/yolo_points.hpp"
#include "radar_interfaces/msg/points.hpp"

using namespace std;
using namespace cv;
using std::placeholders::_1;


uint16_t times = 0;
vector<int> cnt;
vector<float> dists;
int imgRows = 1080, imgCols = 1440;
int length_of_cloud_queue = 5;//default length is 5
radar_interfaces::msg::DistPoints far_distance_it;
radar_interfaces::msg::DistPoints close_distance_it;
radar_interfaces::msg::DistPoints last_far_distance_it;
radar_interfaces::msg::DistPoints last_close_distance_it;
radar_interfaces::msg::DistPoint outpost_distance_it;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
vector<Mat> far_depth_queue;
Mat far_camera_matrix = cv::Mat::zeros(3, 3, CV_64F); // 相机内参矩阵
Mat far_uni_matrix = cv::Mat::zeros(3, 4, CV_64F); // 相机和雷达的变换矩阵
Mat far_distortion_coefficient = cv::Mat::zeros(5, 1, CV_64F);
vector<Mat> close_depth_queue;
Mat close_camera_matrix = cv::Mat::zeros(3, 3, CV_64F); //　相机内参矩阵
Mat close_uni_matrix = cv::Mat::zeros(3, 4, CV_64F); // 相机和雷达的变换矩阵
Mat close_distortion_coefficient = cv::Mat::zeros(5, 1, CV_64F);
bool camera_params_load_done_ = false, pcl_load_done_ = false;

typedef struct {
    float Last_P;//上次估算协方差 不可以为0 ! ! ! ! !
    float Now_P;//当前估算协方差
    float out;//卡尔曼滤波器输出
    float Kg;//卡尔曼增益
    float Q;//过程噪声协方差
    float R;//观测噪声协方差
} Kalman;
Kalman kfp;

void Kalman_Init() {
    kfp.Last_P = 1;
    kfp.Now_P = 0;
    kfp.out = 0;
    kfp.Kg = 0;
    kfp.Q = 0;
    kfp.R = 0.01;
}


void write_csv(std::string filename, vector<float> vals);
float my_KalmanFilter(Kalman *kfp, float input);
Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input);
void MatProject(cv::Mat &input_depth, cv::Mat &input_uv, cv::Mat &Cam_matrix, cv::Mat &Uni_matrix); // Convert world to uv by Matrix_Calculation
void frame_point_match(const radar_interfaces::msg::DistPoints &last_frame, radar_interfaces::msg::DistPoints &this_frame);
double getDepthInRect(Rect rect, vector<Mat> &depth_queue,  radar_interfaces::msg::YoloPoint::_id_type id);


class GetDepth : public rclcpp::Node {
public:
    GetDepth(const string& name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "get depth node initialize!!!");

        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", 10, std::bind(&GetDepth::point_cloudCallback, this, std::placeholders::_1));
        far_yolo_subscription_ = this->create_subscription<radar_interfaces::msg::YoloPoints>(
                "/far_rectangles", 3, std::bind(&GetDepth::far_yoloCallback, this, std::placeholders::_1));
        close_yolo_subscription_ = this->create_subscription<radar_interfaces::msg::YoloPoints>(
                "/close_rectangles", 3, std::bind(&GetDepth::close_yoloCallback, this, std::placeholders::_1));

        far_dist_point_publisher = this->create_publisher<radar_interfaces::msg::DistPoints>(
                "/sensor_far/distance_point", 1);
        close_dist_point_publisher = this->create_publisher<radar_interfaces::msg::DistPoints>(
                "/sensor_close/distance_point", 1);

        while (!parameter_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the parameter service to start...");
        }
        this->LoadCameraParams();
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::YoloPoints>::SharedPtr far_yolo_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::YoloPoints>::SharedPtr close_yolo_subscription_;

    void point_cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr);
    void far_yoloCallback(radar_interfaces::msg::YoloPoints::SharedPtr);
    void close_yoloCallback(radar_interfaces::msg::YoloPoints::SharedPtr);

    rclcpp::Publisher<radar_interfaces::msg::DistPoints>::SharedPtr far_dist_point_publisher;
    rclcpp::Publisher<radar_interfaces::msg::DistPoints>::SharedPtr close_dist_point_publisher;
    rclcpp::Publisher<radar_interfaces::msg::DistPoint>::SharedPtr outpost_dist_point_publisher;

    std::shared_ptr<rclcpp::SyncParametersClient> parameter_client =
            std::make_shared<rclcpp::SyncParametersClient>(this, "pnp_solver");

    void LoadCameraParams();
};


int main(int argc, char **argv) {
    std::cout << "get_depth node init" << std::endl;
    rclcpp::init(argc, argv);
    auto GD_node = std::make_shared<GetDepth>("get_depth");

    Kalman_Init();

    cv::namedWindow("far_depth_show");
    cv::namedWindow("close_depth_show");
    cv::startWindowThread();

    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(GD_node);
        loop_rate.sleep();
    }

    return 0;
}


/**
 * 将受到的点云消息转换成点云
 * @param input 收到的消息
 */
void GetDepth::point_cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);
//    std::cout << "get cloud data, cloud size: " << cloud->size() << std::endl;
    pcl_load_done_ = true;
}


/**
 * update the car_rects
 * @param input
 */
void GetDepth::far_yoloCallback(radar_interfaces::msg::YoloPoints::SharedPtr input) {
    if (!camera_params_load_done_ || !pcl_load_done_) return;
//    std::cout << "enter far_yolo callback!!!" << std::endl;
    Mat far_depth_calc = Mat::zeros(imgRows, imgCols, CV_64FC1);//initialize the depth img 用于运算的深度图
    Mat far_depth_show = Mat::zeros(imgRows, imgCols, CV_64FC1); //用于显示的深度图
    std::vector<radar_interfaces::msg::DistPoint>().swap(far_distance_it.data);
    if (!cloud->empty()) {
        // 点云投影到深度图，并将深度图记录在队列里面
        Mat far_MatCloud = Cloud2Mat(cloud);  // 4*input->size
        MatProject(far_depth_calc, far_MatCloud, far_camera_matrix, far_uni_matrix);
        far_depth_calc.copyTo(far_depth_show);
        far_depth_queue.push_back(far_depth_calc);
        if ((int)far_depth_queue.size() == length_of_cloud_queue) {
            far_depth_queue.erase(far_depth_queue.begin());
        }
    }
    if ((*input).text != "none") {
        for (int j = 0; j < (int)(*input).data.size(); j++) {
            radar_interfaces::msg::DistPoint point_it;
            point_it.tracker_id = (*input).data[j].tracker_id;
            point_it.x = float( (*input).data[j].x) + float( (*input).data[j].width) / 2;
            point_it.y = float( (*input).data[j].y) + float( (*input).data[j].height) / 2;
            point_it.dist = (float)getDepthInRect(
                    Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                    far_depth_queue, (*input).data[j].id);
            point_it.color = (*input).data[j].color;
            point_it.id = (*input).data[j].id;
            far_distance_it.data.push_back(point_it);
            rectangle(far_depth_show,
                      Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                      Scalar(255, 255, 255), 1);
            putText(far_depth_show, std::to_string(point_it.dist), Point(point_it.x, point_it.y),
                    FONT_HERSHEY_COMPLEX_SMALL, 2,
                    Scalar(255, 255, 255), 1, 8, 0);
        }
    }
    this->far_dist_point_publisher->publish(far_distance_it);
    // TODO : cancel debug out
//    std::cout << std::endl << "far_dist_point_publish_one_msg: " << std::endl;
//    for (auto &i : close_distance_it.data) {
//        std::cout << "id : " << (int) i.id << ", [ " << i.x << " , " << i.y <<  " , " << i.dist << " ]" << std::endl;
//    }
    // TODO : end at here
    resize(far_depth_show, far_depth_show, Size((int)imgCols * 0.7, (int)imgRows * 0.7));
    imshow("far_depth_show", far_depth_show);
}

/**
 * update the car_rects
 * @param input
 */
void GetDepth::close_yoloCallback(radar_interfaces::msg::YoloPoints::SharedPtr input) {
    if (!camera_params_load_done_ || !pcl_load_done_) return;
//    std::cout << "enter close_yolo callback!!!" << std::endl;
    Mat close_depth_calc = Mat::zeros(imgRows, imgCols, CV_64FC1);//initialize the depth img 用于运算的深度图
    Mat close_depth_show = Mat::zeros(imgRows, imgCols, CV_64FC1); //用于显示的深度图
    std::vector<radar_interfaces::msg::DistPoint>().swap(close_distance_it.data);
    if (!cloud->empty()) {
        // 点云投影到深度图，并将深度图记录在队列里面
        Mat close_MatCloud = Cloud2Mat(cloud);  // 4*input->size
        MatProject(close_depth_calc, close_MatCloud, close_camera_matrix, close_uni_matrix);
        close_depth_calc.copyTo(close_depth_show);
        close_depth_queue.push_back(close_depth_calc);
        if ((int)close_depth_queue.size() == length_of_cloud_queue) {
            close_depth_queue.erase(close_depth_queue.begin());
        }
    }
    if ((*input).text != "none") {
        for (int j = 0; j < (int)(*input).data.size(); j++) {
            radar_interfaces::msg::DistPoint distPoint;
            distPoint.tracker_id = (*input).data[j].tracker_id;
            distPoint.x = float((*input).data[j].x) + float((*input).data[j].width) / 2;
            distPoint.y = float((*input).data[j].y) + float((*input).data[j].height) / 2;
            cv::Rect dist_rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height);
            distPoint.dist = (float)getDepthInRect(dist_rect, close_depth_queue, (*input).data[j].id);
            distPoint.color = (*input).data[j].color;
            distPoint.id = (*input).data[j].id;
            close_distance_it.data.push_back(distPoint);
            rectangle(close_depth_show,
                      Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                      Scalar(255, 255, 255), 1);
            putText(close_depth_show, std::to_string(distPoint.dist), Point(distPoint.x, distPoint.y),
                    FONT_HERSHEY_COMPLEX_SMALL, 2,
                    Scalar(255, 255, 255), 1, 8, 0);
        }
    }
    this->close_dist_point_publisher->publish(close_distance_it);
    // TODO : cancel debug out
//    std::cout << std::endl << "close_dist_point_publish_one_msg: " << std::endl;
//    for (auto &i : close_distance_it.data) {
//        std::cout << "id : " << (int) i.id << ", [ " << i.x << " , " << i.y <<  " , " << i.dist << " ]" << std::endl;
//    }
    // TODO : end at here
    resize(close_depth_show, close_depth_show, Size((int)imgCols * 0.7, (int)imgRows * 0.7));
    imshow("close_depth_show", close_depth_show);
}


/**
 * 从深度图中获取ROI的深度
 * @param rect ROI
 * @param depth_queue 深度图队列
 * @param id 车辆ID
 * @return 深度值
 */
double getDepthInRect(Rect rect, vector<Mat> &depth_queue,  radar_interfaces::msg::YoloPoint::_id_type id) {
    vector<double> distances;
    //从新到旧遍历深度图队列，直到ROI深度不为0
    for (int i = rect.y; i < (rect.y + rect.height); i++) {
        for (int j = rect.x; j < (rect.x + rect.width); j++) {
            if (depth_queue[depth_queue.size() - 1].at<double>(i, j) > 0) {
                distances.push_back(depth_queue[depth_queue.size() - 1].at<double>(i, j));
                continue;
            } else {
                for (uint8_t k = 0; k < depth_queue.size(); k++) {
                    if (k < depth_queue.size() - 1 && depth_queue[k + 1].at<double>(i, j) == 0 &&
                        depth_queue[k].at<double>(i, j) > 0) {
                        distances.push_back(depth_queue[k].at<double>(i, j));
                        break;
                    }
                }
            }
        }
    }
    if (distances.empty()) {
//        cout << "No Livox points in ROI " << rect << endl;
        return 0;
    } else {
        //根据不同的策略获取深度
        if (id != 12 && id != 13) {
            std::sort(distances.begin(), distances.end());
            auto min_iter = std::min_element(distances.begin(), distances.end());
            double min_dist = *min_iter;
//            if (min_dist >= 1.0) {
//                return min_dist;
//            } else {
//                double mean_distance = 0, sum = 0;
//                for (double distance: distances) sum += distance;
//                mean_distance = sum / distances.size();
//                return mean_distance;
//            }
            double mean_distance = 0, sum = 0;
            for (double distance: distances) sum += distance;
            mean_distance = sum / distances.size();
            return mean_distance;
        } else {
            sort(distances.begin(), distances.end());
            return distances[distances.size() / 2];
//            if (distances.size() >= 5){
//                for(uint8_t j=0;j<5;j++){
//                    sum+=distances[j];
//                }
//                mean_distance=sum/5;
            //return mean_distance;
//                return distances[distances.size()/2];
//            }
//            else {
//                return distances[0];
//                return distances[distances.size()/2];
//            }
        }
    }
}

/**
 * 用于在两帧图像中匹配同一目标，防止z值突变的情况发生
 * @param last_frame 上一帧图像中目标的坐标
 * @param this_frame 这一帧中目标的坐标radar_msgs::dist_points
 */
void frame_point_match(const radar_interfaces::msg::DistPoints &last_frame,
                       radar_interfaces::msg::DistPoints &this_frame) {
    bool match_suc_flag = false;
    for (int k = 0; k < (int)this_frame.data.size(); k++) {
        for (auto &i: last_frame.data) {
            if (abs(i.x - this_frame.data[k].x) < 50 && abs(i.y - this_frame.data[k].y) < 50) {
                this_frame.data[k].last_dist = i.dist;
                match_suc_flag = 1;
            }
        }
        if (!match_suc_flag) this_frame.data[k].last_dist = this_frame.data[k].dist;
        match_suc_flag = 0;
    }
}

/**
 * 将点云合并成矩阵，方便一同运算
 * @param input 输入点云
 * @return 输出矩阵
 */
Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input) {
    Mat res = Mat::zeros(4, (int) input->size(), CV_64F);
    for (int k = 0; k < res.cols; k++) {
        for (int j = 0; j < 4; j++) {
            res.at<double>(j, k) = input->points[k].data[j];
        }
    }
    return res;
}

/**
 * 用于将点云拼成的矩阵变换至像素坐标
 * @param input_depth 输入的点云矩阵
 * @param input_uv 输出
 * @param Cam_matrix 相机内参
 * @param Uni_matrix 相机外参
 */
void MatProject(cv::Mat &input_depth, cv::Mat &input_uv, cv::Mat &Cam_matrix, cv::Mat &Uni_matrix) {
    Mat res = Cam_matrix * Uni_matrix * input_uv;
    for (int i = 0; i < res.cols; i++) {
        int x = round(res.at<double>(0, i) / res.at<double>(2, i));
        int y = round(res.at<double>(1, i) / res.at<double>(2, i));
        if (x >= 0 && x < imgCols && y >= 0 && y < imgRows) {
            input_depth.at<double>(y, x) = res.at<double>(2, i);
        }
    }
}

void GetDepth::LoadCameraParams() {
    std::vector<rclcpp::Parameter> param_vector;
    param_vector = parameter_client->get_parameters(
            {"length_of_cloud_queue", "image_width", "image_height"});
    length_of_cloud_queue = (int)param_vector[0].as_int();
    imgCols = param_vector[1].as_int();
    imgRows = param_vector[2].as_int();
    std::cout << "length_of_pcl_queue, img_width, img_height" << std::endl
    << length_of_cloud_queue << "\t" << imgCols << "\t" << imgRows << "\t" << std::endl;

    // far 相机内参
    std::vector<double> temp_array;
    param_vector = parameter_client->get_parameters(
            {"sensor_far.camera_matrix", "sensor_far.distortion_coefficient",
            "sensor_close.camera_matrix", "sensor_close.distortion_coefficient"}
            );
    temp_array = param_vector[0].as_double_array();
    for (int i = 0; i < far_camera_matrix.rows; ++i) {
        for (int j = 0; j < far_camera_matrix.cols; ++j) {
            far_camera_matrix.at<double>(i, j) = temp_array[i * far_camera_matrix.rows + j];
        }
    }
    cout << endl << "far_Camera matrix load done!" << endl << far_camera_matrix << endl;

    temp_array = param_vector[1].as_double_array();
    for (int i = 0; i < far_distortion_coefficient.rows; ++i) {
        far_distortion_coefficient.at<double>(i, 0) = temp_array[i];
    }
    cout << endl << "far_Distortion coefficient load done!" << endl << far_distortion_coefficient << endl;

    temp_array = param_vector[2].as_double_array();
    for (int i = 0; i < close_camera_matrix.rows; ++i) {
        for (int j = 0; j < close_camera_matrix.cols; ++j) {
            close_camera_matrix.at<double>(i, j) = temp_array[i * close_camera_matrix.rows + j];
        }
    }
    cout << endl << "close_Camera matrix load done!" << endl << close_camera_matrix << endl;

    temp_array = param_vector[3].as_double_array();
    for (int i = 0; i < close_distortion_coefficient.rows; ++i) {
        close_distortion_coefficient.at<double>(i, 0) = temp_array[i];
    }
    cout << endl << "close_Distortion coefficient load done!" << endl << close_distortion_coefficient << endl;
    
    // 相机外参默认值
    param_vector = parameter_client->get_parameters({"sensor_far.uni_matrix"});
    temp_array = param_vector[0].as_double_array();
    for (int i = 0; i < far_uni_matrix.rows; ++i) {
        for (int j = 0; j < far_uni_matrix.cols; ++j) {
            far_uni_matrix.at<double>(i, j) = temp_array[i * far_uni_matrix.cols + j];
        }
    }
    cout << endl << "far Uni matrix load done!" << endl << far_uni_matrix << endl;

    param_vector = parameter_client->get_parameters({"sensor_close.uni_matrix"});
    temp_array = param_vector[0].as_double_array();
    for (int i = 0; i < close_uni_matrix.rows; ++i) {
        for (int j = 0; j < close_uni_matrix.cols; ++j) {
            close_uni_matrix.at<double>(i, j) = temp_array[i * close_uni_matrix.cols + j];
        }
    }
    cout << endl << "close Uni matrix load done!" << endl << close_uni_matrix << endl;

    camera_params_load_done_ = true;
}

void write_csv(std::string filename, vector<float> vals) {
    // Create an output filestream object
    ofstream myFile(filename);
    // Send data to the stream
    for (float val: vals) {
        myFile << val << "\n";
    }
    // Close the file
    myFile.close();
}

float my_KalmanFilter(Kalman *kfp, float input) {
    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->Now_P = kfp->Last_P + kfp->Q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->out = kfp->out + kfp->Kg * (input - kfp->out);//因为这一次的预测值就是上一次的输出值
    //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->Last_P = (1 - kfp->Kg) * kfp->Now_P;
    return kfp->out;
}