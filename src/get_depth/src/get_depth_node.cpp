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
int imgRows = 1024, imgCols = 1280;
int length_of_cloud_queue = 5;//default length is 5
int post_pub_flag = 0;
Point2f outpost_point;
//ros::Publisher far_distancePointPub;
//ros::Publisher close_distancePointPub;
//ros::Publisher outpost_distancePointPub;
radar_interfaces::msg::DistPoints far_distance_it;
radar_interfaces::msg::DistPoints close_distance_it;
radar_interfaces::msg::DistPoints last_far_distance_it;
radar_interfaces::msg::DistPoints last_close_distance_it;
radar_interfaces::msg::DistPoint outpost_distance_it;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
vector<Mat> far_depth_queue;
Mat far_camera_matrix = Mat_<float>(3, 3);//相机内参矩阵
Mat far_uni_matrix = Mat_<float>(3, 4);//相机和雷达的变换矩阵
Mat far_distortion_coefficient = Mat_<float>(5, 1);
vector<Mat> close_depth_queue;
Mat close_camera_matrix = Mat_<float>(3, 3);//相机内参矩阵
Mat close_uni_matrix = Mat_<float>(3, 4);//相机和雷达的变换矩阵
Mat close_distortion_coefficient = Mat_<float>(5, 1);


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
Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);
void MatProject(Mat &input_depth, Mat &input_uv, Mat &Cam_matrix, Mat &Uni_matrix);//Convert world to uv by Matrix_Calculation
void frame_point_match(const radar_interfaces::msg::DistPoints &last_frame, radar_interfaces::msg::DistPoints &this_frame);
float getDepthInRect(Rect rect, vector<Mat> &depth_queue,  radar_interfaces::msg::YoloPoint::_id_type id);


class GetDepth : public rclcpp::Node {
public:
    GetDepth(const string& name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "get depth node initialize!!!");
        this->LoadCameraParams();

        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", 1, std::bind(&GetDepth::point_cloudCallback, this, std::placeholders::_1));
        far_yolo_subscription_ = this->create_subscription<radar_interfaces::msg::YoloPoints>(
                "/far_rectangles", 1, std::bind(&GetDepth::far_yoloCallback, this, std::placeholders::_1));
        close_yolo_subscription_ = this->create_subscription<radar_interfaces::msg::YoloPoints>(
                "/close_rectangles", 1, std::bind(&GetDepth::close_yoloCallback, this, std::placeholders::_1));
        outpost_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
                "/sensor_far/calibration", 1, std::bind(&GetDepth::outpost_Callback, this, std::placeholders::_1));

        far_dist_point_publisher = this->create_publisher<radar_interfaces::msg::DistPoints>(
                "/sensor_far/distance_point", 1);
        close_dist_point_publisher = this->create_publisher<radar_interfaces::msg::DistPoints>(
                "/sensor_close/distance_point", 1);
        outpost_dist_point_publisher = this->create_publisher<radar_interfaces::msg::DistPoint>(
                "/sensor_far/outpost", 1);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::YoloPoints>::SharedPtr far_yolo_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::YoloPoints>::SharedPtr close_yolo_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr outpost_subscription_;

    void point_cloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr);
    void far_yoloCallback(radar_interfaces::msg::YoloPoints::SharedPtr);
    void close_yoloCallback(radar_interfaces::msg::YoloPoints::SharedPtr);
    void outpost_Callback(radar_interfaces::msg::Points::SharedPtr);

    rclcpp::Publisher<radar_interfaces::msg::DistPoints>::SharedPtr far_dist_point_publisher;
    rclcpp::Publisher<radar_interfaces::msg::DistPoints>::SharedPtr close_dist_point_publisher;
    rclcpp::Publisher<radar_interfaces::msg::DistPoint>::SharedPtr outpost_dist_point_publisher;

    void LoadCameraParams();
};


int main(int argc, char **argv) {
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto GD_node = std::make_shared<GetDepth>("get_depth");
    Kalman_Init();

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
void GetDepth::point_cloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr input) {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
}


/**
 * update the car_rects
 * @param input
 */
void GetDepth::far_yoloCallback(radar_interfaces::msg::YoloPoints::SharedPtr input) {
    Mat far_depthes = Mat::zeros(imgRows, imgCols, CV_32FC1);//initialize the depth img 用于运算的深度图
    Mat far_depth_show = Mat::zeros(imgRows, imgCols, CV_32FC1); //用于显示的深度图
    std::vector<radar_interfaces::msg::DistPoint>().swap(far_distance_it.data);
    if (cloud) {
        Mat far_MatCloud = Cloud2Mat(cloud);
        MatProject(far_depthes, far_MatCloud, far_camera_matrix, far_uni_matrix);
        far_depthes.copyTo(far_depth_show);
        far_depth_queue.push_back(far_depthes); //调整队列
        if ((int)far_depth_queue.size() == length_of_cloud_queue) {
            far_depth_queue.erase(far_depth_queue.begin());
        }
    }
    if ((*input).text != "none") {
        for (int j = 0; j < (int)(*input).data.size(); j++) {
            radar_interfaces::msg::DistPoint point_it;
            point_it.x = (*input).data[j].x + (*input).data[j].width / 2;
            point_it.y = (*input).data[j].y + (*input).data[j].height / 2;
            point_it.dist = getDepthInRect(
                    Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                    far_depth_queue, (*input).data[j].id);
            point_it.color = (*input).data[j].color;
            point_it.id = (*input).data[j].id;
            far_distance_it.data.push_back(point_it);
            rectangle(far_depth_show,
                      Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                      Scalar(255, 255, 255), 1);
            putText(far_depth_show, std::to_string(point_it.dist), Point(point_it.x, point_it.y),
                    FONT_HERSHEY_COMPLEX_SMALL, 1,
                    Scalar(255, 255, 255), 1, 8, 0);

            //下面用来辅助英雄测距
            if (post_pub_flag == 1) {
                outpost_distance_it.x = outpost_point.x - 6;
                outpost_distance_it.y = outpost_point.y - 6;
                outpost_distance_it.dist = getDepthInRect(
                        Rect((int) outpost_distance_it.x, (int) outpost_point.y, 12, 12), far_depth_queue, 0);
                outpost_distance_it.color = 3;
                outpost_distance_it.id = 14;
                this->outpost_dist_point_publisher->publish(outpost_distance_it);
//                outpost_distancePointPub.publish(outpost_distance_it);
                rectangle(far_depth_show,
                          Rect((int) outpost_distance_it.x, (int) outpost_point.y, 12, 12),
                          Scalar(255, 255, 255), 1);
                putText(far_depth_show, std::to_string(outpost_distance_it.dist),
                        Point(outpost_distance_it.x, outpost_distance_it.y),
                        FONT_HERSHEY_COMPLEX_SMALL, 1,
                        Scalar(255, 255, 255), 1, 8, 0);
            }
        }
    }
    this->far_dist_point_publisher->publish(far_distance_it);
//    far_distancePointPub.publish(far_distance_it);
    resize(far_depth_show, far_depth_show, Size(960, 768));
//    imshow("far_depth_show", far_depth_show);
//    waitKey(1);
}

/**
 * update the car_rects
 * @param input
 */
void GetDepth::close_yoloCallback(radar_interfaces::msg::YoloPoints::SharedPtr input) {
    Mat close_depthes = Mat::zeros(imgRows, imgCols, CV_32FC1);//initialize the depth img
    Mat close_depth_show = Mat::zeros(imgRows, imgCols, CV_32FC1);
    std::vector<radar_interfaces::msg::DistPoint>().swap(last_close_distance_it.data);
    for (auto &i: close_distance_it.data) {
        last_close_distance_it.data.emplace_back(i);
    }
    std::vector<radar_interfaces::msg::DistPoint>().swap(close_distance_it.data);
    if (cloud) {
        Mat close_MatCloud = Cloud2Mat(cloud);
//        imshow("close_MatCloud", close_MatCloud);
        MatProject(close_depthes, close_MatCloud, close_camera_matrix, close_uni_matrix);
//        imshow("close_depth_show", close_depth_show);
        close_depthes.copyTo(close_depth_show);
        close_depth_queue.push_back(close_depthes);
        if ((int)close_depth_queue.size() == length_of_cloud_queue) {
            close_depth_queue.erase(close_depth_queue.begin());
        }
    }
    if ((*input).text != "none") {
        for (int j = 0; j < (int)(*input).data.size(); j++) {
            radar_interfaces::msg::DistPoint point_it;
            point_it.x = (*input).data[j].x + (*input).data[j].width / 2;
            point_it.y = (*input).data[j].y + (*input).data[j].height / 2;
            point_it.dist = getDepthInRect(
                    Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                    close_depth_queue, (*input).data[j].id);
//            if((*input).data[j].id==10&&(*input).data[j].x>320)
//            {
//                times++;
//                if(times<18)
//                {
//                    point_it.dist= my_KalmanFilter(&kfp,point_it.dist);
//                    float a=my_KalmanFilter(&kfp,point_it.dist);
//                    dists.emplace_back(a);
//                }
//                if(times==18)
//                {
//                    write_csv("dist_kalman.csv",dists);
//                    std::vector<float>().swap(dists);
//                    times=0;
//                }
//            }
            point_it.color = (*input).data[j].color;
            point_it.id = (*input).data[j].id;
            close_distance_it.data.push_back(point_it);
            rectangle(close_depth_show,
                      Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                      Scalar(255, 255, 255), 1);
            putText(close_depth_show, std::to_string(point_it.dist), Point(point_it.x, point_it.y),
                    FONT_HERSHEY_COMPLEX_SMALL, 1,
                    Scalar(255, 255, 255), 1, 8, 0);
        }
        frame_point_match(last_close_distance_it, close_distance_it);
        for (auto &i: close_distance_it.data) {
            if (i.id == 10 && i.x > 320) {
                if (i.dist - i.last_dist > 0.3 || i.dist - i.last_dist < -0.3) {
                    i.dist = i.last_dist;
                } else
                    i.dist = i.dist * 0.8 + i.last_dist * 0.2;
            }
        }
    }
//    close_distancePointPub.publish(close_distance_it);
    this->close_dist_point_publisher->publish(close_distance_it);
    resize(close_depth_show, close_depth_show, Size(960, 768));
    imshow("close_depth_show", close_depth_show);
    waitKey(1);
}



/**
 * 用于接收前哨站位置消息
 * @param outpost 前哨站位置
 */
void GetDepth::outpost_Callback(radar_interfaces::msg::Points::SharedPtr outpost) {
    post_pub_flag = 1;
    outpost_point.x = outpost->data[0].x * 1280;
    outpost_point.y = outpost->data[0].y * 1024;
}

/**
 * 从深度图中获取ROI的深度
 * @param rect ROI
 * @param depth_queue 深度图队列
 * @param id 车辆ID
 * @return 深度值
 */
float getDepthInRect(Rect rect, vector<Mat> &depth_queue,  radar_interfaces::msg::YoloPoint::_id_type id) {
    vector<float> distances;
    //从新到旧遍历深度图队列，直到ROI深度不为0
    for (int i = rect.y; i < (rect.y + rect.height); i++) {
        for (int j = rect.x; j < (rect.x + rect.width); j++) {
            for (uint8_t k = 0; k < depth_queue.size(); k++) {
                if (depth_queue[depth_queue.size() - 1].at<float>(i, j) > 0) {
                    distances.push_back(depth_queue[depth_queue.size() - 1].at<float>(i, j));
                    break;
                } else if (k < depth_queue.size() - 1 && depth_queue[k + 1].at<float>(i, j) == 0 &&
                           depth_queue[k].at<float>(i, j) > 0) {
                    distances.push_back(depth_queue[k].at<float>(i, j));
                    break;
                }
            }
        }
    }
    if (distances.empty()) {
        cout << "No Livox points in ROI " << rect << endl;
        return 0;
    } else {
        float mean_distance;
        float sum = 0;
        //根据不同的策略获取深度
        if (id != 12 && id != 13) {
            for (float distance: distances) {
                sum += distance;
            }
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
    uint8_t match_suc_flag = 0;
    for (int k = 0; k < (int)this_frame.data.size(); k++) {
        for (auto &i: last_frame.data) {
            if ((i.x - this_frame.data[k].x > -50 && i.x - this_frame.data[k].x < 50) &&
                (i.y - this_frame.data[k].y < 50 && i.y - this_frame.data[k].y > -50)) {
                this_frame.data[k].last_dist = i.dist;
                match_suc_flag = 1;
            }
        }
        if (match_suc_flag == 0)this_frame.data[k].last_dist = this_frame.data[k].dist;
        match_suc_flag = 0;
    }
}

/**
 * 将点云合并成矩阵，方便一同运算
 * @param input 输入点云
 * @return 输出矩阵
 */
Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input) {
    Mat res = Mat::zeros(4, (int) input->size(), CV_32F);
    for (int k = 0; k < res.cols; k++) {
        for (int j = 0; j < 4; j++) {
            res.at<float>(j, k) = input->points[k].data[j];
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
void MatProject(Mat &input_depth, Mat &input_uv, Mat &Cam_matrix, Mat &Uni_matrix) {
    Mat res = Cam_matrix * Uni_matrix * input_uv;
    for (int i = 0; i < res.cols; i++) {
        int x = round(res.at<float>(0, i) / res.at<float>(2, i));
        int y = round(res.at<float>(1, i) / res.at<float>(2, i));
        if (x >= 0 && x < imgCols && y >= 0 && y < imgRows) {
            input_depth.at<float>(y, x) = res.at<float>(2, i);
        }
    }
}

void GetDepth::LoadCameraParams() {
    // 创建参数客户端
    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this, "pnp_solver");

    while (!parameter_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
//            return -1;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for the parameter service to start...");
    }

    std::vector<rclcpp::Parameter> param_vector;
    param_vector = parameter_client->get_parameters(
            {"length_of_cloud_queue", "image_width", "image_height"});
    length_of_cloud_queue = param_vector[0].as_int();
    imgCols = param_vector[1].as_int();
    imgRows = param_vector[2].as_int();

    // far 相机内参
    param_vector = parameter_client->get_parameters(
            {"sensor_far.camera_matrix.zerozero", "sensor_far.camera_matrix.zerotwo",
             "sensor_far.camera_matrix.oneone", "sensor_far.camera_matrix.onetwo", "sensor_far.camera_matrix.twotwo"});
    far_camera_matrix.at<float>(0, 0) = param_vector[0].as_double();
    far_camera_matrix.at<float>(0, 2) = param_vector[1].as_double();
    far_camera_matrix.at<float>(1, 1) = param_vector[2].as_double();
    far_camera_matrix.at<float>(1, 2) = param_vector[3].as_double();
    far_camera_matrix.at<float>(2, 2) = param_vector[4].as_double();
    far_camera_matrix.at<float>(0, 1) = 0;
    far_camera_matrix.at<float>(1, 0) = 0;
    far_camera_matrix.at<float>(2, 0) = 0;
    far_camera_matrix.at<float>(2, 1) = 0;
    cout << endl << "far_Camera matrix load done!" << endl << far_camera_matrix << endl;

    param_vector = parameter_client->get_parameters(
            {"sensor_far.distortion_coefficient.zero", "sensor_far.distortion_coefficient.one", 
            "sensor_far.distortion_coefficient.two", "sensor_far.distortion_coefficient.three", 
            "sensor_far.distortion_coefficient.four"}
            );
    far_distortion_coefficient.at<float>(0, 0) = param_vector[0].as_double();
    far_distortion_coefficient.at<float>(1, 0) = param_vector[1].as_double();
    far_distortion_coefficient.at<float>(2, 0) = param_vector[2].as_double();
    far_distortion_coefficient.at<float>(3, 0) = param_vector[3].as_double();
    far_distortion_coefficient.at<float>(4, 0) = param_vector[4].as_double();
    cout << endl << "far_Distortion coefficient load done!" << endl << far_distortion_coefficient << endl;

    param_vector = parameter_client->get_parameters(
            {"sensor_close.camera_matrix.zerozero", "sensor_close.camera_matrix.zerotwo",
             "sensor_close.camera_matrix.oneone", "sensor_close.camera_matrix.onetwo", "sensor_close.camera_matrix.twotwo"});
    close_camera_matrix.at<float>(0, 0) = param_vector[0].as_double();
    close_camera_matrix.at<float>(0, 2) = param_vector[1].as_double();
    close_camera_matrix.at<float>(1, 1) = param_vector[2].as_double();
    close_camera_matrix.at<float>(1, 2) = param_vector[3].as_double();
    close_camera_matrix.at<float>(2, 2) = param_vector[4].as_double();
    close_camera_matrix.at<float>(0, 1) = 0;
    close_camera_matrix.at<float>(1, 0) = 0;
    close_camera_matrix.at<float>(2, 0) = 0;
    close_camera_matrix.at<float>(2, 1) = 0;
    cout << endl << "close_Camera matrix load done!" << endl << close_camera_matrix << endl;

    param_vector = parameter_client->get_parameters(
            {"sensor_close.distortion_coefficient.zero", "sensor_close.distortion_coefficient.one",
             "sensor_close.distortion_coefficient.two", "sensor_close.distortion_coefficient.three",
             "sensor_close.distortion_coefficient.four"}
    );
    close_distortion_coefficient.at<float>(0, 0) = param_vector[0].as_double();
    close_distortion_coefficient.at<float>(1, 0) = param_vector[1].as_double();
    close_distortion_coefficient.at<float>(2, 0) = param_vector[2].as_double();
    close_distortion_coefficient.at<float>(3, 0) = param_vector[3].as_double();
    close_distortion_coefficient.at<float>(4, 0) = param_vector[4].as_double();
    cout << endl << "close_Distortion coefficient load done!" << endl << close_distortion_coefficient << endl;

    // 相机外参默认值
    param_vector = parameter_client->get_parameters(
            {"sensor_far.uni_matrix.zerozero", "sensor_far.uni_matrix.zeroone", "sensor_far.uni_matrix.zerotwo", 
             "sensor_far.uni_matrix.zerothree", "sensor_far.uni_matrix.onezero", "sensor_far.uni_matrix.oneone", 
             "sensor_far.uni_matrix.onetwo", "sensor_far.uni_matrix.onethree", "sensor_far.uni_matrix.twozero", 
            "sensor_far.uni_matrix.twoone", "sensor_far.uni_matrix.twotwo","sensor_far.uni_matrix.twothree"}
            );
    far_uni_matrix.at<float>(0, 0) = param_vector[0].as_double();
    far_uni_matrix.at<float>(0, 1) = param_vector[1].as_double();
    far_uni_matrix.at<float>(0, 2) = param_vector[2].as_double();
    far_uni_matrix.at<float>(0, 3) = param_vector[3].as_double();
    far_uni_matrix.at<float>(1, 0) = param_vector[4].as_double();
    far_uni_matrix.at<float>(1, 1) = param_vector[5].as_double();
    far_uni_matrix.at<float>(1, 2) = param_vector[6].as_double();
    far_uni_matrix.at<float>(1, 3) = param_vector[7].as_double();
    far_uni_matrix.at<float>(2, 0) = param_vector[8].as_double();
    far_uni_matrix.at<float>(2, 1) = param_vector[9].as_double();
    far_uni_matrix.at<float>(2, 2) = param_vector[10].as_double();
    far_uni_matrix.at<float>(2, 3) = param_vector[11].as_double();
    cout << endl << "far Uni matrix load done!" << endl << far_uni_matrix << endl;

    param_vector = parameter_client->get_parameters(
            {"sensor_close.uni_matrix.zerozero", "sensor_close.uni_matrix.zeroone", "sensor_close.uni_matrix.zerotwo",
             "sensor_close.uni_matrix.zerothree", "sensor_close.uni_matrix.onezero", "sensor_close.uni_matrix.oneone",
             "sensor_close.uni_matrix.onetwo", "sensor_close.uni_matrix.onethree", "sensor_close.uni_matrix.twozero",
             "sensor_close.uni_matrix.twoone", "sensor_close.uni_matrix.twotwo","sensor_close.uni_matrix.twothree"}
    );
    close_uni_matrix.at<float>(0, 0) = param_vector[0].as_double();
    close_uni_matrix.at<float>(0, 1) = param_vector[1].as_double();
    close_uni_matrix.at<float>(0, 2) = param_vector[2].as_double();
    close_uni_matrix.at<float>(0, 3) = param_vector[3].as_double();
    close_uni_matrix.at<float>(1, 0) = param_vector[4].as_double();
    close_uni_matrix.at<float>(1, 1) = param_vector[5].as_double();
    close_uni_matrix.at<float>(1, 2) = param_vector[6].as_double();
    close_uni_matrix.at<float>(1, 3) = param_vector[7].as_double();
    close_uni_matrix.at<float>(2, 0) = param_vector[8].as_double();
    close_uni_matrix.at<float>(2, 1) = param_vector[9].as_double();
    close_uni_matrix.at<float>(2, 2) = param_vector[10].as_double();
    close_uni_matrix.at<float>(2, 3) = param_vector[11].as_double();
    cout << endl << "close Uni matrix load done!" << endl << close_uni_matrix << endl;
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