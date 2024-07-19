#ifndef GET_DEPTH_COMPONENT_HPP
#define GET_DEPTH_COMPONENT_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

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


void write_csv(std::string filename, vector<float> vals);
void frame_point_match(const radar_interfaces::msg::DistPoints &last_frame, radar_interfaces::msg::DistPoints &this_frame);


namespace get_depth{
    class DepthSensor : public rclcpp::Node {
    public:
        DepthSensor(const rclcpp::NodeOptions &options);
        ~DepthSensor();

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
        rclcpp::Subscription<radar_interfaces::msg::YoloPoints>::SharedPtr yolo_subscription_;

        void point_cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr);
        void yolo_Callback(radar_interfaces::msg::YoloPoints::SharedPtr);
        Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input);
        void MatProject(cv::Mat &input_depth, cv::Mat &input_uv, cv::Mat &Cam_matrix, cv::Mat &Uni_matrix); // Convert world to uv by Matrix_Calculation
        float GetDepthInRect(Rect rect, std::deque<Mat> &depth_list, radar_interfaces::msg::YoloPoint::_id_type id);

        rclcpp::Publisher<radar_interfaces::msg::DistPoints>::SharedPtr dist_point_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dist_img_publisher_;

//        std::shared_ptr<rclcpp::SyncParametersClient> parameter_client =
//                std::make_shared<rclcpp::SyncParametersClient>(this, "pnp_solver");

        void LoadCameraParams();

        uint16_t times = 0;
        string camera_name = "camera_init", win_name;
        vector<int> cnt;
        vector<float> dists;
        int imgRows = 1200, imgCols = 1920;
        int length_of_cloud_queue = 5, show_by_cv_or_msg = 0;//default length is 5
        radar_interfaces::msg::DistPoints distance_points;
        radar_interfaces::msg::DistPoints last_distance_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        std::deque<Mat> depth_queue;
        Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F); //　相机内参矩阵
        Mat uni_matrix = cv::Mat::zeros(3, 4, CV_64F); // 相机和雷达的变换矩阵
        Mat distortion_coefficient = cv::Mat::zeros(5, 1, CV_64F);
        bool camera_params_load_done_ = false, pcl_load_done_ = false;
        cv::Mat show_img;
        int show_count = 0, show_threshold = 4;
    };
}
#endif  // GET_DEPTH_COMPONENT_HPP