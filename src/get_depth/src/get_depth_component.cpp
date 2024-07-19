#include "get_depth_component.hpp"

namespace get_depth {

    DepthSensor::DepthSensor(const rclcpp::NodeOptions &options)
    : Node("get_depth", options) {
        RCLCPP_INFO(this->get_logger(), "get depth node initialize!!!");

        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "livox/lidar", 1, std::bind(&DepthSensor::point_cloudCallback, this, std::placeholders::_1));
        yolo_subscription_ = this->create_subscription<radar_interfaces::msg::YoloPoints>(
                "rectangles", 1, std::bind(&DepthSensor::yolo_Callback, this, std::placeholders::_1));
        //TODO 在这里加一个时间匹配，只有时间相似才进行计算
        dist_point_publisher = this->create_publisher<radar_interfaces::msg::DistPoints>("distance_point", 1);
        dist_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("dist_img", 1);

        this->LoadCameraParams();
        win_name = camera_name + " dist_show";
        if (show_by_cv_or_msg == 0) {
            cv::namedWindow(win_name);
            cv::startWindowThread();
        }
    }

    DepthSensor::~DepthSensor() {
        cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "get depth node shutdown!!!");
    }

/**
 * 将受到的点云消息转换成点云
 * @param input 收到的消息
 */
    void DepthSensor::point_cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input, *cloud);
        pcl_load_done_ = true;
    }


/**
 * update the car_rects
 * @param input
 */
    void DepthSensor::yolo_Callback(radar_interfaces::msg::YoloPoints::SharedPtr input) {
        if (!camera_params_load_done_ || !pcl_load_done_) return;
//    std::cout << "enter close_yolo callback!!!" << std::endl;
        Mat depth_calc = Mat::zeros(imgRows, imgCols, CV_64FC1);//initialize the depth img
        Mat depth_show = Mat::zeros(imgRows, imgCols, CV_64FC1);
        std::vector<radar_interfaces::msg::DistPoint>().swap(last_distance_points.data);
        for (auto &i: distance_points.data) {
            last_distance_points.data.emplace_back(i);
        }
        std::vector<radar_interfaces::msg::DistPoint>().swap(distance_points.data);
        if (!cloud->empty()) {
            Mat MatCloud = Cloud2Mat(cloud);
            MatProject(depth_calc, MatCloud, camera_matrix, uni_matrix);
            depth_calc.copyTo(depth_show);
            depth_queue.push_back(depth_calc);
            if ((int)depth_queue.size() == length_of_cloud_queue) {
                depth_queue.pop_front();
            }
            pcl_load_done_ = false;
        }
        if ((*input).text != "none") {
            for (auto & point : (*input).data) {
                radar_interfaces::msg::DistPoint distPoint;
                distPoint.x = point.x + point.width / 2;
                distPoint.y = point.y + point.height / 2;
                cv::Rect dist_rect(point.x, point.y, point.width, point.height);
                distPoint.dist = (float)GetDepthInRect(dist_rect, depth_queue, point.id);
                distPoint.color = point.color;
                distPoint.id = point.id;
                distPoint.conf = point.conf;
                distance_points.data.push_back(distPoint);
                rectangle(depth_show, Rect(point.x, point.y, point.width,
                                           point.height), Scalar(255), 2);
                std::stringstream dist_stream;
                dist_stream << std::fixed << std::setprecision(3) << distPoint.dist;
                std::string dist_str = dist_stream.str();
                putText(depth_show, dist_str, Point(distPoint.x, distPoint.y),
                        FONT_HERSHEY_COMPLEX_SMALL, 2,
                        Scalar(255), 2, 8, 0);
            }
//        frame_point_match(last_close_distance_it, close_distance_it);
//        for (auto &i: close_distance_it.data) {
//            if (abs(i.dist - i.last_dist) > 0.3){
//                i.dist = i.last_dist;
//            } else {
//                i.dist = i.dist * 0.8 + i.last_dist * 0.2;
//            }
//        }
        }
        this->dist_point_publisher->publish(distance_points);

        if (show_count++ > show_threshold) {
            show_count = 0;
            show_img = depth_show;
        }
        if (show_by_cv_or_msg == 1) {
            cv_bridge::CvImage dist_img;
            dist_img.encoding = "32FC1";
            dist_img.header.stamp = this->now();
            dist_img.image = show_img;
            sensor_msgs::msg::Image dist_img_msg;
            dist_img.toImageMsg(dist_img_msg);
            this->dist_img_publisher_->publish(dist_img_msg);
        } else if (show_by_cv_or_msg == 0) {
            cv::resize(depth_show, depth_show, cv::Size(depth_show.cols/2, depth_show.rows/2));
            cv::imshow(win_name, depth_show);
        }

    }


    void DepthSensor::LoadCameraParams() {
        this->declare_parameter<std::string>("camera_name", "camera_init");
        camera_name = this->get_parameter("camera_name").as_string();
        this->declare_parameter<int>("show_by_cv_or_msg", 0);
        show_by_cv_or_msg = this->get_parameter("show_by_cv_or_msg").as_int();

        this->declare_parameter<int>("length_of_cloud_queue", 10);
        this->declare_parameter<int>("image_width", 1920);
        this->declare_parameter<int>("image_height", 1200);
        length_of_cloud_queue = (int)this->get_parameter("length_of_cloud_queue").as_int();
        imgCols = (int)this->get_parameter("image_width").as_int();
        imgRows = (int)this->get_parameter("image_height").as_int();

        this->declare_parameter<std::vector<double>>("camera_matrix", {0, 0, 0, 0, 0, 0, 0, 0, 0});
        this->declare_parameter<std::vector<double>>("distortion_coefficient", {0, 0, 0, 0, 0});
        this->declare_parameter<std::vector<double>>("uni_matrix", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
        // 相机内参
        std::vector<double> double_param_array = this->get_parameter("camera_matrix").as_double_array();
        for (int i = 0; i < camera_matrix.rows; ++i) {
            for (int j = 0; j < camera_matrix.cols; ++j) {
                camera_matrix.at<double>(i, j) = double_param_array[i * camera_matrix.rows + j];
            }
        }
        cout << endl << camera_name << "--camera_matrix load done!" << endl << camera_matrix << endl;

        double_param_array = this->get_parameter("distortion_coefficient").as_double_array();
        for (int i = 0; i < distortion_coefficient.rows; ++i) {
            distortion_coefficient.at<double>(i, 0) = double_param_array[i];
        }
        cout << endl << camera_name << "--Distortion coefficient load done!" << endl << distortion_coefficient << endl;

        // 雷达站外参
        double_param_array = this->get_parameter("uni_matrix").as_double_array();
        for (int i = 0; i < uni_matrix.rows; ++i) {
            for (int j = 0; j < uni_matrix.cols; ++j) {
                uni_matrix.at<double>(i, j) = double_param_array[i * uni_matrix.cols + j];
            }
        }
        cout << endl << camera_name << "--Uni matrix load done!" << endl << uni_matrix << endl;

        camera_params_load_done_ = true;
    }

    /**
 * 将点云合并成矩阵，方便一同运算
 * @param input 输入点云
 * @return 输出矩阵
 */
    Mat DepthSensor::Cloud2Mat(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input) {
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
    void DepthSensor::MatProject(cv::Mat &input_depth, cv::Mat &input_uv, cv::Mat &Cam_matrix, cv::Mat &Uni_matrix) {
        Mat res = Cam_matrix * Uni_matrix * input_uv;
        for (int i = 0; i < res.cols; i++) {
            int x = round(res.at<double>(0, i) / res.at<double>(2, i));
            int y = round(res.at<double>(1, i) / res.at<double>(2, i));
            if (x >= 0 && x < imgCols && y >= 0 && y < imgRows) {
                input_depth.at<double>(y, x) = res.at<double>(2, i);
            }
        }
    }


/**
 * 从深度图中获取ROI的深度
 * @param rect ROI
 * @param depth_list 深度图队列
 * @param id 车辆ID
 * @return 深度值
 */
    float DepthSensor::GetDepthInRect(Rect rect, std::deque<Mat> &depth_list, radar_interfaces::msg::YoloPoint::_id_type id) {
        vector<double> distances;
        //从新到旧遍历深度图队列，直到ROI深度不为0
        for (int i = rect.y; i < (rect.y + rect.height); i++) {
            for (int j = rect.x; j < (rect.x + rect.width); j++) {
                if (depth_list[depth_list.size() - 1].at<double>(i, j) > 0) {
                    distances.push_back(depth_list[depth_list.size() - 1].at<double>(i, j));
                    continue;
                } else {
                    for (uint8_t k = 0; k < depth_list.size(); k++) {
                        if (k < depth_list.size() - 1 && depth_list[k + 1].at<double>(i, j) == 0 &&
                            depth_list[k].at<double>(i, j) > 0) {
                            distances.push_back(depth_list[k].at<double>(i, j));
                            break;
                        }
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
            sort(distances.begin(), distances.end());
            return distances[distances.size() / 2];
//            if (id != 12 && id != 13) {
//                for (float distance: distances) {
//                    sum += distance;
//                }
//                mean_distance = sum / distances.size();
//                return mean_distance;
//            } else {
//                sort(distances.begin(), distances.end());
//                return distances[distances.size() / 2];
////            if (distances.size() >= 5){
////                for(uint8_t j=0;j<5;j++){
////                    sum+=distances[j];
////                }
////                mean_distance=sum/5;
//                //return mean_distance;
////                return distances[distances.size()/2];
////            }
////            else {
////                return distances[0];
////                return distances[distances.size()/2];
////            }
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


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(get_depth::DepthSensor);