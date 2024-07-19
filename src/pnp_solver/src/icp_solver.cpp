#include <cstdio>
#include <opencv2/opencv.hpp>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "radar_interfaces/msg/points.hpp"
#include "radar_interfaces/msg/dist_points.hpp"
#include "radar_interfaces/msg/pnp_result.hpp"
#include "radar_interfaces/msg/battle_color.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.14/pcl/registration/ndt.h>
#include <pcl-1.14/pcl/registration/gicp.h>
#include <boost/shared_ptr.hpp>


using std::placeholders::_1;


Eigen::Matrix3f far_R = Eigen::MatrixXf::Identity(3, 3);
Eigen::Matrix3f close_R = Eigen::MatrixXf::Identity(3, 3);
Eigen::Vector4f far_T = Eigen::VectorXf::Ones(4), close_T = Eigen::VectorXf::Ones(4);

Eigen::Matrix4f far_uni_matrix = Eigen::MatrixXf::Identity(4, 4);
Eigen::Matrix4f close_uni_matrix = Eigen::MatrixXf::Identity(4, 4);
Eigen::Matrix4f far_Pnp_matrix = Eigen::MatrixXf::Zero(4, 4);
Eigen::Matrix4f close_Pnp_matrix = Eigen::MatrixXf::Zero(4, 4);

Eigen::Matrix4f init_guess = Eigen::MatrixXf::Identity(4, 4);
Eigen::Matrix4f Z_roll_90 = Eigen::MatrixXf::Zero(4, 4);


radar_interfaces::msg::PnpResult icp_result_msg;


class IcpSolver : public rclcpp::Node {
public:
    IcpSolver(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Icp solver node initialize!!!");

        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", 10, std::bind(&IcpSolver::point_cloudCallback, this, std::placeholders::_1));
        pnp_result_subscription_ = this->create_subscription<radar_interfaces::msg::PnpResult>(
                "pnp_result", 10, std::bind(&IcpSolver::pnp_resultCallback, this, _1));

        icp_result_publisher_ = this->create_publisher<radar_interfaces::msg::PnpResult>("/icp_result", 1);
        rviz_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_point_cloud", 10);
        rviz_target_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_point_cloud", 10);


        while (!parameter_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
//            return -1;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the parameter service to start...");
        }

        load_uni_matrix();
        initializeRegistration();
        map_loader("/home/hlf/Desktop/radar24_ws/src/pnp_solver/data/map.pcd");
        load_ndt_params();
    }

private:
    rclcpp::Subscription<radar_interfaces::msg::PnpResult>::SharedPtr pnp_result_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;

    rclcpp::Publisher<radar_interfaces::msg::PnpResult>::SharedPtr icp_result_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rviz_map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rviz_target_publisher_;

    void initializeRegistration();
    void map_loader(const std::string&);
    void pnp_resultCallback(radar_interfaces::msg::PnpResult::ConstSharedPtr);
    void point_cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr);
    void load_uni_matrix();
    void load_ndt_params();

    std::shared_ptr<rclcpp::SyncParametersClient> parameter_client = std::make_shared<rclcpp::SyncParametersClient>(
            this, "pnp_solver");
    boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;

    // TODO config at here
    double ndt_resolution_ = 1.0;
    double ndt_step_size_ = 0.5;
    double transform_epsilon_ = 0.01;
    bool map_recieved_ {false}, initialpose_recieved_ {false}, enable_debug_{false};
    double score_threshold_ = 2.0;
};

void write_icp_result_msg(Eigen::Matrix4f);

radar_interfaces::msg::PnpResult Icp_result_msg;


int main(int argc, char **argv) {
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto IS_node = std::make_shared<IcpSolver>("icp_solver");

    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok()) {
        rclcpp::spin_some(IS_node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

void IcpSolver::load_ndt_params() {
    this->declare_parameter<double>("resolution", 1.0);
    this->declare_parameter<double>("step_size", 0.5);
    this->declare_parameter<double>("transform_epsilon", 0.01);
    this->declare_parameter<double>("threshold", 2.0);
    this->declare_parameter<int>("enable_debug", 0);

    ndt_resolution_ = this->get_parameter("resolution").as_double();
    ndt_step_size_ = this->get_parameter("step_size").as_double();
    transform_epsilon_ = this->get_parameter("transform_epsilon").as_double();
    score_threshold_ = this->get_parameter("threshold").as_double();
    enable_debug_ = (bool)this->get_parameter("enable_debug").as_int();
}

void IcpSolver::load_uni_matrix() {
    std::vector<rclcpp::Parameter> param_vector;
    std::vector<double> temp_array;
    // 相机外参默认值
    param_vector = parameter_client->get_parameters({"sensor_far.uni_matrix"});
    temp_array = param_vector[0].as_double_array();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            far_uni_matrix(i, j) = temp_array[i * 4 + j];
        }
    }

    param_vector = parameter_client->get_parameters({"sensor_close.uni_matrix"});
    temp_array = param_vector[0].as_double_array();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            close_uni_matrix(i, j) = temp_array[i * 4 + j];
        }
    }

    std::cout << std::endl << "far Uni matrix load done!" << std::endl << far_uni_matrix << std::endl;
    std::cout << std::endl << "close Uni matrix load done!" << std::endl << close_uni_matrix << std::endl;
}

void IcpSolver::initializeRegistration() {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
            new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
}

void IcpSolver::map_loader(const std::string& file_path) {
    RCLCPP_INFO(get_logger(), "mapReceived");
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    //从磁盘上加载PointCloud数据到二进制存储块中，打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *map_cloud_ptr) == -1){
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return;
    }

    //  将source_pcl 从雷达座标系绕Z轴顺时针旋转90度到世界座标系，以和pnp的座标系保持一致
    Eigen::Affine3f rotate_z = Eigen::Affine3f::Identity();
    rotate_z.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*map_cloud_ptr, *rotatedCloud, rotate_z.matrix());

    registration_->setInputTarget(rotatedCloud);
//    registration_->setInputTarget(map_cloud_ptr);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*rotatedCloud, cloud_msg);
    cloud_msg.header.frame_id = "livox_frame";
    cloud_msg.header.stamp = rclcpp::Clock().now();
    this->rviz_map_publisher_->publish(cloud_msg);

    map_recieved_ = true;
    std::cout << "map size: " << map_cloud_ptr->size() << std::endl;
    RCLCPP_INFO(get_logger(), "mapReceived end");
}

void IcpSolver::pnp_resultCallback(radar_interfaces::msg::PnpResult::ConstSharedPtr msg) {
    // load pnp result matrix
    for (int i = 0; i < 3; i++) {
        far_T[i] = msg->far_t[i];
        close_T[i] = msg->close_t[i];
        for (int j = 0; j < 3; j++) {
            far_R(i, j) = msg->far_r[3*i + j];
            close_R(i, j) = msg->close_r[3*i + j];
        }
    }
    far_Pnp_matrix.block(0, 0, 3, 3) << far_R;
    far_Pnp_matrix.col(3) << far_T;
    close_Pnp_matrix.block(0, 0, 3, 3) << close_R;
    close_Pnp_matrix.col(3) << close_T;

    Eigen::Matrix4f far_init = far_Pnp_matrix.inverse() * far_uni_matrix,
            close_init = close_Pnp_matrix.inverse() * close_uni_matrix;
//    init_guess = (far_init + close_init) / 2;
    initialpose_recieved_ = true;

    // print result
//    std::cout << std::endl << "get far_pnp_result: " << std::endl << far_Pnp_matrix << std::endl;
//    std::cout << std::endl << "get close_pnp_result" << std::endl << close_Pnp_matrix << std::endl;
    std::cout << std::endl << "far_temp_init:" << std::endl << far_init << std::endl;
    std::cout << std::endl << "close_temp_init:" << std::endl << close_init << std::endl;
    std::cout << std::endl << "init_guess:" << std::endl << init_guess << std::endl;
}

void IcpSolver::point_cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    char input_signal = '?';
    std::cout << "输入a进行对齐，输入q退出" << std::endl;
    std::cin >> input_signal;
    if (input_signal == 'a') {
        std::cout << "开始一次对齐" << std::endl;
    }
    else if (input_signal == 'q') {
        std::cout << "退出此次对齐" << std::endl;
        return;
    }

    // receive point cloud data from msg, and input this to registration_
    if (!map_recieved_ || !initialpose_recieved_) {
        RCLCPP_INFO(get_logger(), "didn't received map pcl or initial pose value!!!");
        return;
    }
    RCLCPP_INFO(get_logger(), "pcl Received, aligning......");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_ptr);

//    //  将雷达座标系绕Z轴顺时针旋转90度到世界座标系
//    pcl::PointCloud<pcl::PointXYZI>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZI>());
//    pcl::transformPointCloud(*cloud_ptr, *rotatedCloud, Z_roll_90);

    // set input source
    registration_->setInputSource(cloud_ptr);
    RCLCPP_INFO(get_logger(), "pcl size: %d", cloud_ptr->size());

    // align received point cloud data to map
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    rclcpp::Clock system_clock;
    rclcpp::Time time_align_start = system_clock.now();

    registration_->align(*output_cloud, init_guess);
    rclcpp::Time time_align_end = system_clock.now();

    // evaluate the align result and publish it
    bool has_converged = registration_->hasConverged();
    double fitness_score = registration_->getFitnessScore();
    if (!has_converged) {
        RCLCPP_WARN(get_logger(), "The registration didn't converge.");
        return;
    }
    if (fitness_score > score_threshold_) {
        RCLCPP_WARN(get_logger(), "Seem to converge, the fitness score is over %lf.", score_threshold_);
    }
    Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
    init_guess = final_transformation;
//    Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
    write_icp_result_msg(final_transformation);
    this->icp_result_publisher_->publish(icp_result_msg);

    // visualize the result
    sensor_msgs::msg::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*output_cloud, output_cloud_msg);
    output_cloud_msg.header.frame_id = "livox_frame";
    output_cloud_msg.header.stamp = rclcpp::Clock().now();
    this->rviz_target_publisher_->publish(output_cloud_msg);

    if (enable_debug_) {
//        std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
        std::cout << "output pcl size: " << output_cloud->size() << std::endl;
        std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() <<
                  "[sec]" << std::endl;
        std::cout << "has converged(1-Y, 0-X): " << has_converged << std::endl;
        std::cout << "fitness score: " << fitness_score << std::endl;
        std::cout << "final transformation:" << std::endl << final_transformation << std::endl;
        /* delta_angle check
         * trace(RotationMatrix) = 2(cos(theta) + 1)
         */
        double init_cos_angle = 0.5 *
                                (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
        double cos_angle = 0.5 *
                           (final_transformation.coeff(0,
                                                       0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
        double init_angle = acos(init_cos_angle);
        double angle = acos(cos_angle);
        // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
        double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
        std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
        std::cout << "-----------------------------------------------------" << std::endl;
    }
}

void write_icp_result_msg(Eigen::Matrix4f result_matrix) {
    Eigen::Matrix4f far_pnp, close_pnp;
    far_pnp = (result_matrix.inverse() * far_uni_matrix).inverse();
    close_pnp = (result_matrix.inverse() * close_uni_matrix).inverse();
    for (int i = 0; i < 3; i++) {
        Icp_result_msg.far_t[i] = far_pnp(3, i);
        Icp_result_msg.close_t[i] = close_pnp(3, i);
        for (int j = 0; j < 3; j++) {
            Icp_result_msg.far_r[3*i + j] = far_pnp(i, j);
            Icp_result_msg.close_r[3*i + j] = close_pnp(i, j);
        }
    }
}