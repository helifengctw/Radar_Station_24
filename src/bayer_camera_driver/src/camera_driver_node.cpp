#include "bayer_camera_driver/hikvision_ros2_driver.hpp"


int main(int argc, char** argv) {
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::spin(std::make_shared<HikvisionDriver>(options));
    rclcpp::shutdown();
    return 0;
}