#ifndef HIKVISION_ROS2_DRIVER_HPP
#define HIKVISION_ROS2_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>

// ros2
#include <radar_interfaces/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>

// hikvision sdk
#include <MvCameraControl.h>

namespace bayer_camera_driver {
    class HikvisionDriver : public rclcpp::Node {
    public:
        HikvisionDriver(const rclcpp::NodeOptions &options);
        ~HikvisionDriver();
        void declare_and_get_params();
        void set_exp();
        void set_gain();
        void set_whiteBalance();
//        bool AutoGain_Operation();

    private:
        struct Imple;
        std::unique_ptr<Imple> p_Implement;
    };
}


#endif  // HIKVISION_ROS2_DRIVER_HPP