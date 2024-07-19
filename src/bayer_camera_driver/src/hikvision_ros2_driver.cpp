#include "bayer_camera_driver/hikvision_ros2_driver.hpp"

namespace bayer_camera_driver {
#define MV_CHECK(logger, func, ...)                                            \
    do {                                                                       \
        int nRet = func(__VA_ARGS__);                                          \
        if (MV_OK != nRet) {                                                   \
            RCLCPP_ERROR(logger, "hikvision sdk error: " #func " = %d", nRet); \
        }                                                                      \
    } while (0)


// 相机驱动的实现
    struct HikvisionDriver::Imple {
        std::unique_ptr<rclcpp::Logger> logger;

        void *handle;
        image_transport::Publisher img_pub;
        std::shared_ptr<rclcpp::Publisher<radar_interfaces::msg::CameraInfo>> p_info_pub;

        static void image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);

        std::string camera_name = "camera_close";
        int camera_height = 1200;
        int camera_width = 1920;
        float camera_exp = 3500.0;
        float camera_gain = 22.1;
        int camera_auto_exp = 0;
        float camera_auto_maxexp = 4500;
        float camera_auto_minexp = 100;
        int camera_auto_gain = 0;
        float camera_auto_maxgain = 26.0;
        float camera_auto_mingain = 2.0;
        int camera_auto_whitebalance = 0;
    };

    void
    HikvisionDriver::Imple::image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
        auto node = reinterpret_cast<HikvisionDriver *>(pUser);

        uint64_t dev_stamp =
                (uint64_t) pFrameInfo->nDevTimeStampHigh << 32ull | (uint64_t) pFrameInfo->nDevTimeStampLow;
        uint64_t host_stamp = pFrameInfo->nHostTimeStamp;
        auto p_img_msg = std::make_unique<sensor_msgs::msg::Image>();
        if (pFrameInfo->nFrameLen > p_img_msg->data.max_size()) {
            RCLCPP_ERROR_ONCE(node->get_logger(), "image bytes exceed max available size");
            return;
        }

        p_img_msg->header.frame_id = node->p_Implement->camera_name;
        p_img_msg->header.stamp.nanosec = host_stamp % 1000ull * 1000000ull;
        p_img_msg->header.stamp.sec = host_stamp / 1000ull;
        p_img_msg->is_bigendian = false;
        p_img_msg->width = pFrameInfo->nWidth;
        p_img_msg->height = pFrameInfo->nHeight;
        if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerRG8) {
            p_img_msg->step = pFrameInfo->nWidth * 1;
            p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
        } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerBG8) {
            p_img_msg->step = pFrameInfo->nWidth * 1;
            p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerGR8) {
            p_img_msg->step = pFrameInfo->nWidth * 1;
            p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
        } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerGB8) {
            p_img_msg->step = pFrameInfo->nWidth * 1;
            p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
        } else if (pFrameInfo->enPixelType == PixelType_Gvsp_Mono8) {
            p_img_msg->step = pFrameInfo->nWidth * 1;
            p_img_msg->encoding = sensor_msgs::image_encodings::MONO8;
        } else if (pFrameInfo->enPixelType == PixelType_Gvsp_RGB8_Packed) {
            p_img_msg->step = pFrameInfo->nWidth * 3;
            p_img_msg->encoding = sensor_msgs::image_encodings::RGB8;
        } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BGR8_Packed) {
            p_img_msg->step = pFrameInfo->nWidth * 3;
            p_img_msg->encoding = sensor_msgs::image_encodings::BGR8;
        } else {
            RCLCPP_ERROR_ONCE(node->get_logger(), "unsupport pixel format: %d", (int) pFrameInfo->enPixelType);
            return;
        }
        p_img_msg->data.resize(p_img_msg->height * p_img_msg->step);
        std::copy_n(pData, pFrameInfo->nFrameLen, p_img_msg->data.begin());
        node->p_Implement->img_pub.publish(std::move(p_img_msg));

        auto p_info_msg = std::make_unique<radar_interfaces::msg::CameraInfo>();
        MVCC_FLOATVALUE stExposureTime = {0}, stGain = {0};
        MV_CC_GetFloatValue(node->p_Implement->handle, "ExposureTime", &stExposureTime);
        MV_CC_GetFloatValue(node->p_Implement->handle, "Gain", &stGain);
        p_info_msg->header.frame_id = node->p_Implement->camera_name;
        p_info_msg->header.stamp.nanosec = host_stamp % 1000ull * 1000000ull;
        p_info_msg->header.stamp.sec = host_stamp / 1000ull;
        p_info_msg->dev_stamp.nanosec = dev_stamp % 1000000000ull;
        p_info_msg->dev_stamp.sec = dev_stamp / 1000000000ull;
        p_info_msg->frame_num = pFrameInfo->nFrameNum;
        p_info_msg->gain = stGain.fCurValue;
        p_info_msg->exposure_time = stExposureTime.fCurValue;
        p_info_msg->width = pFrameInfo->nWidth;
        p_info_msg->height = pFrameInfo->nHeight;
        node->p_Implement->p_info_pub->publish(std::move(p_info_msg));
    }

    HikvisionDriver::HikvisionDriver(const rclcpp::NodeOptions &options)
            : rclcpp::Node("hikvision_driver_node", options), p_Implement(std::make_unique<Imple>()) {
        auto logger = get_logger();
        p_Implement->logger = std::make_unique<rclcpp::Logger>(logger);

        declare_and_get_params();
        p_Implement->camera_name = this->get_parameter("camera_name").as_string();
        RCLCPP_INFO(logger, "trying to open camera: '%s'", p_Implement->camera_name.c_str());

        rclcpp::QoS qos(1);
        p_Implement->img_pub = image_transport::create_publisher(this, "raw/image", qos.get_rmw_qos_profile());
        p_Implement->p_info_pub = create_publisher<radar_interfaces::msg::CameraInfo>("raw/camera_info", qos);

        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        MV_CHECK(logger, MV_CC_EnumDevices, MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);

        for (uint32_t nDeviceId = 0; nDeviceId < stDeviceList.nDeviceNum; nDeviceId++) {
            auto *pDeviceInfo = stDeviceList.pDeviceInfo[nDeviceId];
            const char *pUserDefinedName = nullptr;
            if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
                pUserDefinedName = (const char *) pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
                if (pUserDefinedName == p_Implement->camera_name) {
                    int nIp1 = (int) ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                    int nIp2 = (int) ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                    int nIp3 = (int) ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                    int nIp4 = (int) (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
                    RCLCPP_INFO(logger, "[%s]: GIGE, %s, %d.%d.%d.%d", pUserDefinedName,
                                pDeviceInfo->SpecialInfo.stGigEInfo.chModelName, nIp1, nIp2, nIp3, nIp4);
                }
            } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
                pUserDefinedName = (const char *) pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
                if (pUserDefinedName == p_Implement->camera_name) {
                    RCLCPP_INFO(logger, "[%s]: USB, %s", pUserDefinedName,
                                pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
                }
            } else {
                RCLCPP_WARN(logger, "type(%d) not support", pDeviceInfo->nTLayerType);
            }
            if (pUserDefinedName == p_Implement->camera_name) {
                MV_CHECK(logger, MV_CC_CreateHandle, &p_Implement->handle, pDeviceInfo);
                MV_CHECK(logger, MV_CC_OpenDevice, p_Implement->handle);
                set_exp();
                set_gain();
                set_whiteBalance();
                MV_CHECK(logger, MV_CC_RegisterImageCallBackEx, p_Implement->handle,
                         &HikvisionDriver::Imple::image_callback_ex,
                         this); // 在这里注册用相机流的回调函数
                MV_CHECK(logger, MV_CC_StartGrabbing, p_Implement->handle);
//            MV_CHECK(logger, MV_CC_SetBayerCvtQuality, p_Implement->handle, 2); // 设置bayer格式的插值方式
                break;
            }
        }
        if (p_Implement->handle == nullptr) {
            RCLCPP_ERROR(logger, "camera-'%s' not found", p_Implement->camera_name.c_str());
        }
    }

    HikvisionDriver::~HikvisionDriver() {
        if (p_Implement->handle == nullptr) return;
        auto logger = get_logger();

        MV_CHECK(logger, MV_CC_StopGrabbing, p_Implement->handle);
        MV_CHECK(logger, MV_CC_CloseDevice, p_Implement->handle);
        MV_CHECK(logger, MV_CC_DestroyHandle, p_Implement->handle);
        p_Implement->handle = nullptr;
    }

    void HikvisionDriver::declare_and_get_params() {
        this->declare_parameter<uint16_t>("camera_height", 540);
        this->get_parameter("camera_height", this->p_Implement->camera_height);
        this->declare_parameter<uint16_t>("camera_width", 720);
        this->get_parameter("camera_width", this->p_Implement->camera_width);
        this->declare_parameter<float_t>("camera_exp", 4000.0);
        this->get_parameter("camera_exp", this->p_Implement->camera_exp);
        this->declare_parameter<float_t>("camera_gain", 12.0);
        this->get_parameter("camera_gain", this->p_Implement->camera_gain);
        this->declare_parameter<std::string>("camera_name", "camera_init");
        this->get_parameter("camera_name", this->p_Implement->camera_name);
        this->declare_parameter<uint16_t>("camera_auto_exp", 0);
        this->get_parameter("camera_auto_exp", this->p_Implement->camera_auto_exp);
        this->declare_parameter<uint16_t>("camera_auto_gain", 0);
        this->get_parameter("camera_auto_gain", this->p_Implement->camera_auto_gain);
        this->declare_parameter<uint16_t>("camera_auto_whitebalance", 0);
        this->get_parameter("camera_auto_whitebalance", this->p_Implement->camera_auto_whitebalance);
        this->declare_parameter<float_t>("camera_auto_maxexp", 4500.0);
        this->get_parameter("camera_auto_maxexp", this->p_Implement->camera_auto_maxexp);
        this->declare_parameter<float_t>("camera_auto_minexp", 100.0);
        this->get_parameter("camera_auto_minexp", this->p_Implement->camera_auto_minexp);
        this->declare_parameter<float_t>("camera_auto_maxgain", 17.0);
        this->get_parameter("camera_auto_maxgain", this->p_Implement->camera_auto_maxgain);
        this->declare_parameter<float_t>("camera_auto_mingain", 0.0);
        this->get_parameter("camera_auto_mingain", this->p_Implement->camera_auto_mingain);
    }

    void HikvisionDriver::set_exp() {
        MVCC_FLOATVALUE device_ExposureTime = {0};
        MV_CHECK(this->get_logger(), MV_CC_GetFloatValue, this->p_Implement->handle, "ExposureTime",
                 &device_ExposureTime);
        RCLCPP_INFO(this->get_logger(), "(%s) device original {exposure time} value: [%f]",
                    this->p_Implement->camera_name.c_str(), device_ExposureTime.fCurValue);

        MV_CHECK(this->get_logger(), MV_CC_SetFloatValue, this->p_Implement->handle, "ExposureTime",
                 this->p_Implement->camera_exp);
        RCLCPP_INFO(this->get_logger(), "(%s) target {exposure time} value : [%f]",
                    this->p_Implement->camera_name.c_str(), this->p_Implement->camera_exp);

        MV_CHECK(this->get_logger(), MV_CC_GetFloatValue, this->p_Implement->handle, "ExposureTime",
                 &device_ExposureTime);
        RCLCPP_INFO(this->get_logger(), "set (%s) device {exposure time} value set to: [%f]\n",
                    this->p_Implement->camera_name.c_str(), device_ExposureTime.fCurValue);
    }

    void HikvisionDriver::set_gain() {
        MVCC_FLOATVALUE device_Gain = {0};
        MV_CHECK(this->get_logger(), MV_CC_GetFloatValue, this->p_Implement->handle, "Gain", &device_Gain);
        RCLCPP_INFO(this->get_logger(), "(%s) device original {gain} value: [%f]",
                    this->p_Implement->camera_name.c_str(), device_Gain.fCurValue);

        MV_CHECK(this->get_logger(), MV_CC_SetFloatValue, this->p_Implement->handle, "Gain",
                 this->p_Implement->camera_gain);
        RCLCPP_INFO(this->get_logger(), "(%s) target {gain} value: [%f]", this->p_Implement->camera_name.c_str(),
                    this->p_Implement->camera_gain);

        MV_CHECK(this->get_logger(), MV_CC_GetFloatValue, this->p_Implement->handle, "Gain", &device_Gain);
        RCLCPP_INFO(this->get_logger(), "set (%s) device {gain} value: [%f]\n", this->p_Implement->camera_name.c_str(),
                    device_Gain.fCurValue);
    }

    void HikvisionDriver::set_whiteBalance() {
        MVCC_ENUMVALUE device_AutoWhiteBalance = {0};
        MV_CHECK(this->get_logger(), MV_CC_GetEnumValue, this->p_Implement->handle, "BalanceWhiteAuto",
                 &device_AutoWhiteBalance);
        RCLCPP_INFO(this->get_logger(), "(%s) device original auto white balance value: [%d]",
                    this->p_Implement->camera_name.c_str(), device_AutoWhiteBalance.nCurValue);

        MV_CHECK(this->get_logger(), MV_CC_SetEnumValue, this->p_Implement->handle, "BalanceWhiteAuto",
                 this->p_Implement->camera_auto_whitebalance);
        RCLCPP_INFO(this->get_logger(), "(%s) target value for auto white balance value to set: [%d]",
                    this->p_Implement->camera_name.c_str(), this->p_Implement->camera_auto_whitebalance);

        MV_CHECK(this->get_logger(), MV_CC_GetEnumValue, this->p_Implement->handle, "BalanceWhiteAuto",
                 &device_AutoWhiteBalance);
        RCLCPP_INFO(this->get_logger(), "set (%s) device auto white balance value: [%d]\n",
                    this->p_Implement->camera_name.c_str(), device_AutoWhiteBalance.nCurValue);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bayer_camera_driver::HikvisionDriver);