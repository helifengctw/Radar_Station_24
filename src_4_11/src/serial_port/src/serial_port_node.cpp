#include <cstdio>
#include <opencv2/opencv.hpp>
#include <memory>
#include <serial/serial.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "CRC8_CRC16.h"
#include "CRC.h"
#include "radar_interfaces/msg/point.hpp"
#include "radar_interfaces/msg/points.hpp"
#include "radar_interfaces/msg/game_state.hpp"
#include "radar_interfaces/msg/supply_projectile_action.hpp"
#include "radar_interfaces/msg/referee_warning.hpp"
#include "chrono"
#include "serial_port.h"

using namespace cv;
using std::placeholders::_1;
using namespace std::chrono_literals;


uint8_t warn_state;

/**
 * 串口通讯类
 */
class SerialPort : public rclcpp::Node {
public:
    int serial_port_init();

    SerialPort(std::string name) : Node (name){
        RCLCPP_INFO(this->get_logger(), "Successfully initialize serial port node :serial_port !!!");
        if (serial_port_init() <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port, please check the hardware !!!");
        } else {
            RCLCPP_INFO(this->get_logger(), "successfully initialize serial port !!!");
        }
        game_state_publisher_ = this->create_publisher<radar_interfaces::msg::GameState>(
                "game_state", 1);
        supply_projectile_action_publisher_ = this->create_publisher<radar_interfaces::msg::SupplyProjectileAction>(
                "supply_projectile_action", 1);
        referee_warning_publisher_ = this->create_publisher<radar_interfaces::msg::RefereeWarning>(
                "referee_warning", 1);

        world_point_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
                "/world_point", 1, std::bind(&SerialPort::worldPointsCallback, this, _1));
        guard_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
                "/guard_pub", 1, std::bind(&SerialPort::GuardCallback, this, _1));
//        hero_subscription_ = this->create_subscription<radar_interfaces::msg::Point>(
//                "/hero_pub", 1, std::bind(&SerialPort::HeroCallback, this, _1));

//        timer_ = this->create_wall_timer(
//                500ms, std::bind(&SerialPort::timerCallback, this));
    }

    serial::Serial ser;
    map_msg mapMsg;
    robot_interactive_msgs robotInteractiveMsgs;
    robot_interactive_msgs HeroMsgs;
    robot_interactive_control_msgs robotInteractiveControlMsgs;
    robot_health_msgs robotHealthMsgs;
    game_result_msg gameResultMsg;
    site_event_msgs siteEventMsgs;
    supply_projectile_action_msg supplyProjectileActionMsg;
    referee_warning_msg refereeWarningMsg;
    dart_remaining_time_msg dartRemainingTimeMsg;
    game_status_msgs gameStatusMsgs;
//    ros::Publisher gameStatePub;
//    ros::Publisher supplyProjectileActionPub;
//    ros::Publisher refereeWarningPub;
    radar_interfaces::msg::GameState gameStateRosMsg;
    radar_interfaces::msg::SupplyProjectileAction supplyProjectileActionRosMsg;
    radar_interfaces::msg::RefereeWarning refereeWarningRosMsg;
    uint8_t receiveData[1024];
    bool is_enemy_red = false;

    bool sendMapMsgs(uint16_t, float, float);
    bool sendInteractiveMsgs(uint16_t);
    bool sendHeroMsgs();
    bool receiveMsgs();
    void LoadBattleColor();
//    void timerCallback();
    

private:
    void worldPointsCallback(const radar_interfaces::msg::Points::SharedPtr) const;
    void GuardCallback(const radar_interfaces::msg::Points::SharedPtr);

    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr world_point_subscription_;
    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr guard_subscription_;

    rclcpp::Publisher<radar_interfaces::msg::GameState>::SharedPtr game_state_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::SupplyProjectileAction>::SharedPtr supply_projectile_action_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::RefereeWarning>::SharedPtr referee_warning_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

//SerialPort sp;
std::vector<car_point> worldPoints;


int main(int argc, char **argv) {
    //初始化节点
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);

    auto sp_node = std::make_shared<SerialPort>("serial_port");

    sp_node->LoadBattleColor();

    rclcpp::Rate loop(100);
    int count = 0;
    RCLCPP_INFO(sp_node->get_logger(), "Looping!/n");
    while (rclcpp::ok()) {
        count++;
        //分频为10hz，实现10hz定频发送
        if (count < 10) continue;
        sp_node->sendInteractiveMsgs(7);
        sp_node->sendHeroMsgs();
        //逐一发送小地图目标点，当等待发布的点为空时接收一次新的消息
        if (!worldPoints.empty()) {
            if (worldPoints[0].color) {
                sp_node->sendMapMsgs(100 + worldPoints[0].id, worldPoints[0].point.x, worldPoints[0].point.y);
            } else {
                sp_node->sendMapMsgs(worldPoints[0].id, worldPoints[0].point.x, worldPoints[0].point.y);
            }
            worldPoints.erase(worldPoints.begin());
        } else {
            rclcpp::spin_some(sp_node);
//                for (int i = 0; i < 10; i++) {
//                    car_point carPoint;
//                    carPoint.point = Point2f(1.4 * i, 2.8 * i);
//                    carPoint.id = 6;
//                    if (i < 5) {
//                        carPoint.color = true;
//                    } else {
//                        carPoint.color = false;
//                    }
//                    worldPoints.push_back(carPoint);
//                }
            //测试用
        }
        count = 0;
        sp_node->receiveMsgs();
        rclcpp::spin_some(sp_node);
//        ros::spinOnce();
        //循环休眠
        loop.sleep();
    }
    sp_node->ser.close(); // 关闭串口
//    rclcpp::shutdown();
    return 0;
}


//void SerialPort::timerCallback() {
//
//}


/**
 * 读取我方阵营参数
 */
void SerialPort::LoadBattleColor() {
    std::string exchange = this->get_parameter("battle_state/battle_color").as_string();
    if (exchange == "red") {
        this->is_enemy_red = false;
    } else {
        this->is_enemy_red = true;
    }
}

/**
 * 串口初始化
 * @return
 */
int SerialPort::serial_port_init() {
    //串口初始化
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        //串口设置
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);

        ser.setStopbits(serial::stopbits_t::stopbits_one);
        ser.setBytesize(serial::bytesize_t::eightbits);
        ser.setParity(serial::parity_t::parity_none);       //设置校验位

        ser.open(); //打开
    }
    catch(serial::IOException &e) {
        std::cout<<"unable to open ser"<<std::endl;
        return -1;
    }
    catch(std::invalid_argument &e) {
        std::cout<<"std::invalid_argument"<<std::endl;
        return -1;
    }
    catch(serial::SerialException &e) {
        std::cout<<"serial::SerialException"<<std::endl;
        return -1;
    }
    //检查串口
    if(ser.isOpen()) {
        std::cout<<"ser open"<<std::endl;
    } else {
        return -1;
    }

    return 0;
}

/**
 * 发送小地图消息
 * @param id 车辆ID
 * @param x x坐标
 * @param y y坐标
 * @return 是否成功发送
 */
bool SerialPort::sendMapMsgs(uint16_t id, float x, float y) {
    mapMsg.head.SOF = 0xA5;
    mapMsg.head.data_length = 10;
    mapMsg.head.seq = 1;
    mapMsg.head.crc = get_CRC8_check_sum((uint8_t *) &mapMsg, (sizeof(mapMsg.head) - sizeof(mapMsg.head.crc)),
                                         0xff);
    mapMsg.cmd_id = 0x0305;
    mapMsg.data.target_position_x = x;
    mapMsg.data.target_position_y = y;
    mapMsg.data.target_robot_id = id;
    mapMsg.crc = get_CRC16_check_sum((uint8_t *) &mapMsg, (sizeof(mapMsg) - sizeof(mapMsg.crc)), 0xffff);
    ser.write((uint8_t *) &mapMsg, sizeof(map_msg));
    std::cout << "Send one map msg target_id = " << mapMsg.data.target_robot_id << " x = "
              << mapMsg.data.target_position_x << " y = " << mapMsg.data.target_position_y << std::endl;
    return true;
}

/**
 * 发送车间交互消息
 * @param receiver_id 在当接受者为红方时的ID
 * @return 是否成功发送
 */
bool SerialPort::sendInteractiveMsgs(uint16_t receiver_id)//接受者ID以红方为准
{
    //构造头
    robotInteractiveMsgs.head.SOF = 0xA5;
    robotInteractiveMsgs.head.data_length = sizeof(robot_interactive_data);
    robotInteractiveMsgs.head.seq = 1;
    robotInteractiveMsgs.head.crc = get_CRC8_check_sum((uint8_t *) &robotInteractiveMsgs,
                                                       (sizeof(robotInteractiveMsgs.head) -
                                                        sizeof(robotInteractiveMsgs.head.crc)), 0xff);
    robotInteractiveMsgs.cmd_id = 0x0301;
    robotInteractiveMsgs.data.cmd_id = 0x0201;
    //根据阵营自动调整ID
    if (is_enemy_red) {
        robotInteractiveMsgs.data.sender_id = 109;
        robotInteractiveMsgs.data.receiver_id = 100 + receiver_id;
    } else {
        robotInteractiveMsgs.data.sender_id = 9;
        robotInteractiveMsgs.data.receiver_id = receiver_id;
    }
    std::cout << robotInteractiveMsgs.data.sender_id << robotInteractiveMsgs.data.receiver_id << std::endl;
    robotInteractiveMsgs.crc = get_CRC16_check_sum((uint8_t *) &robotInteractiveMsgs,
                                                   (sizeof(robotInteractiveMsgs) -
                                                    sizeof(robotInteractiveMsgs.crc)), 0xffff);
    ser.write((uint8_t *) &robotInteractiveMsgs, sizeof(robotInteractiveMsgs));
    std::cout << "Send one interactive msg " << std::endl;
    std::cout << robotInteractiveMsgs.data.receiver_id << std::endl;
    return true;
}

/**
 * 发送与英雄交互的消息
 * @return 是否成功发送
 */
bool SerialPort::sendHeroMsgs(){
    //构造头
    HeroMsgs.head.SOF = 0xA5;
    HeroMsgs.head.data_length = sizeof(robot_interactive_data);
    HeroMsgs.head.seq = 1;
    HeroMsgs.head.crc = get_CRC8_check_sum((uint8_t *) &HeroMsgs,
                                           (sizeof(HeroMsgs.head) -
                                            sizeof(HeroMsgs.head.crc)), 0xff);
    HeroMsgs.cmd_id = 0x0301;
    HeroMsgs.data.cmd_id = 0x0202;
    if (is_enemy_red) {
        HeroMsgs.data.sender_id = 109;
        HeroMsgs.data.receiver_id = 100 + 1;
    } else {
        HeroMsgs.data.sender_id = 9;
        HeroMsgs.data.receiver_id = 1;
    }
    std::cout << HeroMsgs.data.sender_id << HeroMsgs.data.receiver_id << std::endl;
    HeroMsgs.crc = get_CRC16_check_sum((uint8_t *) &HeroMsgs,
                                       (sizeof(HeroMsgs) -
                                        sizeof(HeroMsgs.crc)), 0xffff);
    ser.write((uint8_t *) &HeroMsgs, sizeof(HeroMsgs));
    std::cout << "Send one Hero msg " << std::endl;
    std::cout << HeroMsgs.data.receiver_id << std::endl;
    return true;
}

/**
 * 接收消息
 * @return 是否成功接收
 */
bool SerialPort::receiveMsgs() {
    if (ser.available()) {
        bool if_pub = false;
        ser.read(receiveData, ser.available());
        //使用所有消息类型进行匹配，若CRC校验通过，则匹配成功
        gameStatusMsgs = (*(game_status_msgs *) receiveData);
        dartRemainingTimeMsg = (*(dart_remaining_time_msg *) receiveData);
        robotHealthMsgs = (*(robot_health_msgs *) receiveData);
        gameResultMsg = (*(game_result_msg *) receiveData);
        siteEventMsgs = (*(site_event_msgs *) receiveData);
        supplyProjectileActionMsg = (*(supply_projectile_action_msg *) receiveData);
        refereeWarningMsg = (*(referee_warning_msg *) receiveData);
        dartRemainingTimeMsg = (*(dart_remaining_time_msg *) receiveData);

        gameStateRosMsg.dart_remaining_time = 16;
        gameStateRosMsg.winner = 3;
        if ((gameStatusMsgs.head.crc == get_CRC8_check_sum((uint8_t *) &gameStatusMsgs,
                                                           (sizeof(gameStatusMsgs.head) -
                                                            sizeof(gameStatusMsgs.head.crc)), 0xff)) &&
            (gameStatusMsgs.crc ==
             get_CRC16_check_sum((uint8_t *) &gameStatusMsgs, (sizeof(gameStatusMsgs) - sizeof(gameStatusMsgs.crc)),
                                 0xffff))) {

            gameStateRosMsg.game_progress = gameStatusMsgs.data.game_progress;
            gameStateRosMsg.game_type = gameStatusMsgs.data.game_type;
            gameStateRosMsg.stage_remain_time = gameStatusMsgs.data.stage_remain_time;
            if_pub = true;
        }
        if ((dartRemainingTimeMsg.head.crc == get_CRC8_check_sum((uint8_t *) &dartRemainingTimeMsg,
                                                                 (sizeof(dartRemainingTimeMsg.head) -
                                                                  sizeof(dartRemainingTimeMsg.head.crc)), 0xff)) &&
            (dartRemainingTimeMsg.crc == get_CRC16_check_sum((uint8_t *) &dartRemainingTimeMsg,
                                                             (sizeof(dartRemainingTimeMsg) -
                                                              sizeof(dartRemainingTimeMsg.crc)), 0xffff))) {
            gameStateRosMsg.dart_remaining_time = dartRemainingTimeMsg.data.dart_remaining_time;
            if_pub = true;
        }
        if ((robotHealthMsgs.head.crc == get_CRC8_check_sum((uint8_t *) &robotHealthMsgs,
                                                            (sizeof(robotHealthMsgs.head) -
                                                             sizeof(robotHealthMsgs.head.crc)), 0xff)) &&
            (robotHealthMsgs.crc == get_CRC16_check_sum((uint8_t *) &robotHealthMsgs,
                                                        (sizeof(robotHealthMsgs) - sizeof(robotHealthMsgs.crc)),
                                                        0xffff))) {
            gameStateRosMsg.blue_1_robot_hp = robotHealthMsgs.data.blue_1_robot_hp;
            gameStateRosMsg.blue_2_robot_hp = robotHealthMsgs.data.blue_2_robot_hp;
            gameStateRosMsg.blue_3_robot_hp = robotHealthMsgs.data.blue_3_robot_hp;
            gameStateRosMsg.blue_4_robot_hp = robotHealthMsgs.data.blue_4_robot_hp;
            gameStateRosMsg.blue_5_robot_hp = robotHealthMsgs.data.blue_5_robot_hp;
            gameStateRosMsg.blue_7_robot_hp = robotHealthMsgs.data.blue_7_robot_hp;
            gameStateRosMsg.blue_base_hp = robotHealthMsgs.data.blue_base_hp;
            gameStateRosMsg.blue_outpose_hp = robotHealthMsgs.data.blue_outpose_hp;
            gameStateRosMsg.red_1_robot_hp = robotHealthMsgs.data.red_1_robot_hp;
            gameStateRosMsg.red_2_robot_hp = robotHealthMsgs.data.red_2_robot_hp;
            gameStateRosMsg.red_3_robot_hp = robotHealthMsgs.data.red_3_robot_hp;
            gameStateRosMsg.red_4_robot_hp = robotHealthMsgs.data.red_4_robot_hp;
            gameStateRosMsg.red_5_robot_hp = robotHealthMsgs.data.red_5_robot_hp;
            gameStateRosMsg.red_7_robot_hp = robotHealthMsgs.data.red_7_robot_hp;
            gameStateRosMsg.red_base_hp = robotHealthMsgs.data.red_base_hp;
            gameStateRosMsg.red_outpose_hp = robotHealthMsgs.data.red_outpose_hp;
            if_pub = true;
        }
        if ((gameResultMsg.head.crc == get_CRC8_check_sum((uint8_t *) &gameResultMsg, (sizeof(gameResultMsg.head) -
                                                                                       sizeof(gameResultMsg.head.crc)),
                                                          0xff)) && (gameResultMsg.crc ==
                                                                     get_CRC16_check_sum((uint8_t *) &gameResultMsg,
                                                                                         (sizeof(gameResultMsg) -
                                                                                          sizeof(gameResultMsg.crc)),
                                                                                         0xffff))) {
            gameStateRosMsg.winner = gameResultMsg.data.winner;
            if_pub = true;
        }
        if ((siteEventMsgs.head.crc == get_CRC8_check_sum((uint8_t *) &siteEventMsgs, (sizeof(siteEventMsgs.head) -
                                                                                       sizeof(siteEventMsgs.head.crc)),
                                                          0xff)) && (siteEventMsgs.crc ==
                                                                     get_CRC16_check_sum((uint8_t *) &siteEventMsgs,
                                                                                         (sizeof(siteEventMsgs) -
                                                                                          sizeof(siteEventMsgs.crc)),
                                                                                         0xffff))) {
            gameStateRosMsg.if_supply_projectile_one_occupied = (siteEventMsgs.data.event_type | 0x80000000);
            gameStateRosMsg.if_supply_projectile_two_occupied = (siteEventMsgs.data.event_type | 0x40000000);
            gameStateRosMsg.if_supply_projectile_three_occupied = (siteEventMsgs.data.event_type | 0x20000000);
            gameStateRosMsg.if_wind_mill_hit_place_occupied = (siteEventMsgs.data.event_type | 0x10000000);
            gameStateRosMsg.if_wind_mill_big_lighted = (siteEventMsgs.data.event_type | 0x80000000);
            gameStateRosMsg.if_wind_mill_small_lighted = (siteEventMsgs.data.event_type | 0x08000000);
            gameStateRosMsg.if_rb2_occupied = (siteEventMsgs.data.event_type | 0x04000000);
            gameStateRosMsg.if_rb3_occupied = (siteEventMsgs.data.event_type | 0x02000000);
            gameStateRosMsg.if_rb4_occupied = (siteEventMsgs.data.event_type | 0x01000000);
            gameStateRosMsg.if_base_protected = (siteEventMsgs.data.event_type | 0x00800000);
            gameStateRosMsg.if_outpose_alive = (siteEventMsgs.data.event_type | 0x00400000);
            if_pub = true;
        }
        if ((supplyProjectileActionMsg.head.crc == get_CRC8_check_sum((uint8_t *) &supplyProjectileActionMsg,
                                                                      (sizeof(supplyProjectileActionMsg.head) -
                                                                       sizeof(supplyProjectileActionMsg.head.crc)),
                                                                      0xff)) && (supplyProjectileActionMsg.crc ==
                                                                                 get_CRC16_check_sum(
                                                                                         (uint8_t *) &supplyProjectileActionMsg,
                                                                                         (sizeof(supplyProjectileActionMsg) -
                                                                                          sizeof(supplyProjectileActionMsg.crc)),
                                                                                         0xffff))) {
            supplyProjectileActionRosMsg.supply_projectile_id = supplyProjectileActionMsg.data.supply_projectile_id;
            supplyProjectileActionRosMsg.supply_robot_id = supplyProjectileActionMsg.data.supply_robot_id;
            supplyProjectileActionRosMsg.supply_projectile_step = supplyProjectileActionMsg.data.supply_projectile_step;
            supplyProjectileActionRosMsg.supply_projectile_num = supplyProjectileActionMsg.data.supply_projectile_num;
            if_pub = true;
        }
        if ((refereeWarningMsg.head.crc == get_CRC8_check_sum((uint8_t *) &refereeWarningMsg,
                                                              (sizeof(refereeWarningMsg.head) -
                                                               sizeof(refereeWarningMsg.head.crc)), 0xff)) &&
            (refereeWarningMsg.crc == get_CRC16_check_sum((uint8_t *) &refereeWarningMsg,
                                                          (sizeof(refereeWarningMsg) -
                                                           sizeof(refereeWarningMsg.crc)), 0xffff))) {
            refereeWarningRosMsg.level = refereeWarningMsg.data.level;
            refereeWarningRosMsg.foul_robot_id = refereeWarningMsg.data.foul_robot_id;
            if_pub = true;
        }
        if ((dartRemainingTimeMsg.head.crc == get_CRC8_check_sum((uint8_t *) &dartRemainingTimeMsg,
                                                                 (sizeof(dartRemainingTimeMsg.head) -
                                                                  sizeof(dartRemainingTimeMsg.head.crc)), 0xff)) &&
            (dartRemainingTimeMsg.crc == get_CRC16_check_sum((uint8_t *) &dartRemainingTimeMsg,
                                                             (sizeof(dartRemainingTimeMsg) -
                                                              sizeof(dartRemainingTimeMsg.crc)), 0xffff))) {
            gameStateRosMsg.dart_remaining_time = dartRemainingTimeMsg.data.dart_remaining_time;
            if_pub = true;
        }
        if (if_pub) {
            game_state_publisher_->publish(gameStateRosMsg);
//            gameStatePub.publish(gameStateRosMsg);
            return true;
        }
        return false;
    }
    return false;
}

/**
 * 目标世界坐标回调函数
 * @param msg 收到的消息
 */
void SerialPort::worldPointsCallback(const radar_interfaces::msg::Points::SharedPtr msg) const {
    warn_state = msg->id;
    static int pubCount = 0;
    if (this->is_enemy_red) {
        for (int i = 0; i < msg->data.size(); i++) {
            if (msg->data[i].id <= 4) {
                car_point carPoint;
                carPoint.id = msg->data[i].id + 1;
                carPoint.color = 0;
                carPoint.point = Point2f((msg->data[i].x * 15.0), (msg->data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            } else if (msg->data[i].id == 5) {
                car_point carPoint;
                carPoint.id = 7;
                carPoint.color = 0;
                carPoint.point = Point((msg->data[i].x * 15.0), (msg->data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            } else if (msg->data[i].id == 12) {
                car_point carPoint;
                if (pubCount == 0) {
                    carPoint.id = 6;
                } else if (pubCount == 1) {
                    carPoint.id = 9;
                } else if (pubCount == 2) {
                    carPoint.id = 10;
                } else if (pubCount == 3) {
                    carPoint.id = 11;
                }
                if (pubCount >= 3) {
                    pubCount = 0;
                } else {
                    pubCount++;
                }
                carPoint.color = 0;
                carPoint.point = Point((msg->data[i].x * 15.0), (msg->data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            }
        }
    } else {
        for (int i = 0; i < msg->data.size(); i++) {
            if (msg->data[i].id >= 6 || msg->data[i].id <= 10) {
                car_point carPoint;
                carPoint.id = msg->data[i].id - 5;
                carPoint.color = 1;
                carPoint.point = Point((msg->data[i].x * 15.0), (msg->data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            } else if (msg->data[i].id == 11) {
                car_point carPoint;
                carPoint.id = 7;
                carPoint.color = 1;
                carPoint.point = Point((msg->data[i].x * 15.0), (msg->data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            } else if (msg->data[i].id == 13) {
                car_point carPoint;
                carPoint.id = 6;
                if (pubCount == 0) {
                    carPoint.id = 6;
                } else if (pubCount == 1) {
                    carPoint.id = 9;
                } else if (pubCount == 2) {
                    carPoint.id = 10;
                } else if (pubCount == 3) {
                    carPoint.id = 11;
                }
                if (pubCount >= 3) {
                    pubCount = 0;
                } else {
                    pubCount++;
                }
                carPoint.color = 1;
                carPoint.point = Point((msg->data[i].x * 15.0), (msg->data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            }
        }
    }
}

/**
 * 哨兵相关预警消息的接受
 * @param msg
 */
void SerialPort::GuardCallback(const radar_interfaces::msg::Points::SharedPtr msg) {
    this->robotInteractiveMsgs.data.content[0]=0xcc;
    this->robotInteractiveMsgs.data.content[1]=(int16_t)msg->data[0].x;
    this->robotInteractiveMsgs.data.content[2]=(int16_t)msg->data[0].x>>8;
    this->robotInteractiveMsgs.data.content[3]=(int16_t)msg->data[0].y;
    this->robotInteractiveMsgs.data.content[4]=(int16_t)msg->data[0].y>>8;
    this->robotInteractiveMsgs.data.content[5]=(int16_t)msg->data[0].z;
    this->robotInteractiveMsgs.data.content[6]=(int16_t)msg->data[0].z>>8;
    this->robotInteractiveMsgs.data.content[7]=(int16_t)msg->data[1].x;
    this->robotInteractiveMsgs.data.content[8]=(int16_t)msg->data[1].x>>8;
    this->robotInteractiveMsgs.data.content[9]=(int16_t)msg->data[1].y;
    this->robotInteractiveMsgs.data.content[10]=(int16_t)msg->data[1].y>>8;
    this->robotInteractiveMsgs.data.content[11]=(int16_t)msg->data[1].z;
    this->robotInteractiveMsgs.data.content[12]=(int16_t)msg->data[1].z>>8;
}
