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
#include "radar_interfaces/msg/mark_data.hpp"
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

    SerialPort(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Node initialized !!!");
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
        mark_data_publisher_ = this->create_publisher<radar_interfaces::msg::MarkData>(
                "mark_data", 1);

        world_point_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
                "/world_point", 1, std::bind(&SerialPort::worldPointsCallback, this, _1));

        doubleBuffCmdMsg.double_buff_cmd.radar_cmd = 0;

        while (!parameter_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
//            return -1;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the parameter service to start...");
        }
    }

    serial::Serial ser;
    map_msg mapMsg;
    mark_msg markMsg;
    double_buff_info_msg doubleBuffInfoMsg;
    double_buff_cmd_msg doubleBuffCmdMsg;
    robot_interactive_msgs robotInteractiveMsgs;
    robot_interactive_control_msgs robotInteractiveControlMsgs;
    robot_health_msgs robotHealthMsgs;
    game_result_msg gameResultMsg;
    site_event_msgs siteEventMsgs;
    supply_projectile_action_msg supplyProjectileActionMsg;
    referee_warning_msg refereeWarningMsg;
    dart_remaining_time_msg dartRemainingTimeMsg;
    game_status_msgs gameStatusMsgs;

    radar_interfaces::msg::GameState gameStateRosMsg;
    radar_interfaces::msg::SupplyProjectileAction supplyProjectileActionRosMsg;
    radar_interfaces::msg::RefereeWarning refereeWarningRosMsg;
    radar_interfaces::msg::MarkData markDataRosMsg;
    uint8_t receiveData[1024];
    int double_buff_chance = 0, used_double_buff_chance = 0;
    bool is_enemy_red = false, if_double_buff_exerting = false;
    mark_data referee_mark_data, last_referee_mark_data;

    bool sendMapMsgs(uint16_t, float, float);
    bool sendInteractiveMsgs(uint16_t);
    bool sendDoubleBuffCmdMsgs();
    bool receiveMsgs();
    void LoadBattleColor();
    void publish_mark_data();

private:
    std::shared_ptr<rclcpp::SyncParametersClient> parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this, "pnp_solver");

    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr world_point_subscription_;

    rclcpp::Publisher<radar_interfaces::msg::GameState>::SharedPtr game_state_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::SupplyProjectileAction>::SharedPtr supply_projectile_action_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::RefereeWarning>::SharedPtr referee_warning_publisher_;
    rclcpp::Publisher<radar_interfaces::msg::MarkData>::SharedPtr mark_data_publisher_;

    void worldPointsCallback(const radar_interfaces::msg::Points::SharedPtr) const;
};

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
        count = 0;
        //逐一发送小地图目标点，当等待发布的点为空时接收一次新的消息
        if (!worldPoints.empty()) {
            int write_count = 0;
            for (auto &i: worldPoints) {
                int id_2_send = i.id + i.color * 100;
                sp_node->sendMapMsgs(id_2_send, i.point.x, i.point.y);
                sp_node->robotInteractiveMsgs.data.content[write_count++] = (int16_t) id_2_send;
                sp_node->robotInteractiveMsgs.data.content[write_count++] = (int16_t) i.point.x;
                sp_node->robotInteractiveMsgs.data.content[write_count++] = (int16_t) i.point.x >> 8;
                sp_node->robotInteractiveMsgs.data.content[write_count++] = (int16_t) i.point.y;
                sp_node->robotInteractiveMsgs.data.content[write_count++] = (int16_t) i.point.y >> 8;
            }
            worldPoints.clear();
            sp_node->sendInteractiveMsgs(7);
//            sp_node->robotInteractiveMsgs.data.content = ;
//            std::cout << worldPoints[0].id << ": (" << worldPoints[0].point.x << "," << worldPoints[0].point.y << ")" << std::endl;
        } else {
            rclcpp::spin_some(sp_node);
        }
        sp_node->receiveMsgs();  //接受来自串口的所有消息并加以判断、处理
        if (sp_node->double_buff_chance > sp_node->used_double_buff_chance) {
            sp_node->sendDoubleBuffCmdMsgs();
            if (sp_node->if_double_buff_exerting) sp_node->used_double_buff_chance++;
        }
        rclcpp::spin_some(sp_node);
        //循环休眠
        loop.sleep();
    }
    sp_node->ser.close(); // 关闭串口
//    rclcpp::shutdown();
    return 0;
}


/**
 * 读取我方阵营参数
 */
void SerialPort::LoadBattleColor() {
    std::vector<rclcpp::Parameter> param_vector;
    param_vector = parameter_client->get_parameters(
            {"battle_state.battle_color"});

    std::string exchange = param_vector[0].as_string();
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
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        //串口设置
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);

        ser.setStopbits(serial::stopbits_t::stopbits_one);
        ser.setBytesize(serial::bytesize_t::eightbits);
        ser.setParity(serial::parity_t::parity_none);       //设置校验位

        ser.open(); //打开
    }
    catch (serial::IOException &e) {
        std::cout << "unable to open ser" << std::endl;
        return -1;
    }
    catch (std::invalid_argument &e) {
        std::cout << "std::invalid_argument" << std::endl;
        return -1;
    }
    catch (serial::SerialException &e) {
        std::cout << "serial::SerialException" << std::endl;
        return -1;
    }
    //检查串口
    if (ser.isOpen()) {
        std::cout << "ser open" << std::endl;
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


bool SerialPort::sendDoubleBuffCmdMsgs() {
    doubleBuffCmdMsg.head.SOF = 0xA5;
    doubleBuffCmdMsg.head.data_length = 1;
    doubleBuffCmdMsg.head.seq = 1;
    doubleBuffCmdMsg.head.crc = get_CRC8_check_sum((uint8_t *) &doubleBuffCmdMsg,
                                                   (sizeof(doubleBuffCmdMsg.head) -
                                                    sizeof(doubleBuffCmdMsg.head.crc)), 0xff);
    doubleBuffCmdMsg.cmd_id = 0x0121;
    doubleBuffCmdMsg.double_buff_cmd.radar_cmd++;
    doubleBuffCmdMsg.crc = get_CRC16_check_sum((uint8_t *) &doubleBuffCmdMsg,
                                               (sizeof(doubleBuffCmdMsg) - sizeof(doubleBuffCmdMsg.crc)), 0xffff);
    ser.write((uint8_t *) &doubleBuffCmdMsg, sizeof(doubleBuffCmdMsg));
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
//    std::cout << std::endl << robotInteractiveMsgs.data.sender_id << " ==> " << robotInteractiveMsgs.data.receiver_id << std::endl;
    robotInteractiveMsgs.crc = get_CRC16_check_sum((uint8_t *) &robotInteractiveMsgs,
                                                   (sizeof(robotInteractiveMsgs) -
                                                    sizeof(robotInteractiveMsgs.crc)), 0xffff);
    ser.write((uint8_t *) &robotInteractiveMsgs, sizeof(robotInteractiveMsgs));
//    std::cout << "Send one interactive msg " << std::endl;
//    std::cout << robotInteractiveMsgs.data.receiver_id << std::endl;
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
        markMsg = (*(mark_msg * ) receiveData);
        gameResultMsg = (*(game_result_msg *) receiveData);
        siteEventMsgs = (*(site_event_msgs *) receiveData);
        supplyProjectileActionMsg = (*(supply_projectile_action_msg *) receiveData);
        refereeWarningMsg = (*(referee_warning_msg *) receiveData);
        dartRemainingTimeMsg = (*(dart_remaining_time_msg *) receiveData);

        gameStateRosMsg.dart_remaining_time = 16;  //己方飞镖发射剩余时间
        gameStateRosMsg.winner = 3;  // 0:平局，1:红方胜利，2:蓝方胜利
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

//        if ((markMsg.head.crc == get_CRC8_check_sum((uint8_t *) &markMsg,
//                                                            (sizeof(markMsg.head) -
//                                                             sizeof(markMsg.head.crc)), 0xff)) &&
//            (markMsg.crc == get_CRC16_check_sum((uint8_t *) &markMsg,
//                                                        (sizeof(markMsg) - sizeof(markMsg.crc)),
//                                                        0xffff))) {
//            last_referee_mark_data = referee_mark_data;
//            referee_mark_data = markMsg.data;
//            publish_mark_data();
//        }

        if ((doubleBuffInfoMsg.head.crc == get_CRC8_check_sum((uint8_t *) &doubleBuffInfoMsg,
                                                    (sizeof(doubleBuffInfoMsg.head) -
                                                     sizeof(doubleBuffInfoMsg.head.crc)), 0xff)) &&
            (doubleBuffInfoMsg.crc == get_CRC16_check_sum((uint8_t *) &doubleBuffInfoMsg,
                                                (sizeof(doubleBuffInfoMsg) - sizeof(doubleBuffInfoMsg.crc)),
                                                0xffff))) {
            double_buff_chance = (doubleBuffInfoMsg.double_buff_info.radar_info >> 6) & 0x03;
            if_double_buff_exerting = (doubleBuffInfoMsg.double_buff_info.radar_info >> 5) & 0x01;
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
            refereeWarningRosMsg.level = refereeWarningMsg.data.level; // 1:双方黄牌，2:黄牌，3:红牌，4:判负
            refereeWarningRosMsg.foul_robot_id = refereeWarningMsg.data.foul_robot_id; //  红1:1，蓝1:101，判负双方黄牌:0
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
 * 发布标记进度消息
 */
void SerialPort::publish_mark_data() {
    markDataRosMsg.mark_list[0] = referee_mark_data.mark_hero_progress - last_referee_mark_data.mark_hero_progress;
    markDataRosMsg.mark_list[1] = referee_mark_data.mark_engineer_progress - last_referee_mark_data.mark_engineer_progress;
    markDataRosMsg.mark_list[2] = referee_mark_data.mark_standard_3_progress - last_referee_mark_data.mark_standard_3_progress;
    markDataRosMsg.mark_list[3] = referee_mark_data.mark_standard_4_progress - last_referee_mark_data.mark_standard_4_progress;
    markDataRosMsg.mark_list[4] = referee_mark_data.mark_standard_5_progress - last_referee_mark_data.mark_standard_5_progress;
    markDataRosMsg.mark_list[5] = referee_mark_data.mark_sentry_progress - last_referee_mark_data.mark_sentry_progress;
    mark_data_publisher_->publish(markDataRosMsg);

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
                worldPoints.push_back(carPoint);
//                worldPoints.insert(worldPoints.begin(), carPoint);
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
                worldPoints.push_back(carPoint);
//                worldPoints.insert(worldPoints.begin(), carPoint);
            }
        }
    }
}

