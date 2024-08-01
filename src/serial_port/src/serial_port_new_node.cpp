#include "serial_port.h"

int main(int argc, char **argv) {
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto SP_node = std::make_shared<SerialPort>("serial_port");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(SP_node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}

SerialPort::SerialPort(std::string name) : Node(name){
    if (ser_init() < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port, please check the hardware !!!");
    } else {
        RCLCPP_INFO(this->get_logger(), "successfully initialize serial port !!!");
    }
    load_params();
    mark_data_publisher_ = this->create_publisher<radar_interfaces::msg::MarkData>("mark_data", 1);
    world_point_subscription_ = this->create_subscription<radar_interfaces::msg::Points>(
            "/world_point", 1, std::bind(&SerialPort::worldPointsCallback, this, _1));
    timer_ = this->create_wall_timer(200ms, std::bind(&SerialPort::TimerCallback, this));

    doubleBuffCmdMsg.double_buff_cmd.radar_cmd = 0;
}

SerialPort::~SerialPort() {
    ser.close();
    RCLCPP_INFO(this->get_logger(), "Serial_port closed !!!");
}

void SerialPort::TimerCallback() {
    if (receiveMsgs()) { //接受来自串口的所有消息并加以判断、处理
        if_receive = false;
        if (double_buff_chance > used_double_buff_chance) {
            if (if_double_buff_exerting) used_double_buff_chance++;
            else {
                if (gameStatusMsgs.data.stage_remain_time < 5*60) {
                    if (detected_enemy_count > 3) TriggerDoubleBuffOnce();
                } else if (gameStatusMsgs.data.stage_remain_time < 2*60) {
                    TriggerDoubleBuffOnce();
                }
            }
        }
    }
}

/**
 * 串口初始化
 * @return
 */
int SerialPort::ser_init() {
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

void SerialPort::load_params() {
    this->declare_parameter<std::string>("our_color", "our_init");
    std::string our_color = this->get_parameter("our_color").as_string();
    if (our_color == "red") if_enemy_red = false;
    else if (our_color == "blue") if_enemy_red = true;
}

bool SerialPort::sendMapMsgs() {
    mapMsg.head.SOF = 0xA5;
    mapMsg.head.data_length = 24;
    mapMsg.head.seq = 1;
    mapMsg.head.crc = get_CRC8_check_sum((uint8_t *) &mapMsg, (sizeof(mapMsg.head) - sizeof(mapMsg.head.crc)),
                                         0xff);
    mapMsg.cmd_id = 0x0305;
    mapMsg.data = mapRobotData;
    mapMsg.crc = get_CRC16_check_sum((uint8_t *) &mapMsg, (sizeof(mapMsg) - sizeof(mapMsg.crc)), 0xffff);
    ser.write((uint8_t *) &mapMsg, sizeof(map_msg));
    return true;
}

bool SerialPort::TriggerDoubleBuffOnce() {
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
bool SerialPort::sendInteractiveMsgs(uint16_t receiver_id) { //接受者ID以红方为准
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
    if (if_enemy_red) {
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
        ser.read(receiveData, ser.available());
        //使用所有消息类型进行匹配，若CRC校验通过，则匹配成功
        gameStatusMsgs = (*(game_status_msgs *) receiveData);
        robotHealthMsgs = (*(robot_health_msgs *) receiveData);
        markMsg = (*(mark_msg *) receiveData);
        gameResultMsg = (*(game_result_msg *) receiveData);
        siteEventMsgs = (*(site_event_msgs *) receiveData);
        refereeWarningMsg = (*(referee_warning_msg *) receiveData);

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
            if_receive = true;
        }

        if ((markMsg.head.crc == get_CRC8_check_sum((uint8_t *) &markMsg,
                                                            (sizeof(markMsg.head) -
                                                             sizeof(markMsg.head.crc)), 0xff)) &&
            (markMsg.crc == get_CRC16_check_sum((uint8_t *) &markMsg,
                                                        (sizeof(markMsg) - sizeof(markMsg.crc)),
                                                        0xffff))) {
            last_referee_mark_data = referee_mark_data;
            referee_mark_data = markMsg.data;
            publish_mark_data();
            if_receive = true;
        }

        if ((doubleBuffInfoMsg.head.crc == get_CRC8_check_sum((uint8_t *) &doubleBuffInfoMsg,
                                                    (sizeof(doubleBuffInfoMsg.head) -
                                                     sizeof(doubleBuffInfoMsg.head.crc)), 0xff)) &&
            (doubleBuffInfoMsg.crc == get_CRC16_check_sum((uint8_t *) &doubleBuffInfoMsg,
                                                (sizeof(doubleBuffInfoMsg) - sizeof(doubleBuffInfoMsg.crc)),
                                                0xffff))) {
            double_buff_chance = doubleBuffInfoMsg.double_buff_info.radar_info & 0x03;
            if_double_buff_exerting = (doubleBuffInfoMsg.double_buff_info.radar_info >> 2) & 0x01;
            if_receive = true;
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
            if_receive = true;
        }

        if ((gameResultMsg.head.crc == get_CRC8_check_sum((uint8_t *) &gameResultMsg, (sizeof(gameResultMsg.head) -
                                                                                       sizeof(gameResultMsg.head.crc)),
                                                          0xff)) && (gameResultMsg.crc ==
                                                                     get_CRC16_check_sum((uint8_t *) &gameResultMsg,
                                                                                         (sizeof(gameResultMsg) -
                                                                                          sizeof(gameResultMsg.crc)),
                                                                                         0xffff))) {
            gameStateRosMsg.winner = gameResultMsg.data.winner;
            if_receive = true;
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
            if_receive = true;
        }

        if ((refereeWarningMsg.head.crc == get_CRC8_check_sum((uint8_t *) &refereeWarningMsg,
                                                              (sizeof(refereeWarningMsg.head) -
                                                               sizeof(refereeWarningMsg.head.crc)), 0xff)) &&
            (refereeWarningMsg.crc == get_CRC16_check_sum((uint8_t *) &refereeWarningMsg,
                                                          (sizeof(refereeWarningMsg) -
                                                           sizeof(refereeWarningMsg.crc)), 0xffff))) {
            refereeWarningRosMsg.level = refereeWarningMsg.data.level; // 1:双方黄牌，2:黄牌，3:红牌，4:判负
            refereeWarningRosMsg.foul_robot_id = refereeWarningMsg.data.foul_robot_id; //  红1:1，蓝1:101，判负双方黄牌:0
            if_receive = true;
        }

        if (if_receive) return true;
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
void SerialPort::worldPointsCallback(const radar_interfaces::msg::Points::SharedPtr msg){
    if (msg->data.empty()) return;
    detected_enemy_count = 0;
    if (if_enemy_red) {
        for (auto p: msg->data) {
            if (p.id == 0) {
                detected_enemy_count++;
                mapRobotData.hero_position_x = int(p.x * field_height);
                mapRobotData.hero_position_y = int(p.y * field_width);
            } else if (p.id == 1) {
                detected_enemy_count++;
                mapRobotData.engineer_position_x = int(p.x * field_height);
                mapRobotData.engineer_position_y = int(p.y * field_width);
            } else if (p.id == 2) {
                detected_enemy_count++;
                mapRobotData.infantry_3_position_x = int(p.x * field_height);
                mapRobotData.infantry_3_position_y = int(p.y * field_width);
            } else if (p.id == 3) {
                detected_enemy_count++;
                mapRobotData.infantry_4_position_x = int(p.x * field_height);
                mapRobotData.infantry_4_position_y = int(p.y * field_width);
            } else if (p.id == 4) {
                detected_enemy_count++;
                mapRobotData.infantry_5_position_x = int(p.x * field_height);
                mapRobotData.infantry_5_position_y = int(p.y * field_width);
            } else if (p.id == 3) {
                detected_enemy_count++;
                mapRobotData.infantry_4_position_x = int(p.x * field_height);
                mapRobotData.infantry_4_position_y = int(p.y * field_width);
            } else if (p.id == 5) {
                detected_enemy_count++;
                mapRobotData.sentry_position_x = int(p.x * field_height);
                mapRobotData.sentry_position_y = int(p.y * field_width);
            } else if (p.id == 12) {
                //TODO: 处理未知
            }
        }
    } else {
        for (auto p: msg->data) {
            if (p.id == 6) {
                detected_enemy_count++;
                mapRobotData.hero_position_x = int(p.x * field_height);
                mapRobotData.hero_position_y = int(p.y * field_width);
            } else if (p.id == 7) {
                detected_enemy_count++;
                mapRobotData.engineer_position_x = int(p.x * field_height);
                mapRobotData.engineer_position_y = int(p.y * field_width);
            } else if (p.id == 8) {
                detected_enemy_count++;
                mapRobotData.infantry_3_position_x = int(p.x * field_height);
                mapRobotData.infantry_3_position_y = int(p.y * field_width);
            } else if (p.id == 9) {
                detected_enemy_count++;
                mapRobotData.infantry_4_position_x = int(p.x * field_height);
                mapRobotData.infantry_4_position_y = int(p.y * field_width);
            } else if (p.id == 10) {
                detected_enemy_count++;
                mapRobotData.infantry_5_position_x = int(p.x * field_height);
                mapRobotData.infantry_5_position_y = int(p.y * field_width);
            } else if (p.id == 11) {
                detected_enemy_count++;
                mapRobotData.sentry_position_x = int(p.x * field_height);
                mapRobotData.sentry_position_y = int(p.y * field_width);
            } else if (p.id == 13) {
                //TODO: 处理未知
            }
        }
    }
    if (sendMapMsgs()) {
        std::cout << "Send map msg successfully --- detected enemy count:[" << detected_enemy_count << std::endl;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send map msg !!!");
    }
}

