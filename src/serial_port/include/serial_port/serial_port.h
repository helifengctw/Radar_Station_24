#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

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

using namespace cv;
using std::placeholders::_1;
using namespace std::chrono_literals;

//消息头
struct frame_header
{
    uint8_t SOF = 0xA5;//固定值
    uint16_t data_length;//data的长度
    uint8_t seq; //包序号
    uint8_t crc; //帧头crc8
} __attribute__((packed));

//小地图消息数据 10hz 发送
struct map_robot_data
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} __attribute__((packed));

struct map_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0305;
    map_robot_data data;
    uint16_t crc;
} __attribute__((packed)); // 最大5hz发送

struct mark_data {
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} __attribute__((packed));

struct mark_msgs {
    frame_header head;
    uint16_t cmd_id = 0x020c;
    mark_data data;
    uint16_t crc;
} __attribute__((packed));

struct double_buff_info_data {
    uint8_t radar_info;
} __attribute__((packed));

struct double_buff_info_msgs {
    frame_header head;
    uint16_t cmd_id = 0x020e;
    double_buff_info_data double_buff_info;
    uint16_t crc;
} __attribute__((packed));

struct double_buff_cmd_data {
    uint8_t radar_cmd;
} __attribute__((packed));

struct double_buff_cmd_msgs {
    frame_header head;
    uint16_t cmd_id = 0x020e;
    double_buff_cmd_data double_buff_cmd;
    uint16_t crc;
} __attribute__((packed));

//车间通讯
struct robot_interactive_data//最大10HZ 发送和接收
{
    uint16_t cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t content[113];
} __attribute__((packed));

struct robot_interactive_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0301;
    robot_interactive_data data;
    uint16_t crc;
} __attribute__((packed));

// 车辆血量消息
// 1hz 接收
struct robot_health_data {
    uint16_t red_1_robot_hp = 0;
    uint16_t red_2_robot_hp = 0;
    uint16_t red_3_robot_hp = 0;
    uint16_t red_4_robot_hp = 0;
    uint16_t red_5_robot_hp = 0;
    uint16_t red_7_robot_hp = 0;
    uint16_t red_outpose_hp = 0;//前哨站
    uint16_t red_base_hp = 0;//基地

    uint16_t blue_1_robot_hp = 0;
    uint16_t blue_2_robot_hp = 0;
    uint16_t blue_3_robot_hp = 0;
    uint16_t blue_4_robot_hp = 0;
    uint16_t blue_5_robot_hp = 0;
    uint16_t blue_7_robot_hp = 0;
    uint16_t blue_outpose_hp = 0;//前哨站
    uint16_t blue_base_hp = 0;//基地
} __attribute__((packed));

//1HZ
struct robot_health_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0003;
    robot_health_data data;
    uint16_t crc;
} __attribute__((packed));


//比赛状态信息
// 1hz 接收
struct game_status_data {
    uint8_t game_type: 4; //1：机甲大师赛 2：单项赛 3：人工智能挑战赛 4：联盟赛3v3 5：联盟赛1v1
    uint8_t game_progress: 4; //0：未开始比赛 1：准备阶段 2：自检阶段 3：5s倒计时 4：对战中 5：比赛结算中
    uint16_t stage_remain_time; //当前阶段剩余时间，单位s
    uint64_t SyncTimeStamp; //机器人接收到该指令的精确Unix时间,当机载端收到有效的NTP服务器授时后生效
} __attribute__((packed));

struct game_status_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0001;
    game_status_data data;
    uint16_t crc;
} __attribute__((packed));

// 比赛结果
// 比赛结束发送 接收
struct game_result_data {
    uint8_t winner; //0平局 1红方胜利 2蓝方胜利
} __attribute__((packed));

struct game_result_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0002;
    game_result_data data;
    uint16_t crc;
} __attribute__((packed));

//飞镖交互消息
struct dart_interactivate_msg_data {
    uint8_t dart_launch_opening_status; //fei biao fa she kou zhuang tai 1 guan bi 2 zheng zai huo dong 0 yi jing kai qi
    uint8_t dart_attack_target; //飞镖的打击目标,默认为前哨站;0:前哨站; 1:基地。
    uint16_t target_change_time; //切换打击目标时的比赛剩余时间,单位秒,从未切换默认为 0。
    uint16_t operate_launch_cmd_time; //最近一次操作手确定发射指令时的比赛剩余时间,单位秒, 初始值为 0。
} __attribute__((packed));

//场地机关占领消息
// 1hz 接收
struct site_event_data {
    uint32_t event_type;
    //bit 0:己方补给站 1 号补血点占领状态 1 为已占领;
    //bit 1:己方补给站 2 号补血点占领状态 1 为已占领;
    //bit 2:己方补给站 3 号补血点占领状态 1 为已占领;
    //bit 3-5:己方能量机关状态:
    // bit 3 为打击点占领状态,1 为占领;
    // bit 4 为小能量机关激活状态,1 为已激活;
    // bit 5 为大能量机关激活状态,1 为已激活;
    //bit 6:己方侧 R2/B2 环形高地占领状态 1 为已占领;
    //bit 7:己方侧 R3/B3 梯形高地占领状态 1 为已占领;
    //bit 8:己方侧 R4/B4 梯形高地占领状态 1 为已占领;
    //bit 9:己方基地护盾状态: 1 为基地有虚拟护盾血量; 0 为基地无虚拟护盾血量;
    //bit 10:己方前哨战状态: 1 为前哨战存活; 0 为前哨战被击毁;
    //bit 10 -31: 保留
} __attribute__((packed));

struct site_event_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0101;
    site_event_data data;
    uint16_t crc;
} __attribute__((packed));


//裁判判罚消息
//触发时发送 接收
struct referee_warning_data {
    uint8_t level; //1黄牌 2红牌 3判负
    uint8_t foul_robot_id; //犯规机器人ID 判负时为0
} __attribute__((packed));

struct referee_warning_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0104;
    referee_warning_data data;
    uint16_t crc;
} __attribute__((packed));



class SerialPort : public rclcpp::Node {
public:
    SerialPort(std::string name);
    ~SerialPort();
    serial::Serial ser;
    void TimerCallback();

private:
    rclcpp::Subscription<radar_interfaces::msg::Points>::SharedPtr world_point_subscription_;
    rclcpp::Publisher<radar_interfaces::msg::MarkData>::SharedPtr mark_data_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int ser_init();
    void worldPointsCallback(radar_interfaces::msg::Points::SharedPtr);
    void load_params();

    map_msgs mapMsg;
    map_robot_data mapRobotData;
    mark_msgs markMsg;
    double_buff_info_msgs doubleBuffInfoMsg;
    double_buff_cmd_msgs doubleBuffCmdMsg;
    robot_interactive_msgs robotInteractiveMsgs;
    robot_health_msgs robotHealthMsgs;
    game_result_msgs gameResultMsg;
    site_event_msgs siteEventMsgs;
    referee_warning_msgs refereeWarningMsg;
    game_status_msgs gameStatusMsgs;

    radar_interfaces::msg::GameState gameStateRosMsg;
    radar_interfaces::msg::RefereeWarning refereeWarningRosMsg;
    radar_interfaces::msg::MarkData markDataRosMsg;

    uint8_t receiveData[1024];
    int double_buff_chance = 0, used_double_buff_chance = 0, detected_enemy_count = 0;
    float field_height = 2800, field_width = 1500;
    bool if_enemy_red = false, if_double_buff_exerting = false, if_receive = false, if_prepared = false;
    mark_data referee_mark_data, last_referee_mark_data;

    bool sendMapMsgs();
    bool sendInteractiveMsgs(uint16_t);
    bool TriggerDoubleBuffOnce();
    bool receiveMsgs();
    void publish_mark_data();
};

#endif //SERIAL_PORT_H
