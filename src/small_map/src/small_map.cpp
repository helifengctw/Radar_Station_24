#include "small_map/small_map.h"

// 1, 7:工程  *  0, 6:英雄  *
int main(int argc, char **argv) {
    std::cout << "ros node init" << std::endl;
    rclcpp::init(argc, argv);
    auto SM_node = std::make_shared<SmallMap>("small_map");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(SM_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}


SmallMap::SmallMap(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "small map node initialize!!!");
    far_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoints>(
            "/sensor_far/distance_point", 1, std::bind(&SmallMap::far_distPointCallback, this, _1));
    close_distant_point_subscription_ = this->create_subscription<radar_interfaces::msg::DistPoints>(
            "/sensor_close/distance_point", 1, std::bind(&SmallMap::close_distPointCallback, this, _1));
    pickup_information_subscription_ = this->create_subscription<radar_interfaces::msg::Point>(
            "pickup_information", 1, std::bind(&SmallMap::pickup_infoCallback, this, _1));
    game_status_subscription_ = this->create_subscription<robot_serial::msg::Gamestatus>(
            "/serial_gamestatus", 1, std::bind(&SmallMap::game_status_Callback, this, _1));
    double_info_subscription_ = this->create_subscription<robot_serial::msg::DoubleInfo>(
            "/serial_double_info", 1, std::bind(&SmallMap::double_info_Callback, this, _1));
    event_subscription_ = this->create_subscription<robot_serial::msg::Event>(
            "/serial_event", 1, std::bind(&SmallMap::event_Callback, this, _1));
    timer_ = this->create_wall_timer(50ms, std::bind(&SmallMap::TimerCallback, this));
    serial_world_point_publisher_ = this->create_publisher<robot_serial::msg::MapPoints>("/serial_world_points", 1);
    double_hurt_cmd_publisher_ = this->create_publisher<robot_serial::msg::DoubleHurt>("/double_hurt_cmd", 1);
    world_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/world_point", 1);
    Pnp_result_client_ = this->create_client<radar_interfaces::srv::PnpResult>("pnp_results");

    this->load_param();

    img_show_width = (int)field_width*30;
    img_show_height = (int)field_height*30;
    small_map = cv::imread(small_map_png_path);
    if (small_map.empty()) RCLCPP_ERROR(this->get_logger(), "small map load failed!!!");
    cv::resize(small_map, small_map, Size(img_show_width, img_show_height));
    small_map.copyTo(small_map_copy);
    cv::namedWindow("small_map");
    cv::startWindowThread();

    //等待服务端上线
    while (!Pnp_result_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "等待被打断, 不等了");
            rclcpp::shutdown();
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Request for pnp result");
    }

    pnp_request = std::make_shared<radar_interfaces::srv::PnpResult::Request>();
    pnp_request->small_map_qidong = true;
    RCLCPP_INFO(this->get_logger(), "small_map qidong: %d", pnp_request->small_map_qidong);
    //发送异步请求，然后等待返回，返回时调用回调函数
    Pnp_result_client_->async_send_request(pnp_request, std::bind(&SmallMap::Pnp_resultCallback, this, _1));
    pnp_request->small_map_qidong = false;
}

SmallMap::~SmallMap() {
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "small map node shutdown!!!");
}

void SmallMap::TimerCallback() {
    small_map.copyTo(small_map_copy);
//    for (auto &i: far_points.data) {
//        draw_point_on_map(i, small_map_copy, "Yellow");
//    }
//    for (auto &i : close_points.data) {
//        draw_point_on_map(i, small_map_copy, "Orange");
//    }
    remove_duplicate();
//    std::cout << far_points.data.size() << ", " << close_points.data.size() << " ===> ";
    for (auto &i: result_points.data) {
        draw_point_on_map(i, small_map_copy, "Black");
        if (red_or_blue) {
            if (i.id == 0) {
                if (i.x >= 0.56) detected_dangerous_enemy_count += 3;
                serial_world_points.x1 = uint16_t (i.x * 2800);
                serial_world_points.y1 = uint16_t (i.y * 1500);
            } else if (i.id == 1) {
                if (i.x >= 0.56) detected_dangerous_enemy_count++;
                serial_world_points.x2 = uint16_t (i.x * 2800);
                serial_world_points.y2 = uint16_t (i.y * 1500);
            } else if (i.id == 2) {
                if (i.x >= 0.56) detected_dangerous_enemy_count += 2;
                serial_world_points.x3 = uint16_t (i.x * 2800);
                serial_world_points.y3 = uint16_t (i.y * 1500);
            } else if (i.id == 3) {
                if (i.x >= 0.56) detected_dangerous_enemy_count += 2;
                serial_world_points.x4 = uint16_t (i.x * 2800);
                serial_world_points.y4 = uint16_t (i.y * 1500);
            } else if (i.id == 4) {
                if (i.x >= 0.56) detected_dangerous_enemy_count += 2;
                serial_world_points.x5 = uint16_t (i.x * 2800);
                serial_world_points.y5 = uint16_t (i.y * 1500);
            } else if (i.id == 5) {
                if (i.x >= 0.56) detected_dangerous_enemy_count++;
                serial_world_points.x6 = uint16_t (i.x * 2800);
                serial_world_points.y6 = uint16_t (i.y * 1500);
            }
        } else {
            if (i.id == 6) {
                if (i.x < 0.44) detected_dangerous_enemy_count += 3;
                serial_world_points.x1 = uint16_t (i.x * 2800);
                serial_world_points.y1 = uint16_t (i.y * 1500);
            } else if (i.id == 7) {
                if (i.x < 0.44) detected_dangerous_enemy_count++;
                serial_world_points.x2 = uint16_t (i.x * 2800);
                serial_world_points.y2 = uint16_t (i.y * 1500);
            } else if (i.id == 8) {
                if (i.x < 0.44) detected_dangerous_enemy_count += 2;
                serial_world_points.x3 = uint16_t (i.x * 2800);
                serial_world_points.y3 = uint16_t (i.y * 1500);
            } else if (i.id == 9) {
                if (i.x < 0.44) detected_dangerous_enemy_count += 2;
                serial_world_points.x4 = uint16_t (i.x * 2800);
                serial_world_points.y4 = uint16_t (i.y * 1500);
            } else if (i.id == 10) {
                if (i.x < 0.44) detected_dangerous_enemy_count += 2;
                serial_world_points.x5 = uint16_t (i.x * 2800);
                serial_world_points.y5 = uint16_t (i.y * 1500);
            } else if (i.id == 11) {
                if (i.x < 0.44) detected_dangerous_enemy_count++;
                serial_world_points.x6 = uint16_t (i.x * 2800);
                serial_world_points.y6 = uint16_t (i.y * 1500);
            }
        }
    }
    cv::imshow("small_map", small_map_copy);
    serial_world_point_publisher_->publish(serial_world_points);
//    std::cout << "detected_dangerous_enemy_count, " << detected_dangerous_enemy_count << std::endl;
//    std::cout << far_points.data.size() << ", " << close_points.data.size() << std::endl;
//    this->world_point_publisher_->publish(result_points);

    if (last_game_progress < 4 && game_progress < 4) {
        double_hurt_msg.radar_cmd = 0x00;
        used_chance = 0x00;
    } else if (last_game_progress >= 4 && game_progress >= 4) {
        if (remain_time <= 6*60) {
            if (exerting && trigger_once) {
                used_chance++;
                trigger_once = false;
            } else if (!exerting && !trigger_once && double_hurt_chance > 0) {
                if (remain_time >= 70 && remain_time < 6*60) {
                    if (detected_dangerous_enemy_count >= 5) {
                        trigger_once = true;
                        big_energy_start_time = remain_time + 50;
                        trigger_time = remain_time + 15;
                    } else if (big_energy_enable) {
                        trigger_once = true;
                        big_energy_start_time = remain_time;
                        if (detected_dangerous_enemy_count >= 3) trigger_time = remain_time + 15;
                        else trigger_time = remain_time;
                    }
                } else if (used_chance == 0x00 && remain_time < 70) {
                    trigger_once = true;
                    big_energy_start_time = remain_time + 50;
                    trigger_time = remain_time + 15;
                } else if (used_chance == 0x01 && remain_time < 45) {
                    trigger_once = true;
                    big_energy_start_time = remain_time + 50;
                    trigger_time = remain_time + 15;
                }
            }
            if (trigger_once && !exerting && trigger_time - 10 >= remain_time) {
                if (used_chance == 0x00)  double_hurt_msg.radar_cmd = 0x01;
                else if (used_chance == 0x01 && big_energy_start_time - 45 > remain_time) double_hurt_msg.radar_cmd = 0x02;
            } else {
                if (used_chance == 0x00)  double_hurt_msg.radar_cmd = 0x00;
                else if (used_chance == 0x01) double_hurt_msg.radar_cmd = 0x01;
            }
        } else {
            double_hurt_msg.radar_cmd = 0x00;
        }
    }
    std::cout << "!!!!!!!cmd: " << (int)double_hurt_msg.radar_cmd << std::endl;
    double_hurt_cmd_publisher_->publish(double_hurt_msg);

    std::cout << "chance--used: " << (int)double_hurt_chance
              << " -- " << (int)used_chance
              << ", exerting: " << (int)exerting
              << ", trigger: " << trigger_once << std::endl;
    detected_dangerous_enemy_count = 0;
}

void SmallMap::game_status_Callback(robot_serial::msg::Gamestatus::SharedPtr msg) {
    remain_time = msg->stage_remain_time;
    last_game_progress = game_progress;
    game_progress = msg->game_progress;
    std::cout << "(0-未开始 ==>> 4-比赛中) stage: " << (int)game_progress
        << ", remain_time: " << (int)remain_time << std::endl;
}

void SmallMap::double_info_Callback(robot_serial::msg::DoubleInfo::SharedPtr msg) {
    if (double_hurt_chance < 3 && exerting < 2) {
        double_hurt_chance = msg->double_hurt_chance;
        exerting = msg->exerting;
    }
}

void SmallMap::event_Callback(robot_serial::msg::Event::SharedPtr msg) {
    small_energy_enable = msg->small_energy_organ_status;
    big_energy_enable = msg->big_energy_organ_status;
    std::cout << "small_energy: " << (int)small_energy_enable
        << ", big_energy: " << (int)big_energy_enable << std::endl;
}

void SmallMap::far_distPointCallback(const radar_interfaces::msg::DistPoints::SharedPtr input) {
    std::vector<radar_interfaces::msg::Point>().swap(far_points.data);
    for (int i = 0; i < input->data.size(); i++) {
        if (input->data[i].dist > 0) {
            Mat x8_pixel;
            x8_pixel = (Mat_<double>(3, 1) << (double) input->data[i].x, (double) input->data[i].y, 1);
            x8_pixel *= (1000 * input->data[i].dist);  // input->data[i].dist -- *10m
            Mat calcWorld = far_invR * (far_invM * x8_pixel - far_T); //2D-3D变换
            calcWorld /= 1000; // 单位：mm => m
            double x, y;
            if (red_or_blue) {
                x = 1 - calcWorld.at<double>(0, 0) / field_height;
                y = 1 - calcWorld.at<double>(1, 0) / field_width;
            } else {
                x = calcWorld.at<double>(0, 0) / field_height;
                y = calcWorld.at<double>(1, 0) / field_width;
            }
            radar_interfaces::msg::Point point;
            point.x = (float) x;
            point.y = (float) y;
            point.z = (float) calcWorld.at<double>(2, 0);
            point.id = input->data[i].id;
            point.conf = input->data[i].conf;
            point.tracker_id = input->data[i].tracker_id;
            far_points.data.push_back(point);
        }
    }
}

void SmallMap::close_distPointCallback(const radar_interfaces::msg::DistPoints::SharedPtr input) {
    std::vector<radar_interfaces::msg::Point>().swap(close_points.data);
    for (int i = 0; i < input->data.size(); i++) {
        if (input->data[i].dist > 0) {
            Mat x8_pixel;
            x8_pixel = (Mat_<double>(3, 1) << (double) input->data[i].x, (double) input->data[i].y, 1);
            x8_pixel *= (1000 * input->data[i].dist);
            Mat calcWorld = close_invR * (close_invM * x8_pixel - close_T); //2D-3D变换
            calcWorld /= 1000;
            double x, y;
            if (red_or_blue) {
                x = 1 - calcWorld.at<double>(0, 0) / field_height;
                y = 1 - calcWorld.at<double>(1, 0) / field_width;
            } else {
                x = calcWorld.at<double>(0, 0) / field_height;
                y = calcWorld.at<double>(1, 0) / field_width;
            }
            radar_interfaces::msg::Point point;
            point.x = (float) x;
            point.y = (float) y;
            point.z = (float) calcWorld.at<double>(2, 0);
            point.id = input->data[i].id;
            point.conf = input->data[i].conf;
            point.tracker_id = input->data[i].tracker_id;
            close_points.data.push_back(point);
        }
    }
}

void SmallMap::Pnp_resultCallback(rclcpp::Client<radar_interfaces::srv::PnpResult>::SharedFuture response) {
    auto& result = response.get();
    for (int i = 0; i < 3; i++) {
        far_T.at<double>(i) = result->far_t[i];
        close_T.at<double>(i) = result->close_t[i];
        for (int j = 0; j < 3; j++) {
            far_CamMatrix_.at<double>(i, j) = result->far_cam_matrix[3 * i + j];
            far_R.at<double>(i, j) = result->far_r[3 * i + j];
            close_CamMatrix_.at<double>(i, j) = result->close_cam_matrix[3 * i + j];
            close_R.at<double>(i, j) = result->close_r[3 * i + j];
        }
    }
    cout << "pnp result if ok?: " << result->if_ok;
    cout << endl << "far R matrix load done!" << endl << far_R << endl;
    cout << endl << "far T matrix load done!" << endl << far_T << endl;
    cout << endl << "close R matrix load done!" << endl << close_R << endl;
    cout << endl << "close T matrix load done!" << endl << close_T << endl;
    cout << endl << "far_Camera matrix load done!" << endl << far_CamMatrix_ << endl;
    cout << endl << "close_Camera matrix load done!" << endl << close_CamMatrix_ << endl;

    cv::invert(far_CamMatrix_, far_invM);
    cv::invert(far_R, far_invR);
    cv::invert(close_CamMatrix_, close_invM);
    cv::invert(close_R, close_invR);
}

void SmallMap::pickup_infoCallback(radar_interfaces::msg::Point::SharedPtr msg) {
    if (msg->id == 66) {
        pnp_request->small_map_qidong = true;
        RCLCPP_INFO(this->get_logger(), "utilities-pickup operated: %d", pnp_request->small_map_qidong);
        //发送异步请求，然后等待返回，返回时调用回调函数
        Pnp_result_client_->async_send_request(pnp_request, std::bind(&SmallMap::Pnp_resultCallback, this, _1));
        pnp_request->small_map_qidong = false;
    }
}

void SmallMap::add_grid(cv::Mat &src) {
    for (int i = 1; i < 14; i++) {
        cv::line(src, cv::Point(30 * i, 0), cv::Point(30 * i, img_show_height), color_table["Orange"], 1);
    }
    for (int i = 1; i < 24; i++) {
        cv::line(src, cv::Point(0, 30 * i), cv::Point(img_show_width, 30 * i), color_table["Orange"], 1);
    }
}

void SmallMap::draw_point_on_map(const radar_interfaces::msg::Point &point, Mat &image, const string& txt_color) {
    Scalar color;
    string id;
    if (point.id <= 5 || point.id == 12) color = color_table["Red"];
    else color = color_table["Blue"];
    circle(image, calculate_pixel_codi(point), 10,
           color, -1, LINE_8, 0);
    if (point.id != 12 && point.id != 13) {
        if (point.id <= 5) id = to_string(point.id + 1);
        if (point.id == 5) id = "G";
        if (point.id >= 6) id = to_string(point.id - 5);
        if (point.id == 11) id = "G";
        putText(image, id, calculate_pixel_text_codi(point), cv::FONT_HERSHEY_SIMPLEX,
                0.7, color_table[txt_color], 2);

    }
}

void SmallMap::remove_duplicate() {
    vector<radar_interfaces::msg::Point>().swap(result_points.data);
    std::vector<radar_interfaces::msg::Point>::iterator far_target_iter, close_target_iter;
    bool far_erase_flag = false, close_erase_flag = false;
    int loop_count = 0, loop_max = 7;

    while(rclcpp::ok()) { //把far中已知的点与close匹配并去掉
        auto iter_far = far_points.data.begin();
        for (iter_far = far_points.data.begin(); iter_far!=far_points.data.end(); iter_far++) {
            if (far_erase_flag || close_erase_flag) break;
            if (iter_far->id >= 12) continue;
            for (auto iter_close = close_points.data.begin(); iter_close!=close_points.data.end(); iter_close++) {
                if (far_erase_flag || close_erase_flag) break;
                if (iter_far->id == iter_close->id && iter_far->id < 12 && iter_close->id < 12
                    && calculate_dist(*iter_far, *iter_close) < dist_threshold) {
                    result_points.data.emplace_back(*iter_far); //far较准确，所以以far为基准
                    far_target_iter = iter_far;
                    close_target_iter = iter_close;
                    far_erase_flag = true;
                    close_erase_flag = true;
                }
            }
        }
        if (far_erase_flag) {
            far_points.data.erase(far_target_iter);
            far_erase_flag = false;
        }
        if (close_erase_flag) {
            close_points.data.erase(close_target_iter);
            close_erase_flag = false;
        }
        if (iter_far == far_points.data.end()) break;
    }

    for (auto i : close_points.data) {
        if (i.id < 12) {
            //TODO 如果close全为未知，则...
        }
    }

    while (rclcpp::ok()) {
        if (loop_count++ > loop_max || far_points.data.empty()) break;
        std::vector<std::pair<double, std::vector<radar_interfaces::msg::Point>::iterator>> sorted_points;
        auto pf_iter = far_points.data.begin();
        for (auto pc_iter = close_points.data.begin(); pc_iter < close_points.data.end(); pc_iter++) {
            double dist_m = calculate_dist(*pf_iter, *pc_iter);
            sorted_points.emplace_back(
                    dist_m, std::vector<radar_interfaces::msg::Point>::iterator(pc_iter));
        }
        std::sort(sorted_points.begin(), sorted_points.end(),
                  [](const std::pair<double, std::vector<radar_interfaces::msg::Point>::iterator> &a,
                     const std::pair<double, std::vector<radar_interfaces::msg::Point>::iterator> &b) {
                      return a.first < b.first;
                  });
        for (auto i : sorted_points) {
            if (calculate_dist(*pf_iter, *i.second) < dist_threshold) {
                if ((pf_iter->id == 12 && i.second->id < 6)
                || (pf_iter->id == 13 && (i.second->id >= 6 && i.second->id < 12))) {
                    pf_iter->id = i.second->id;
                    result_points.data.emplace_back(*pf_iter);
                    far_target_iter = pf_iter;
                    close_target_iter = i.second;
                    far_erase_flag = true;
                    close_erase_flag = true;
                    break;
                }
//                } else if ((pf_iter->id == 12 && i.second->id == 12) || (pf_iter->id == 13 && i.second->id == 13)) {
//                    result_points.data.emplace_back(*pf_iter);
//                    far_target_iter = pf_iter;
//                    close_target_iter = i.second;
//                    far_erase_flag = true;
//                    close_erase_flag = true;
//                    break;
//                }
            }
        }
        if (far_erase_flag) {
            far_points.data.erase(far_target_iter);
            far_erase_flag = false;
        }
        if (close_erase_flag) {
            close_points.data.erase(close_target_iter);
            close_erase_flag = false;
        }
    }
    for (auto i : far_points.data) {
        if (i.id < 12) result_points.data.push_back(i);
    }
    for (auto i : close_points.data) {
        if (i.id < 12) result_points.data.push_back(i);
    }
//    std::cout << "result_points size: " << result_points.data.size() << endl;
}

radar_interfaces::msg::Point SmallMap::calculate_relative_codi(const Point3f &guard,
                                                               const radar_interfaces::msg::Point &enemy,
                                                               uint8_t priority_id) {
    radar_interfaces::msg::Point re_codi;
    re_codi.x = enemy.x * field_height - guard.x;
    re_codi.y = enemy.y * field_width - guard.y;
    re_codi.z = enemy.z * 1000 - guard.z;
    re_codi.id = priority_id;
    return re_codi;
}

/*
 * 把result_points 里面归一化的点转换成小地图(450*840)图像中的像素座标
 */
Point2d SmallMap::calculate_pixel_codi(const radar_interfaces::msg::Point &point) {
    Point2d res;
    res.x = (1 - point.y) * img_show_width - X_shift;
    res.y = (1 - point.x) * img_show_height - Y_shift;
    return res;
}

Point2d SmallMap::calculate_pixel_text_codi(const radar_interfaces::msg::Point &point) {
    Point2d res;
    res.x = (1 - point.y) * img_show_width - X_shift - 7;
    res.y = (1 - point.x) * img_show_height - Y_shift + 7;
    return res;
}

double SmallMap::Point2PointDist(const radar_interfaces::msg::Point &a, const Point2d &b) {
    double res = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return res;
}

double SmallMap::calculate_dist(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b) {
    cv::Point2d p1(a.x * field_height, a.y * field_width), p2(b.x * field_height, b.y * field_width);
    double res = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    return res;
}

bool SmallMap::is_enemy_car(uint8_t id) {
    if (red_or_blue == 0) {
        return ((id >= 6 && id <= 11) || id == 13);
    } else {
        return (id <= 5 || id == 12);
    }
}

bool SmallMap::check_same_color(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b) {
    return (
            ((a.id < 6 || a.id == 12) && (b.id < 6 || b.id == 12)) ||
    (((b.id >= 6 && b.id < 12) || b.id == 13) && ((a.id >= 6 && a.id < 12) || a.id == 13))
    );
}

void SmallMap::load_param() {
    this->declare_parameter<int>("small_map_params.small_map_shift_X", 0);
    this->declare_parameter<int>("small_map_params.small_map_shift_Y", 0);
    this->declare_parameter<std::string>("battle_state.our_color", "empty");
    this->declare_parameter<double>("small_map_params.dist_threshold", 0.0);
    this->declare_parameter<double>("small_map_params.field_width", 0.0);
    this->declare_parameter<double>("small_map_params.field_height", 0.0);
    this->declare_parameter<double>("small_map_params.imgCols", 0.0);
    this->declare_parameter<double>("small_map_params.imgRows", 0.0);
    this->declare_parameter<float>("small_map_params.center_far.x", 0.0);
    this->declare_parameter<float>("small_map_params.center_far.y", 0.0);
    this->declare_parameter<float>("small_map_params.center_close.x", 0.0);
    this->declare_parameter<float>("small_map_params.center_close.y", 0.0);

    X_shift = this->get_parameter("small_map_params.small_map_shift_X").as_int();  // =30
    Y_shift = this->get_parameter("small_map_params.small_map_shift_Y").as_int();  // =5
    string btlcolor = this->get_parameter("battle_state.our_color").as_string();
    dist_threshold = this->get_parameter("small_map_params.dist_threshold").as_double();  // =1.6
    field_width = this->get_parameter("small_map_params.field_width").as_double();  // =14.0
    field_height = this->get_parameter("small_map_params.field_height").as_double();  // =15.0
    imgCols = this->get_parameter("small_map_params.imgCols").as_double();  // =1920.0
    imgRows = this->get_parameter("small_map_params.imgRows").as_double();  // =1200.0
    center_far.x = this->get_parameter("small_map_params.center_far.x").as_double();  // =0.0
    center_far.y = this->get_parameter("small_map_params.center_far.y").as_double();  // =0.0
    center_close.x = this->get_parameter("small_map_params.center_close.x").as_double();  // =0.0
    center_close.y = this->get_parameter("small_map_params.center_close.y").as_double();  // =0.0

    RCLCPP_INFO(this->get_logger(), "Load X_shift[%d], Y_shift[%d], our_color[%s]", X_shift, Y_shift, btlcolor.c_str());
    RCLCPP_INFO(this->get_logger(), "field_width[%f], field_height[%f]", field_width, field_height);
    RCLCPP_INFO(this->get_logger(), "imgCols[%f], imgRows[%f]", imgCols, imgRows);
    RCLCPP_INFO(this->get_logger(), "dist_threshold[%f]", dist_threshold);
    RCLCPP_INFO(this->get_logger(), "center_far[%f, %f]", center_far.x, center_far.y);
    RCLCPP_INFO(this->get_logger(), "center_close[%f, %f]", center_close.x, center_close.y);
    if (btlcolor == "red") red_or_blue = 0;
    else if (btlcolor == "blue") red_or_blue = 1;
    small_map_png_path = "/home/hlf/Desktop/radar24_ws/src/small_map/images_24/red_smallmap.png";
}
