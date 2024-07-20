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
    Icp_result_subscription_ = this->create_subscription<radar_interfaces::msg::PnpResult>(
            "icp_result", 1, std::bind(&SmallMap::Icp_resultCallback, this, _1));
    timer_ = this->create_wall_timer(25ms, std::bind(&SmallMap::TimerCallback, this));
    world_point_publisher_ = this->create_publisher<radar_interfaces::msg::Points>("/world_point", 10);
    Pnp_result_client_ = this->create_client<radar_interfaces::srv::PnpResult>("pnp_results");

    this->load_param();
    if (red_or_blue == 0) {
        std::string small_map_png =
                std::string("/home/hlf/Desktop/radar24_ws/src/small_map/images_24") + "/red_smallmap.png";
        small_map = cv::imread(small_map_png);
    } else {
        std::string small_map_png =
                std::string("/home/hlf/Desktop/radar24_ws/src/small_map/images_24") + "/blue_smallmap.png";
        small_map = cv::imread(small_map_png);
    }
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
}

SmallMap::~SmallMap() {
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "small map node shutdown!!!");
}

void SmallMap::TimerCallback() {
    small_map.copyTo(small_map_copy);
    remove_duplicate();
    for (auto &i: result_points.data) {
        draw_point_on_map(i, small_map_copy);
    }
//    vector<radar_interfaces::msg::Point>().swap(filtered_result_points.data);
//    //根据裁判系统的标记进度，确定哪些tracker_id需要锁定
//    for (auto &i: result_points.data) {
//        if (is_enemy_car(i.id)) {
//            if (red_or_blue == 0 && i.id <= 5) {
//                filtered_result_points.data.emplace_back(i);
//                // TODO : cancel debug out
//                std::cout << "id : " << (int) i.id << ", [ " << i.x << " , " << i.y << " , " << i.z << " ]" << std::endl;
//                // TODO : end at here
//            } else if (red_or_blue == 1 && i.id >= 6 && i.id <= 11) { // id >= 6 || id == 13
//                filtered_result_points.data.emplace_back(i);
//                // TODO : cancel debug out
//                std::cout << "id : " << (int) i.id << ", [ " << i.x << " , " << i.y << " , " << i.z << " ]" << std::endl;
//                // TODO : end at here
//            }
//        }
//        draw_point_on_map(i, small_map_copy);
//    }
    this->world_point_publisher_->publish(result_points);

    if (mouse_click_flag) {
        mouse_click_flag = false;
        std::string toshow = "at : " + to_string(mouse_point.x) + " , " + to_string(mouse_point.y);
        cv::putText(small_map_copy, toshow, cv::Point(10, 400), cv::FONT_HERSHEY_SIMPLEX, 0.7, color_table["Red"], 2);
    }

    cv::imshow("small_map", small_map_copy);
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
            double x = calcWorld.at<double>(0, 0) / field_width;
            double y = calcWorld.at<double>(1, 0) / field_height;
            radar_interfaces::msg::Point point;
            point.x = (float) x;
            point.y = (float) y;
            point.z = (float) calcWorld.at<double>(2, 0);
            point.id = input->data[i].id;
            point.conf = input->data[i].conf;
            point.tracker_id = input->data[i].tracker_id;
            if (point.id == 66) {
                mouse_click_flag = true;
                mouse_point = Point2f(x * field_width, y * field_height);
            }
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
            double x = calcWorld.at<double>(0, 0) / field_width;
            double y = calcWorld.at<double>(1, 0) / field_height;
            radar_interfaces::msg::Point point;
            point.x = (float) x;
            point.y = (float) y;
            point.z = (float) calcWorld.at<double>(2, 0);
            point.id = input->data[i].id;
            point.conf = input->data[i].conf;
            point.tracker_id = input->data[i].tracker_id;
//            std::cout << x << " : " << y << " id: " << point.id << endl;
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

void SmallMap::Icp_resultCallback(radar_interfaces::msg::PnpResult::SharedPtr msg) {
    for (int i = 0; i < 3; i++) {
        far_T.at<double>(i) = msg->far_t[i];
        close_T.at<double>(i) = msg->close_t[i];
        for (int j = 0; j < 3; j++) {
            far_R.at<double>(i, j) = msg->far_r[3 * i + j];
            close_R.at<double>(i, j) = msg->close_r[3 * i + j];
        }
    }
    cout << "icp result received" << endl;
    cout << endl << "far R matrix load done!" << endl << far_R << endl;
    cout << endl << "far T matrix load done!" << endl << far_T << endl;
    cout << endl << "close R matrix load done!" << endl << close_R << endl;
    cout << endl << "close T matrix load done!" << endl << close_T << endl;
}

void SmallMap::add_grid(cv::Mat &src) {
    for (int i = 1; i < 14; i++) {
        cv::line(src, cv::Point(30 * i, 0), cv::Point(30 * i, img_show_height), color_table["Orange"], 1);
    }
    for (int i = 1; i < 24; i++) {
        cv::line(src, cv::Point(0, 30 * i), cv::Point(img_show_width, 30 * i), color_table["Orange"], 1);
    }
}

void SmallMap::draw_point_on_map(const radar_interfaces::msg::Point &point, Mat &image) {
    Scalar color;
    string id;
    if (point.id <= 5 || point.id == 12) color = color_table["Red"];//Scalar(0, 0, 255);
    else color = color_table["Blue"];
    circle(image, calculate_pixel_codi(point), 10,
           color, -1, LINE_8, 0);
    if (point.id != 12 && point.id != 13) {
        if (point.id <= 5) id = to_string(point.id + 1);
        if (point.id == 5) id = "G";
        if (point.id >= 6) id = to_string(point.id - 5);
        if (point.id == 11) id = "G";
        putText(image, id, calculate_pixel_text_codi(point), cv::FONT_HERSHEY_SIMPLEX,
                0.7, color_table["Withe"], 2);
    }
}

void SmallMap::remove_duplicate() {vector<radar_interfaces::msg::Point>().swap(result_points.data);
//    std::vector<std::vector<radar_interfaces::msg::Point>::iterator> far_remove_list, close_remove_list ;
//    for (auto pf_iter = far_points.data.begin(); pf_iter < far_points.data.end(); pf_iter++) {
//        for (auto pc_iter = close_points.data.begin(); pc_iter < close_points.data.end(); pc_iter++) {
//            if (pf_iter->id == pc_iter->id && pc_iter->id < 12 && pf_iter->id < 12) { // 去除两侧同序号的点
//                double dist_m = calculate_dist(*pf_iter, *pc_iter);
//                radar_interfaces::msg::Point center;
//                if (dist_m < dist_threshold) {
//                    center.id = pf_iter->id;
//                    center.x = (pf_iter->x + pc_iter->x) / 2;
//                    center.y = (pf_iter->y + pf_iter->y) / 2;
//                } else {
//                    if (pf_iter->conf > pc_iter->conf) {
//                        center.id = pf_iter->id;
//                        center.x = pf_iter->x;
//                        center.y = pf_iter->y;
//                    } else {
//                        center.id = pc_iter->id;
//                        center.x = pc_iter->x;
//                        center.y = pc_iter->y;
//                    }
//                }
//                result_points.data.emplace_back(center);
//                far_remove_list.push_back(pf_iter);
//                close_remove_list.push_back(pc_iter);
//                break;
//            }
//        }
//    }
//    if (!far_remove_list.empty() && !close_remove_list.empty()) {
//        for (auto i : far_remove_list) {
//            far_points.data.erase(i);
//        }
//        for (auto j : close_remove_list) {
//            close_points.data.erase(j);
//        }
//    }
    for (auto i : far_points.data) {
        result_points.data.push_back(i);
    }
    for (auto j : close_points.data) {
        result_points.data.push_back(j);
    }


//    radar_interfaces::msg::Points left_may_overlap_points;
//    radar_interfaces::msg::Points right_may_overlap_points;
//    for (auto &i: far_points.data) {
//        double test = pointPolygonTest(left_region, calculate_pixel_codi(i), false);
//        double test2 = pointPolygonTest(right_region, calculate_pixel_codi(i), false);
//        if (test > 0 && test2 < 0) {
//            result_points.data.emplace_back(i);
//        } else if (test == 0 && i.x != 200) {
//            result_points.data.emplace_back(i);
//        } else if (test2 <= 0) {
//            left_may_overlap_points.data.emplace_back(i);
//        }
//    }
//    for (auto &i: close_points.data) {
//        double test = pointPolygonTest(right_region, calculate_pixel_codi(i), false);
//        double test2 = pointPolygonTest(left_region, calculate_pixel_codi(i), false);
//        if (test > 0) {
//            result_points.data.emplace_back(i);
//        } else if (test == 0 && i.x != 255) {
//            result_points.data.emplace_back(i);
//        } else if (test2 <= 0) {
//            right_may_overlap_points.data.emplace_back(i);
//        }
//    }

    //    for (auto it_left_a = left_may_overlap_points.data.begin(); it_left < left_may_overlap_points.data.end();) {
//        for (auto it_left_b = right_may_overlap_points.data.begin(); it_right < right_may_overlap_points.data.end();) {
//            if (it_left->id == it_right->id && it_left->id == 12 && calculate_dist(*it_left, *it_right) < 5) {
//                radar_interfaces::msg::Point center;
//                center.id = 12;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty()) left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty()) right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            } else if (it_left->id == it_right->id && it_left->id == 13 && calculate_dist(*it_left, *it_right) < 5) {
//                radar_interfaces::msg::Point center;
//                center.id = 13;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            } else if (it_left->id == it_right->id && it_left->id < 12 && it_left->id > 0) {
//                radar_interfaces::msg::Point center;
//                center.id = it_left->id;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            }
//            if (inner_erase_flag == 1) {
//                inner_erase_flag = 0;
//                continue;
//            }
//            it_right++;
//        }
//        if (outer_erase_flag == 1) {
//            outer_erase_flag = 0;
//            continue;
//        }
//        it_left++;
//    }

//    uint8_t inner_erase_flag = 0;
//    uint8_t outer_erase_flag = 0;
//    for (auto it_left = left_may_overlap_points.data.begin(); it_left < left_may_overlap_points.data.end();) {
//        for (auto it_right = right_may_overlap_points.data.begin(); it_right < right_may_overlap_points.data.end();) {
//            if (it_left->id == it_right->id && it_left->id == 12 && calculate_dist(*it_left, *it_right) < 5) {
//                radar_interfaces::msg::Point center;
//                center.id = 12;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty()) left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty()) right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            } else if (it_left->id == it_right->id && it_left->id == 13 && calculate_dist(*it_left, *it_right) < 5) {
//                radar_interfaces::msg::Point center;
//                center.id = 13;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            } else if (it_left->id == it_right->id && it_left->id < 12 && it_left->id > 0) {
//                radar_interfaces::msg::Point center;
//                center.id = it_left->id;
//                center.x = (it_left->x + it_right->x) / 2;
//                center.y = (it_left->y + it_right->y) / 2;
//                result_points.data.emplace_back(center);
//                if (!left_may_overlap_points.data.empty())left_may_overlap_points.data.erase(it_left);
//                if (!right_may_overlap_points.data.empty())right_may_overlap_points.data.erase(it_right);
//                inner_erase_flag = 1;
//                outer_erase_flag = 1;
//            }
//            if (inner_erase_flag == 1) {
//                inner_erase_flag = 0;
//                continue;
//            }
//            it_right++;
//        }
//        if (outer_erase_flag == 1) {
//            outer_erase_flag = 0;
//            continue;
//        }
//        it_left++;
//    }
//    for (auto &i: left_may_overlap_points.data) {
//        result_points.data.emplace_back(i);
//    }
//    for (auto &i: right_may_overlap_points.data) {
//        result_points.data.emplace_back(i);
//    }
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
Point2f SmallMap::calculate_pixel_codi(const radar_interfaces::msg::Point &point) {
    Point2f res;
    res.x = point.x * img_show_width - (float) X_shift;
    res.y = (1 - point.y) * img_show_height - (float) Y_shift;
    return res;
}

Point2f SmallMap::calculate_pixel_text_codi(const radar_interfaces::msg::Point &point) {
    Point2f res;
    res.x = point.x * img_show_width - (float) X_shift - 7;
    res.y = (1 - point.y) * img_show_height - (float) Y_shift + 7;
    return res;
}

double SmallMap::Point2PointDist(const radar_interfaces::msg::Point &a, const Point3f &b) {
    double res = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return res;
}

double SmallMap::calculate_dist(const radar_interfaces::msg::Point &a, const radar_interfaces::msg::Point &b) {
    cv::Point2d p1(a.x * field_width, a.y * field_height), p2(b.x * field_width, b.y * field_height);
    double res = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    return res;
}

bool SmallMap::is_enemy_car(uint8_t id) {
    if (red_or_blue == 0) {
        if ((id >= 6 && id <= 11) || id == 13)
            return true;
        else return false;
    } else {
        if (id <= 5 || id == 12)
            return true;
        else return false;
    }
}

void SmallMap::load_param() {
//    this->declare_parameter<int>("small_map_params.small_map_shift_X", 0);
//    this->declare_parameter<int>("small_map_params.small_map_shift_Y", 0);
//    this->declare_parameter<std::string>("battle_state.battle_color", "empty");

//    X_shift = this->get_parameter("small_map_params.small_map_shift_X").as_int();  // =30
//    Y_shift = this->get_parameter("small_map_params.small_map_shift_Y").as_int();  // =5
//    string btlcolor = this->get_parameter("battle_state.battle_color").as_string();
    X_shift = 0;
    Y_shift = 0;
    string btlcolor = "blue";
    cout << endl << "Load X_shift, Y_shift, red_or_blue : " << endl;
    cout << "\t" << X_shift << "\t" << Y_shift << "\t" << btlcolor << endl;
    if (btlcolor == "red") red_or_blue = 0;
    else if (btlcolor == "blue") red_or_blue = 1;
}