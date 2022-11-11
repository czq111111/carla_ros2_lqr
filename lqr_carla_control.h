#pragma once
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <stdint.h>

#include "read_data.h"
#include "refercenceline_smooth.h"
#include "matching_line.h"
#include "reference_line.h"
#include "extract_point.h"
#include "LQR_control.h"
#include "lqr_control.h"
#include "struct_common.h"

#include <time.h>
#include <sstream>


#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>

using std::placeholders::_1;
using namespace std;

stringstream ss;
int tim1[3]={0};
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1;
// rclcpp::Node::SharedPtr node_handle = nullptr;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();
    ~ControlNode();
    double PointDistance(const plan_end_info &point, const double x, const double y);
    // 声明定时器：车辆控制器信号定时调用 全局路径广播定时 重规划算法定时调用
    rclcpp::TimerBase::SharedPtr vehicle_control_iteration_timer;
    void VehicleControllerIterationCallback();
     // 声明定时器：车辆控制器信号定时调用 全局路径广播定时 平滑算法定时调用
    rclcpp::TimerBase::SharedPtr smooth_timer;
    void SmoothCallback();

    // 订阅器以及订阅回调函数和广播消息类型
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_data_subscriber;
    void OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    // // 创建仿真时间的订阅者
    // rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_data_subscriber;
    // void ClockCallback(rosgraph_msgs::msg::Clock::SharedPtr msg);

    //创建两个定时器
	rclcpp::TimerBase::SharedPtr timer1,timer2;
    void timer1_callback();
    void timer2_callback();
    

    // 声明广播器：控制信号广播 历史路径广播 全局路径广播 重规划路径广播
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr vehicle_control_publisher;
    carla_msgs::msg::CarlaEgoVehicleControl control_cmd;

    std::unique_ptr<LQR_control> lqr_controller_lateral;
    std::unique_ptr<lqr_control> sl_lqr_controller_lateral;

    VehicleState vehicle_state;//车辆状态信息
    
    bool first_record;
    double v_set;
    double wheelbase;
    
    location_info ego_info;//车辆位置信息

    std::vector<planning_trajectory> Trajectory_; //lqr要跟踪的轨迹
    std::vector<xy_points_info> glob_xy_points;

    std::vector<std::pair<double, double>> xy_points;//全局坐标
    std::vector<double> headings, accumulated_s, kappas, dkappas;//全局heading，kappas

    std::vector<double> x0, y0;
    std::vector<double> x_smooth, y_smooth;
    
    refercenceline_smooth test;

    std::vector<std::pair<double, double>> smooth_points;

    double controlFrequency_ = 100;    //控制频率
    double plannerFrequency_ = 10;     //规划频率
};