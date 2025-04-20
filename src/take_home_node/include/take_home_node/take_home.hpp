#pragma once

#include <mutex>                                  
#include <deque>                                 
#include <utility>
#include <cmath>
#include <cfloat>                                

#include <rclcpp/rclcpp.hpp>                      
#include <std_msgs/msg/float32.hpp>               
#include <std_msgs/msg/float64.hpp>              
#include <nav_msgs/msg/odometry.hpp>             
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>   
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>

class TakeHome : public rclcpp::Node {
public:
  TakeHome(const rclcpp::NodeOptions& options);

private:
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg);
  void steering_report_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_extended_msg);
  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);
  void curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_distance_msg);
  void interpolate();
  double get_time();
  void reset_timer();
  std::deque<std::pair<double,double>> q;
  double last_time = -1.0;
  float last_lap_time = 0.0, last_lap_timestamp = -1.0;
  float eps = 1e-9;
  bool at_line = false;
  std::mutex mutex_;
  std::mutex mutex_2;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  nav_msgs::msg::Odometry::ConstSharedPtr odom_data;
  raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_data;
  raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_extended_data;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_report_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_distance_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr jitter_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher;
};
