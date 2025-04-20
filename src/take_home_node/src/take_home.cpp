#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>
#include <cfloat>          
#include <mutex>            
#include <deque>            
#include <vector>           
#include <utility>          
#include <chrono>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options){
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)

    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "vehicle/uva_odometry", qos_profile,
        std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));
    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "/raptor_dbw_interface/wheel_speed_report", qos_profile,
        std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));
    steering_report_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
        "/raptor_dbw_interface/steering_extended_report", qos_profile,
        std::bind(&TakeHome::steering_report_callback, this, std::placeholders::_1));
    imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "/novatel_top/rawimu", qos_profile,
        std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));
    curvilinear_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "curvilinear_distance", qos_profile,
        std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));
    slip_rr_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_rl_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_fl_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
    slip_fr_publisher = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    jitter_publisher = this->create_publisher<std_msgs::msg::Float64>("imu_top/jitter", qos_profile);
    lap_time_publisher = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(11111),
        std::bind(&TakeHome::interpolate, this)
      );
      reset_timer();
}

/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::reset_timer() {
    start_time_ = this->now();
}
double TakeHome::get_time(){
    return (this->now() - start_time_).seconds();
}
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg){
    std::lock_guard<std::mutex> lk(mutex_);
    odom_data = odom_msg;
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg){
    std::lock_guard<std::mutex> lk(mutex_);
    wheel_speed_data = wheel_speed_msg;
}

void TakeHome::steering_report_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_extended_msg){
    std::lock_guard<std::mutex> lk(mutex_);
    steering_extended_data = steering_extended_msg;
}
void TakeHome::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg){
    std::lock_guard<std::mutex> lk(mutex_2);
    double cur_time = imu_msg->gnss_seconds;
    reset_timer();
    if(last_time!=-1){
        q.push_front({cur_time-last_time, last_time});
        //RCLCPP_INFO(this->get_logger(), "∆t: %.6f (cur_time: %.6f, last_time: %.6f)", cur_time - last_time, cur_time, last_time);
    }
    last_time = cur_time;
}
void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_distance_msg){
    double time = last_time + get_time();
    float p = curvilinear_distance_msg->data;
    if(p==0){
        if(last_lap_timestamp == -1.0){
            last_lap_timestamp = time;
            at_line = true;
        } else{
            if(!at_line){
                at_line = true;
                last_lap_time = time-last_lap_timestamp;
                last_lap_timestamp = time;
            }
        }
    } else{
        at_line = false;
    }
    std_msgs::msg::Float32 lap_time_msg;
    lap_time_msg.data = last_lap_time;
    lap_time_publisher->publish(lap_time_msg);
}
float op(float a, float b){
    float res = (a-b)/b;
    if(std::isnan(res) || std::isinf(res)){
        if(a-b>0){
            return FLT_MAX;
        } else{
            return FLT_MIN;
        }
    }
    return res;
}
double get_std(std::deque<std::pair<double,double>> q){
    int num = q.size();
    if(num<2){
        return -1.0;
    }
    std::deque<std::pair<double,double>> q_copy = q;
    std::vector<double> vals;
    double mean = 0;
    while(q_copy.size()){
        vals.push_back(q_copy.front().first);
        mean+=q_copy.front().first;
        q_copy.pop_front();
    }
    mean/=num;
    double std_dev = 0;
    for(double value : vals){
        double res = value-mean;
        std_dev+=(res*res);
    }
    std_dev/=(num-1);
    return std_dev;
}
void TakeHome::interpolate(){
    std::lock_guard<std::mutex> lk(mutex_); //queue danger if slow, karth said fine tho?
    std::lock_guard<std::mutex> lk2(mutex_2); //queue danger if slow, karth said fine tho?
    while(q.size() && last_time - q.back().second > 1){
        q.pop_back();
    }
    float std = get_std(q);
    if(std!=-1.0){
        std_msgs::msg::Float64 jitter_msg;
        jitter_msg.data = std;
        jitter_publisher->publish(jitter_msg);
    }
    if(!odom_data || !wheel_speed_data || !steering_extended_data) return;
    double v_x = odom_data->twist.twist.linear.x;
    double v_y = odom_data->twist.twist.linear.y;
    float yaw_rate = odom_data->twist.twist.angular.z;
    float w_r = 1.523;
    float v_x_rr = v_x - 0.5*yaw_rate*w_r;
    float v_w_rr = (wheel_speed_data->rear_right)/3.6;
    float kappa_rr = op(v_w_rr,v_x_rr);
    float v_x_rl = v_x + 0.5*yaw_rate*w_r;
    float v_w_rl = (wheel_speed_data->rear_left)/3.6;
    float kappa_rl = op(v_w_rl,v_x_rl);
    float w_f = 1.638;
    float l_f = 1.7238;
    float steering_ratio = (steering_extended_data->primary_steering_angle_fbk)  * M_PI / 180.0f;
    float delta = steering_ratio/15.0;
    float v_x_fr = v_x-0.5*yaw_rate*w_f;
    float v_y_fr = v_y+yaw_rate*l_f;
    float v_x_fr_delta = std::cos(delta)*v_x_fr - std::sin(delta)*v_y_fr;
    float v_w_fr = (wheel_speed_data->front_right)/3.6;
    float kappa_fr = op(v_w_fr,v_x_fr_delta);
    float v_x_fl = v_x+0.5*yaw_rate*w_f;
    float v_y_fl = v_y+yaw_rate*l_f;
    float v_x_fl_delta = std::cos(delta)*v_x_fl - std::sin(delta)*v_y_fl;
    float v_w_fl = (wheel_speed_data->front_left)/3.6;
    float kappa_fl = op(v_w_fl,v_x_fl_delta);
    //publish
    std_msgs::msg::Float32 slip_rr_msg;
    slip_rr_msg.data = kappa_rr;
    slip_rr_publisher->publish(slip_rr_msg);
    std_msgs::msg::Float32 slip_rl_msg;
    slip_rl_msg.data = kappa_rl;
    slip_rl_publisher->publish(slip_rl_msg);
    std_msgs::msg::Float32 slip_fr_msg;
    slip_fr_msg.data = kappa_fr;
    slip_fr_publisher->publish(slip_fr_msg);
    std_msgs::msg::Float32 slip_fl_msg;
    slip_fl_msg.data = kappa_fl;
    slip_fl_publisher->publish(slip_fl_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
