#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace tractor_control_adapter {

struct FeedforwardModel {
  double coast0 = 0.0;
  double coast1 = 0.0;
  double throttle_gain = 0.0;
  double brake_gain = 0.0;

  double coast(double speed) const;
};

class TractorControlNode final : public rclcpp::Node {
 public:
  TractorControlNode();
  void stop();

 private:
  void publish(double accelerator, double brake, double brake1, double steer);
  void debug(double desired_accel, double throttle, double brake);
  void tick();

  std::string cmd_topic_, feedback_topic_, actuation_topic_, base_frame_;
  double rate_, kp_, ki_, wheelbase_, front_track_width_, max_steer_, max_curvature_;
  double accel_limit_, decel_limit_, max_speed_, stop_brake_, max_rolling_brake_, timeout_;
  double target_ = 0.0, yaw_rate_ = 0.0, speed_ = 0.0;
  double setpoint_ = 0.0, integral_ = 0.0;
  int gear_ = 1;
  FeedforwardModel forward_model_, reverse_model_;
  bool cmd_valid_ = false, feedback_valid_ = false;
  // ROS clock nanoseconds: simulated time when use_sim_time is enabled.
  int64_t cmd_time_ns_ = -1, feedback_time_ns_ = -1, last_tick_ns_ = -1;
  int64_t stationary_since_ns_ = -1, shift_until_ns_ = -1;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_, debug_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tractor_control_adapter
