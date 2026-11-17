#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

namespace {
// Float32MultiArray contract: accelerator, service brake, parking brake,
// normalized steering, target gear. Teleop confirms forward gear is 1.
constexpr size_t kAccelerator = 0, kBrake0 = 1, kBrake1 = 2, kSteer = 3, kGear = 4;
double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
struct Model {
  double coast0 = 0, coast1 = 0, throttle_gain = 0, brake_gain = 0;
  double coast(double speed) const { return coast0 + coast1 * clamp(speed, 0, 2); }
};
Model parse_model(const std::string &line, int expected_gear) {
  std::stringstream stream(line);
  std::string cell;
  double values[4];
  if (!std::getline(stream, cell, ',') || std::stoi(cell) != expected_gear)
    throw std::runtime_error("Feedforward model gear mismatch");
  for (double &value : values) {
    if (!std::getline(stream, cell, ',')) throw std::runtime_error("Incomplete feedforward model row");
    value = std::stod(cell);
    if (!std::isfinite(value)) throw std::runtime_error("Nonfinite feedforward coefficient");
  }
  if (values[2] <= 0 || values[3] <= 0) throw std::runtime_error("Nonpositive pedal gain");
  return Model{values[0], values[1], values[2], values[3]};
}
}  // namespace

class TractorControlNode : public rclcpp::Node {
 public:
  TractorControlNode() : Node("tractor_control_node") {
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "/ground_truth/odom");
    actuation_topic_ = declare_parameter<std::string>("actuation_topic", "/vehicle/actuation");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    rate_ = declare_parameter<double>("control_rate_hz", 20.0);
    kp_ = declare_parameter<double>("kp", 1.1);
    ki_ = declare_parameter<double>("ki", 0.15);
    wheelbase_ = declare_parameter<double>("wheelbase", 2.7);
    max_steer_ = declare_parameter<double>("max_steer_angle_rad", 0.7853981633974483);
    max_curvature_ = declare_parameter<double>("max_curvature", 0.37);
    accel_limit_ = declare_parameter<double>("command_accel_limit", 0.7);
    decel_limit_ = declare_parameter<double>("command_decel_limit", 0.7);
    max_speed_ = declare_parameter<double>("max_speed", 2.0);
    stop_brake_ = declare_parameter<double>("stop_brake", 0.12);
    max_rolling_brake_ = declare_parameter<double>("max_rolling_brake", 0.12);
    timeout_ = declare_parameter<double>("timeout_sec", 0.5);
    auto model_path = declare_parameter<std::string>("model_path", "");
    if (model_path.empty()) {
      model_path = ament_index_cpp::get_package_share_directory("tractor_control_adapter") + "/config/road.csv";
    }
    if (rate_ <= 0 || kp_ < 0 || ki_ < 0 || wheelbase_ <= 0 || max_steer_ <= 0 ||
        max_curvature_ <= 0 || accel_limit_ <= 0 || decel_limit_ <= 0 || max_speed_ <= 0 ||
        timeout_ <= 0 || stop_brake_ < 0 || stop_brake_ > 1 ||
        max_rolling_brake_ <= 0 || max_rolling_brake_ > 1) {
      throw std::invalid_argument("invalid tractor controller parameter");
    }
    std::ifstream model_file(model_path);
    std::string header, forward, reverse;
    if (!model_file || !std::getline(model_file, header) ||
        !std::getline(model_file, forward) || !std::getline(model_file, reverse)) {
      throw std::runtime_error("Cannot read feedforward model: " + model_path);
    }
    if (!header.empty() && header.back() == '\r') header.pop_back();
    if (header != "gear,coast0,coast1,throttle_gain,brake_gain")
      throw std::runtime_error("Unexpected feedforward model header: " + model_path);
    forward_model_ = parse_model(forward, 1);
    reverse_model_ = parse_model(reverse, -1);
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(actuation_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/vehicle/control_debug", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(cmd_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          const auto now = std::chrono::steady_clock::now();
          if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
            cmd_valid_ = false;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid or reverse cmd_vel; stopping");
            return;
          }
          target_ = clamp(msg->linear.x, -max_speed_, max_speed_);
          yaw_rate_ = msg->angular.z;
          cmd_time_ = now;
          cmd_valid_ = true;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(feedback_topic_, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          if (msg->child_frame_id != base_frame_ || !std::isfinite(msg->twist.twist.linear.x)) {
            feedback_valid_ = false;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid odometry frame or speed; stopping");
            return;
          }
          speed_ = msg->twist.twist.linear.x;
          feedback_time_ = std::chrono::steady_clock::now();
          feedback_valid_ = true;
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), [this]() { tick(); });
  }

  void stop() { publish(0, 1, 0, 0); }

 private:
  using Steady = std::chrono::steady_clock;
  void publish(double accelerator, double brake, double brake1, double steer) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(5);
    msg.data[kAccelerator] = static_cast<float>(accelerator);
    msg.data[kBrake0] = static_cast<float>(brake);
    msg.data[kBrake1] = static_cast<float>(brake1);
    msg.data[kSteer] = static_cast<float>(steer);
    msg.data[kGear] = static_cast<float>(gear_);
    pub_->publish(msg);
  }
  void debug(double desired_accel, double throttle, double brake) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {static_cast<float>(target_), static_cast<float>(setpoint_),
                static_cast<float>(speed_), static_cast<float>(desired_accel),
                static_cast<float>(throttle), static_cast<float>(brake),
                static_cast<float>(integral_), static_cast<float>(gear_)};
    debug_pub_->publish(msg);
  }
  void tick() {
    const auto now = Steady::now();
    double dt = last_tick_ == Steady::time_point{} ? 1.0 / rate_ :
        clamp(std::chrono::duration<double>(now - last_tick_).count(), 0.001, 0.2);
    last_tick_ = now;
    const bool fresh = cmd_valid_ && feedback_valid_ &&
        std::chrono::duration<double>(now - cmd_time_).count() <= timeout_ &&
        std::chrono::duration<double>(now - feedback_time_).count() <= timeout_;
    if (!fresh) {
      integral_ = 0;
      setpoint_ = 0;
      stop();
      debug(0, 0, 1);
      return;
    }
    const int requested_gear = target_ < -0.01 ? -1 : target_ > 0.01 ? 1 : gear_;
    if (requested_gear != gear_) {
      setpoint_ = 0;
      integral_ = 0;
      if (std::abs(speed_) < 0.07) {
        if (stationary_since_ == Steady::time_point{}) stationary_since_ = now;
        if (std::chrono::duration<double>(now - stationary_since_).count() >= 0.4) {
          gear_ = requested_gear;
          shift_until_ = now + std::chrono::milliseconds(500);
          stationary_since_ = Steady::time_point{};
        }
      } else {
        stationary_since_ = Steady::time_point{};
      }
      const double shift_brake = std::abs(speed_) < 0.07 ? 0.35 : max_rolling_brake_;
      publish(0, shift_brake, 0, 0);
      debug(0, 0, shift_brake);
      return;
    }
    stationary_since_ = Steady::time_point{};
    if (now < shift_until_) {
      publish(0, 0.35, 0, 0);
      debug(0, 0, 0.35);
      return;
    }
    const double old_setpoint = setpoint_;
    const double change = target_ - setpoint_;
    setpoint_ += clamp(change, -decel_limit_ * dt, accel_limit_ * dt);
    if (std::abs(target_) <= 0.01 && std::abs(speed_) < 0.05) {
      integral_ = 0;
      setpoint_ = 0;
      publish(0, stop_brake_, 0, 0);
      debug(0, 0, stop_brake_);
      return;
    }
    const Model &model = gear_ == 1 ? forward_model_ : reverse_model_;
    const double error = gear_ * (setpoint_ - speed_);
    const double candidate = clamp(integral_ + error * dt, -2.0, 2.0);
    const double raw_accel = gear_ * (setpoint_ - old_setpoint) / dt + kp_ * error + ki_ * candidate;
    const double desired_accel = clamp(raw_accel, -decel_limit_, accel_limit_);
    const double coast = model.coast(std::max(0.0, gear_ * speed_));
    const double throttle = clamp((desired_accel - coast) / model.throttle_gain, 0, 1);
    const double brake = clamp((coast - desired_accel) / model.brake_gain, 0, max_rolling_brake_);
    const bool accel_allows_integral = raw_accel == desired_accel ||
        (raw_accel > accel_limit_ && error < 0) || (raw_accel < -decel_limit_ && error > 0);
    const bool pedal_allows_integral = (throttle < 1 && brake < max_rolling_brake_) ||
        (throttle >= 1 && error < 0) || (brake >= max_rolling_brake_ && error > 0);
    if (accel_allows_integral && pedal_allows_integral) integral_ = candidate;
    double steer = 0;
    if (std::abs(target_) > 0.05 && std::abs(setpoint_) > 0.1 && gear_ == 1) {
      const double curvature = clamp(yaw_rate_ / std::max(std::abs(setpoint_), 0.3), -max_curvature_, max_curvature_);
      steer = clamp(std::atan(wheelbase_ * curvature) / max_steer_, -1, 1);
    }
    publish(throttle, brake, 0, steer);
    debug(desired_accel, throttle, brake);
  }
  std::string cmd_topic_, feedback_topic_, actuation_topic_, base_frame_;
  double rate_, kp_, ki_, wheelbase_, max_steer_, max_curvature_, accel_limit_, decel_limit_, max_speed_, stop_brake_, max_rolling_brake_, timeout_;
  double target_ = 0, yaw_rate_ = 0, speed_ = 0, setpoint_ = 0, integral_ = 0;
  int gear_ = 1;
  Model forward_model_, reverse_model_;
  bool cmd_valid_ = false, feedback_valid_ = false;
  Steady::time_point cmd_time_{}, feedback_time_{}, last_tick_{}, stationary_since_{}, shift_until_{};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_, debug_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TractorControlNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("tractor_control_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
