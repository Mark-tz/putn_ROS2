/*
 * scout_skid_steer.hpp
 *
 * Created on: Mar 25, 2020 22:52
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_SKID_STEER_HPP
#define SCOUT_SKID_STEER_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace wescore {
class ScoutSkidSteer {
 public:
  explicit ScoutSkidSteer(std::shared_ptr<rclcpp::Node> node, std::string robot_name = "");

  void SetupSubscription();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string robot_name_;
  std::string motor_fr_topic_;
  std::string motor_fl_topic_;
  std::string motor_rl_topic_;
  std::string motor_rr_topic_;
  std::string cmd_topic_;

  const double SCOUT_WHEELBASE = 0.498;
  const double SCOUT_WHEEL_RADIUS = 0.16459;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_fr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_fl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_rl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_rr_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
}  // namespace wescore

#endif /* SCOUT_SKID_STEER_HPP */
