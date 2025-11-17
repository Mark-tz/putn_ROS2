#include <rclcpp/rclcpp.hpp>
#include <string>
#include "scout_gazebo/scout_skid_steer.hpp"

using namespace wescore;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scout_odom");

  std::string robot_namespace = node->declare_parameter<std::string>("robot_namespace", "scout_default");
  RCLCPP_INFO(node->get_logger(), "Namespace: %s", robot_namespace.c_str());

  ScoutSkidSteer skid_steer_controller(node, robot_namespace);
  skid_steer_controller.SetupSubscription();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}