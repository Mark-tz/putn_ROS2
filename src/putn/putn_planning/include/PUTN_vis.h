/**
 *  This file is used for visualization
 *  Author: jianzhuozhu
 *  Date: 2021-7-27
 */

#ifndef PUTN_VIS_H
#define PUTN_VIS_H

#include "PUTN_classes.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace PUTN
{
namespace visualization
{
void visWorld(World* world, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_vis_pub);
void visSurf(const std::vector<Node*>& solution, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_vis_pub);
void visOriginAndGoal(const std::vector<Node*>& pts, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr origin_and_goal_vis_pub);
void visPath(const std::vector<Node*>& solution, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_pub);
void visTree(const std::vector<Node*>& tree, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_vis_pub);
}

}
#endif