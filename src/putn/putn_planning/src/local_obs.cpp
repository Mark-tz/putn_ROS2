#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl/filters/passthrough.h>
#include <rclcpp/rclcpp.hpp>
#include "backward.hpp"
#include "PUTN_classes.h"

using namespace std;
using namespace Eigen;
using namespace PUTN;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pt_sub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pub;
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_array_pub;

double resolution, local_x_l, local_x_u, local_y_l, local_y_u, local_z_l, local_z_u;

std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

void rcvVelodyneCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr velodyne_points);

void rcvVelodyneCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr velodyne_points)
{
  static int __obs_log_counter = 0;
  if ((__obs_log_counter++ % 50) == 0) {
    RCLCPP_INFO(node->get_logger(), "Receive velodyne!");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*velodyne_points, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(cloud);
  passthrough.setFilterFieldName("x");
  passthrough.setFilterLimits(local_x_l, local_x_u);
  passthrough.filter(*cloud_after_PassThrough);

  passthrough.setInputCloud(cloud_after_PassThrough);
  passthrough.setFilterFieldName("y");
  passthrough.setFilterLimits(local_y_l, local_y_u);
  passthrough.filter(*cloud_after_PassThrough);

  passthrough.setInputCloud(cloud_after_PassThrough);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(local_z_l, local_z_u);
  passthrough.filter(*cloud_after_PassThrough);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);
  Vector3d lowerbound(local_x_l, local_y_l, local_z_l);
  Vector3d upperbound(local_x_u, local_y_u, local_z_u);
  World local_world = World(resolution);
  local_world.initGridMap(lowerbound, upperbound);
  for (const auto& pt : (*cloud_after_PassThrough).points)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    if (local_world.isFree(obstacle))
    {
      local_world.setObs(obstacle);

      Vector3d obstacle_round = local_world.coordRounding(obstacle);
      pcl::PointXYZ pt_add;
      pt_add.x = obstacle_round(0);
      pt_add.y = obstacle_round(1);
      pt_add.z = obstacle_round(2);
      cloud_filt->points.push_back(pt_add);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);

  std_msgs::msg::Float32MultiArray obs_array;
  for (const auto& pt : cloud_filt->points)
  {
    geometry_msgs::msg::PointStamped origin_point;
    origin_point.header.frame_id = "aft_mapped";
    origin_point.point.x = pt.x;
    origin_point.point.y = pt.y;
    origin_point.point.z = pt.z;

    geometry_msgs::msg::PointStamped trans_point;
    try {
      trans_point = tf_buffer->transform(origin_point, "world");
    } catch (const tf2::TransformException& ex) {
      continue;
    }

    pcl::PointXYZ _pt;
    if (!(-1.2 < pt.x && pt.x < 0.4 && -0.4 < pt.y && pt.y < 0.4))
    {
      obs_array.data.push_back(trans_point.point.x);
      obs_array.data.push_back(trans_point.point.y);
      obs_array.data.push_back(trans_point.point.z);
    }

    _pt.x = trans_point.point.x;
    _pt.y = trans_point.point.y;
    _pt.z = trans_point.point.z;

    cloud_tran->points.push_back(_pt);
  }

  sensor_msgs::msg::PointCloud2 obs_vis;
  pcl::toROSMsg(*cloud_tran, obs_vis);

  obs_vis.header.frame_id = "world";
  obs_pub->publish(obs_vis);
  obs_array_pub->publish(obs_array);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("local_obs_node");

  pt_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("map", 1, rcvVelodyneCallBack);

  obs_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("obs_vis", 1);
  obs_array_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("/obs", 1);

  node->declare_parameter("map/resolution", 0.1);
  node->declare_parameter("map/local_x_l", -2.0);
  node->declare_parameter("map/local_x_u", 2.0);
  node->declare_parameter("map/local_y_l", -2.0);
  node->declare_parameter("map/local_y_u", 2.0);
  node->declare_parameter("map/local_z_l", -0.3);
  node->declare_parameter("map/local_z_u", 0.5);

  node->get_parameter("map/resolution", resolution);
  node->get_parameter("map/local_x_l", local_x_l);
  node->get_parameter("map/local_x_u", local_x_u);
  node->get_parameter("map/local_y_l", local_y_l);
  node->get_parameter("map/local_y_u", local_y_u);
  node->get_parameter("map/local_z_l", local_z_l);
  node->get_parameter("map/local_z_u", local_z_u);

  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  rclcpp::Rate rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
