#include "backward.hpp"
#include "PUTN_planner.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("global_planning_node"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("global_planning_node"), __VA_ARGS__)
#define ROS_INFO_THROTTLE(count_mod, ...) do { \
  static int __log_counter = 0; \
  if ((__log_counter++ % (count_mod)) == 0) { \
    RCLCPP_INFO(rclcpp::get_logger("global_planning_node"), __VA_ARGS__); \
  } \
} while(0)
#define ROS_WARN_THROTTLE(count_mod, ...) do { \
  static int __log_counter_w = 0; \
  if ((__log_counter_w++ % (count_mod)) == 0) { \
    RCLCPP_WARN(rclcpp::get_logger("global_planning_node"), __VA_ARGS__); \
  } \
} while(0)

using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace PUTN;
using namespace PUTN::visualization;
using namespace PUTN::planner;

namespace backward
{
backward::SignalHandling sh;
}

// ros2 related
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub;
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr wp_sub;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_vis_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_vis_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_vis_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_vis_pub;
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr path_interpolation_pub;
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tree_tra_pub;

// indicate whether the robot has a moving goal
bool has_goal = false;

// simulation param from launch file
double resolution;
double z_min;
double z_max;
double goal_thre;
double step_size;
double h_surf_car;
double max_initial_time;
double radius_fit_plane;
FitPlaneArg fit_plane_arg;
double neighbor_radius;

// useful global variables
Vector3d start_pt;
Vector3d target_pt;
World* world = NULL;
PFRRTStar* pf_rrt_star = NULL;

// function declaration
void rcvWaypointsCallback(const nav_msgs::msg::Path::SharedPtr wp);
void rcvPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map);
void pubInterpolatedPath(const vector<Node*>& solution, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr* path_interpolation_pub);
void findSolution();
void callPlanner();

/**
 *@brief receive goal from rviz
 */
void rcvWaypointsCallback(const nav_msgs::msg::Path::SharedPtr wp)
{
  if (!world->has_map_)
    return;
  has_goal = true;
  target_pt = Vector3d(wp->poses[0].pose.position.x, wp->poses[0].pose.position.y, wp->poses[0].pose.position.z);
  ROS_INFO("Receive the planning target");
}

/**
 *@brief receive point cloud to build the grid map
 */
void rcvPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*pointcloud_map, cloud);

  pcl::PointCloud<pcl::PointXYZ> cloud_clip;
  cloud_clip.reserve(cloud.size());
  for (const auto& pt : cloud)
  {
    if (pt.z >= z_min && pt.z <= z_max) {
      cloud_clip.push_back(pt);
    }
  }

  world->initGridMap(cloud_clip);

  for (const auto& pt : cloud_clip)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    world->setObs(obstacle);
  }
  ROS_INFO_THROTTLE(50, "global_planning_node: cloud_in=%zu cloud_clip=%zu z=[%.3f,%.3f]", cloud.size(), cloud_clip.size(), z_min, z_max);
  visWorld(world, grid_map_vis_pub);
}

/**
 *@brief Linearly interpolate the generated path to meet the needs of local planning
 */
void pubInterpolatedPath(const vector<Node*>& solution, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr* path_interpolation_pub)
{
  if (!path_interpolation_pub || !(*path_interpolation_pub))
    return;
  std_msgs::msg::Float32MultiArray msg;
  for (size_t i = 0; i < solution.size(); i++)
  {
    if (i == solution.size() - 1)
    {
      msg.data.push_back(solution[i]->position_(0));
      msg.data.push_back(solution[i]->position_(1));
      msg.data.push_back(solution[i]->position_(2));
    }
    else
    {
      size_t interpolation_num = (size_t)(EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
      Vector3d diff_pt = solution[i + 1]->position_ - solution[i]->position_;
      for (size_t j = 0; j < interpolation_num; j++)
      {
        Vector3d interpt = solution[i]->position_ + diff_pt * (float)j / interpolation_num;
        msg.data.push_back(interpt(0));
        msg.data.push_back(interpt(1));
        msg.data.push_back(interpt(2));
      }
    }
  }
  (*path_interpolation_pub)->publish(msg);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void findSolution()
{
  
  ROS_INFO_THROTTLE(50, "Start calling PF-RRT*");
  Path solution = Path();

  pf_rrt_star->initWithGoal(start_pt, target_pt);

  // Case1: The PF-RRT* can't work at when the origin can't be project to surface
  if (pf_rrt_star->state() == Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
  }
  // Case2: If both the origin and the target can be projected,the PF-RRT* will execute
  //       global planning and try to generate a path
  else if (pf_rrt_star->state() == Global)
  {
    ROS_INFO_THROTTLE(200, "Starting PF-RRT* algorithm at the state of global planning");
    int max_iter = 5000;
    double max_time = 100.0;

    while (solution.type_ == Path::Empty && max_time < max_initial_time)
    {
      solution = pf_rrt_star->planner(max_iter, max_time);
      max_time += 100.0;
    }

    if (!solution.nodes_.empty())
      ROS_INFO_THROTTLE(50, "Get a global path!");
    else
      ROS_WARN_THROTTLE(50, "No solution found!");
  }
  // Case3: If the origin can be projected while the target can not,the PF-RRT*
  //       will try to find a temporary target for transitions.
  else
  {
    ROS_INFO_THROTTLE(50, "Starting PF-RRT* algorithm at the state of rolling planning");
    int max_iter = 1500;
    double max_time = 100.0;

    solution = pf_rrt_star->planner(max_iter, max_time);

    if (!solution.nodes_.empty())
      ROS_INFO_THROTTLE(50, "Get a sub path!");
    else
      ROS_WARN_THROTTLE(50, "No solution found!");
  }
  ROS_INFO_THROTTLE(50, "End calling PF-RRT*");
  

  pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);
  visPath(solution.nodes_, path_vis_pub);
  visSurf(solution.nodes_, surf_vis_pub);

  // When the PF-RRT* generates a short enough global path,it's considered that the robot has
  // reached the goal region.
  if (solution.type_ == Path::Global && EuclideanDistance(pf_rrt_star->origin(), pf_rrt_star->target()) < goal_thre)
  {
    has_goal = false;
    visOriginAndGoal({}, goal_vis_pub);
    visPath({}, path_vis_pub);
    ROS_INFO_THROTTLE(50, "The Robot has achieved the goal!!!");
  }

  if (solution.type_ == Path::Empty)
    visPath({}, path_vis_pub);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void callPlanner()
{
  static double init_time_cost = 0.0;
  if (!world->has_map_)
    return;

  // The tree will expand at a certain frequency to explore the space more fully
  if (!has_goal && init_time_cost < 1000)
  {
    timeval start;
    gettimeofday(&start, NULL);
    pf_rrt_star->initWithoutGoal(start_pt);
    timeval end;
    gettimeofday(&end, NULL);
    init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    if (pf_rrt_star->state() == WithoutGoal)
    {
      int max_iter = 550;
      double max_time = 100.0;
      pf_rrt_star->planner(max_iter, max_time);
      ROS_INFO("Current size of tree: %d", (int)(pf_rrt_star->tree().size()));
    }
    else
      ROS_WARN("The start point can't be projected,unable to execute PF-RRT* algorithm");
  }
  // If there is a specified moving target,call PF-RRT* to find a solution
  else if (has_goal)
  {
    findSolution();
    init_time_cost = 0.0;
  }
  // The expansion of tree will stop after the process of initialization takes more than 1s
  else
    ROS_INFO_THROTTLE(50, "The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star->tree().size()));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("global_planning_node");

  map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("map", 10, rcvPointCloudCallBack);
  wp_sub = node->create_subscription<nav_msgs::msg::Path>("waypoints", 10, rcvWaypointsCallback);

  grid_map_vis_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis", 10);
  path_vis_pub = node->create_publisher<visualization_msgs::msg::Marker>("path_vis", 20);
  goal_vis_pub = node->create_publisher<visualization_msgs::msg::Marker>("goal_vis", 10);
  surf_vis_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("surf_vis", 100);
  tree_vis_pub = node->create_publisher<visualization_msgs::msg::Marker>("tree_vis", 10);
  tree_tra_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("tree_tra", 10);
  path_interpolation_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("global_path", 10);

  node->declare_parameter<double>("map/resolution", 0.1);
  node->get_parameter("map/resolution", resolution);
  node->declare_parameter<double>("map/z_min", -0.5);
  node->declare_parameter<double>("map/z_max", 0.4);
  node->get_parameter("map/z_min", z_min);
  node->get_parameter("map/z_max", z_max);

  node->declare_parameter<double>("planning/goal_thre", 1.0);
  node->declare_parameter<double>("planning/step_size", 0.2);
  node->declare_parameter<double>("planning/h_surf_car", 0.4);
  node->declare_parameter<double>("planning/neighbor_radius", 1.0);
  node->declare_parameter<double>("planning/w_fit_plane", 0.4);
  node->declare_parameter<double>("planning/w_flatness", 4000.0);
  node->declare_parameter<double>("planning/w_slope", 0.4);
  node->declare_parameter<double>("planning/w_sparsity", 0.4);
  node->declare_parameter<double>("planning/ratio_min", 0.25);
  node->declare_parameter<double>("planning/ratio_max", 0.4);
  node->declare_parameter<double>("planning/conv_thre", 0.1152);
  node->declare_parameter<double>("planning/radius_fit_plane", 1.0);
  node->declare_parameter<double>("planning/max_initial_time", 1000.0);

  node->get_parameter("planning/goal_thre", goal_thre);
  node->get_parameter("planning/step_size", step_size);
  node->get_parameter("planning/h_surf_car", h_surf_car);
  node->get_parameter("planning/neighbor_radius", neighbor_radius);

  node->get_parameter("planning/w_fit_plane", fit_plane_arg.w_total_);
  node->get_parameter("planning/w_flatness", fit_plane_arg.w_flatness_);
  node->get_parameter("planning/w_slope", fit_plane_arg.w_slope_);
  node->get_parameter("planning/w_sparsity", fit_plane_arg.w_sparsity_);
  node->get_parameter("planning/ratio_min", fit_plane_arg.ratio_min_);
  node->get_parameter("planning/ratio_max", fit_plane_arg.ratio_max_);
  node->get_parameter("planning/conv_thre", fit_plane_arg.conv_thre_);

  node->get_parameter("planning/radius_fit_plane", radius_fit_plane);
  node->get_parameter("planning/max_initial_time", max_initial_time);

  world = new World(resolution);
  pf_rrt_star = new PFRRTStar(h_surf_car, world);

  pf_rrt_star->setGoalThre(goal_thre);
  pf_rrt_star->setStepSize(step_size);
  pf_rrt_star->setFitPlaneArg(fit_plane_arg);
  pf_rrt_star->setFitPlaneRadius(radius_fit_plane);
  pf_rrt_star->setNeighborRadius(neighbor_radius);

  pf_rrt_star->goal_vis_pub_ = goal_vis_pub;
  pf_rrt_star->tree_vis_pub_ = tree_vis_pub;
  pf_rrt_star->tree_tra_pub_ = tree_tra_pub;

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  while (rclcpp::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);

    geometry_msgs::msg::TransformStamped transform;
    while (true && rclcpp::ok())
    {
      try
      {
        transform = tf_buffer->lookupTransform("world", "base_link", tf2::TimePointZero);
        break;
      }
      catch (const tf2::TransformException& ex)
      {
        continue;
      }
    }
    start_pt << transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z;

    rclcpp::spin_some(node);
    callPlanner();
    double ms;
    do
    {
      timeval end;
      gettimeofday(&end, NULL);
      ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    } while (ms < 100);
  }
  rclcpp::shutdown();
  return 0;
}
