#include "backward.hpp"
#include "PUTN_planner.h"
#include "PUTN_astar_planner.h"
#include "PUTN_vis.h"
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
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trav_vis_pub;

// indicate whether the robot has a moving goal
bool has_goal = false;

// simulation param from launch file
double resolution_xy;
double resolution_z;
double z_min;
double z_max;
double goal_thre;
double step_size;
double h_surf_car;
double max_initial_time;
double radius_fit_plane;
FitPlaneArg fit_plane_arg;
double neighbor_radius;
double max_slope_deg_param = 15.0;
double obs_inflation_param = 0.5;

// useful global variables
Vector3d start_pt;
Vector3d target_pt;
World* world = NULL;
// PFRRTStar* pf_rrt_star = NULL;
AStarPlanner* astar_planner = NULL;

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
  timeval t0; gettimeofday(&t0, NULL);
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
  timeval t1; gettimeofday(&t1, NULL);
  for (const auto& pt : cloud_clip)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    world->setObs(obstacle);
  }
  timeval t2; gettimeofday(&t2, NULL);
  // inflate only on surface plane to thicken horizontally
  int infl_xy = std::max(1, (int)std::round(0.2 / resolution_xy));
  world->inflateOccupancy(infl_xy, 0);
  timeval t3; gettimeofday(&t3, NULL);
  // ensure each (ix,iy) has at least one surface cell via neighbor interpolation
  world->ensureSurfacePerColumn(std::max(1, (int)std::round(0.4 / resolution_xy)));
  
  // Set clearance for vertical check (robot height)
  world->setClearanceZ(h_surf_car);
  world->updateVerticalCaches();
  
  // Re-apply obstacle inflation on the updated map
  world->inflateMap(obs_inflation_param);

  timeval t4; gettimeofday(&t4, NULL);
  ROS_INFO_THROTTLE(50, "global_planning_node: cloud_in=%zu cloud_clip=%zu z=[%.3f,%.3f]", cloud.size(), cloud_clip.size(), z_min, z_max);
  visWorld(world, grid_map_vis_pub);
  if (trav_vis_pub)
    visMapTraversability(world, trav_vis_pub, max_slope_deg_param);
  timeval t5; gettimeofday(&t5, NULL);
  double ms_init = 1000 * (t1.tv_sec - t0.tv_sec) + 0.001 * (t1.tv_usec - t0.tv_usec);
  double ms_setobs = 1000 * (t2.tv_sec - t1.tv_sec) + 0.001 * (t2.tv_usec - t1.tv_usec);
  double ms_infl = 1000 * (t3.tv_sec - t2.tv_sec) + 0.001 * (t3.tv_usec - t2.tv_usec);
  double ms_interp = 1000 * (t4.tv_sec - t3.tv_sec) + 0.001 * (t4.tv_usec - t3.tv_usec);
  double ms_vis = 1000 * (t5.tv_sec - t4.tv_sec) + 0.001 * (t5.tv_usec - t4.tv_usec);
  ROS_INFO_THROTTLE(50, "map_build_time ms: init=%.2f setObs=%.2f inflate=%.2f ensureColumn=%.2f visualize=%.2f", ms_init, ms_setobs, ms_infl, ms_interp, ms_vis);
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
  
  if (!solution.empty()) {
    ROS_INFO("Published interpolated path with %zu points. Triggering potential downstream effects.", msg.data.size()/3);
  }
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void findSolution()
{
  
  ROS_INFO_THROTTLE(50, "Start calling A*");
  Path solution = Path();

  astar_planner->initWithGoal(start_pt, target_pt);

  // Case1: The A* can't work at when the origin can't be project to surface
  if (astar_planner->state() == AStarPlanner::Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start A* algorithm!!!");
  }
  // Case2: If both the origin and the target can be projected,the A* will execute
  //       global planning and try to generate a path
  else if (astar_planner->state() == AStarPlanner::Global)
  {
    ROS_INFO_THROTTLE(200, "Starting A* algorithm at the state of global planning");
    // A* doesn't really use max_iter/time like RRT in incremental loop, it runs to completion or limit
    int max_iter = 50000; // Give it enough iterations
    double max_time = 500.0; // ms

    // Run once
    solution = astar_planner->planner(max_iter, max_time);
    
    // Visualization of "tree" (visited nodes)
    std::vector<Node*> visited = astar_planner->getVisitedNodes();
    visTree(visited, tree_vis_pub);
    
    // Publish tree_tra for GPR or local planner compatibility
    // Format: [x, y, z, z_true] per node
    if (tree_tra_pub) {
        std_msgs::msg::Float32MultiArray tree_msg;
        tree_msg.data.reserve(visited.size() * 4);
        for (const auto* n : visited) {
            tree_msg.data.push_back(n->position_(0));
            tree_msg.data.push_back(n->position_(1));
            tree_msg.data.push_back(n->position_(2));
            tree_msg.data.push_back(n->position_(2)); // z_true is same as z for A*
        }
        tree_tra_pub->publish(tree_msg);
    }

    if (!solution.nodes_.empty())
      ROS_INFO_THROTTLE(50, "Get a global path!");
    else
      ROS_WARN_THROTTLE(50, "No solution found!");
  }
  // Case3: Rolling planning logic - for A* we usually just replan Global or to a sub-goal
  // For compatibility with code structure, we treat it same as Global if it was valid
  else
  {
     // Should be covered by Global or Invalid
  }
  ROS_INFO_THROTTLE(50, "End calling A*");
  

  pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);
  visPath(solution.nodes_, path_vis_pub);
  visSurf(solution.nodes_, surf_vis_pub);

  // When the A* generates a short enough global path,it's considered that the robot has
  // reached the goal region.
  if (solution.type_ == Path::Global && EuclideanDistance(start_pt, target_pt) < goal_thre)
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
  /*
  if (!has_goal)
  {
    if (init_time_cost < 1000)
    {
      timeval t0; gettimeofday(&t0, NULL);
      pf_rrt_star->initWithoutGoal(start_pt);
      timeval t1; gettimeofday(&t1, NULL);
      init_time_cost = 1000 * (t1.tv_sec - t0.tv_sec) + 0.001 * (t1.tv_usec - t0.tv_usec);
      ROS_INFO_THROTTLE(50, "initWithoutGoal time ms=%.2f", init_time_cost);
      
      if (pf_rrt_star->state() == WithoutGoal)
      {
        // Log before planner call
        ROS_INFO_THROTTLE(10, "Calling planner(WithoutGoal) - Start: tree_size=%d", (int)(pf_rrt_star->tree().size()));

        int max_iter = 550;
        double max_time = 100.0;
        timeval t2; gettimeofday(&t2, NULL);
        pf_rrt_star->planner(max_iter, max_time);
        timeval t3; gettimeofday(&t3, NULL);
        double ms_plan = 1000 * (t3.tv_sec - t2.tv_sec) + 0.001 * (t3.tv_usec - t2.tv_usec);
        ROS_INFO_THROTTLE(50, "planner(WithoutGoal) iterations completed: %d/%d, time used: %.2f/%.2f ms, tree_size=%d", 
             pf_rrt_star->getCurrentIterations(), max_iter, ms_plan, max_time, (int)(pf_rrt_star->tree().size()));
      }
      else
        ROS_WARN_THROTTLE(50, "The start point can't be projected,unable to execute PF-RRT* algorithm");
    }
    else
    {
      ROS_WARN_THROTTLE(50, "initWithoutGoal took too long (%.2f ms) > 1000ms. Resetting timer to try again next cycle.", init_time_cost);
      init_time_cost = 0.0;
    }
  }
  */
  if (!has_goal) {
     // A* does not support "WithoutGoal" random exploration. Do nothing.
     return;
  }
  // If there is a specified moving target,call PF-RRT* to find a solution
  else if (has_goal)
  {
    timeval t4; gettimeofday(&t4, NULL);
    findSolution();
    timeval t5; gettimeofday(&t5, NULL);
    double ms_find = 1000 * (t5.tv_sec - t4.tv_sec) + 0.001 * (t5.tv_usec - t4.tv_usec);
    ROS_INFO_THROTTLE(50, "findSolution time ms=%.2f", ms_find);
    init_time_cost = 0.0;
  }
  // The expansion of tree will stop after the process of initialization takes more than 1s
  else
  {
    static int log_counter = 0;
    if (log_counter++ % 50 == 0) {
        // ROS_INFO("The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star->tree().size()));
        ROS_INFO("Planner idle (A* mode). Waiting for goal.");
    }
  }
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
  trav_vis_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("grid_trav_vis", 10);

  node->declare_parameter<double>("map/resolution_xy", 0.1);
  node->get_parameter("map/resolution_xy", resolution_xy);
  node->declare_parameter<double>("map/resolution_z", 0.01);
  node->get_parameter("map/resolution_z", resolution_z);
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
  node->declare_parameter<double>("planning/max_height_diff", 0.2);
  node->declare_parameter<double>("planning/max_slope_deg", 15.0);
  node->declare_parameter<double>("planning/clearance_z", 0.5);
  node->declare_parameter<bool>("planning/disable_shortcut", false);

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
  double max_height_diff;
  node->get_parameter("planning/max_height_diff", max_height_diff);
  node->get_parameter("planning/max_slope_deg", max_slope_deg_param);
  double clearance_z;
  bool disable_shortcut_param = false;
  node->get_parameter("planning/clearance_z", clearance_z);
  node->get_parameter("planning/disable_shortcut", disable_shortcut_param);

  node->declare_parameter<double>("planning/obs_inflation", 0.5);
  node->get_parameter("planning/obs_inflation", obs_inflation_param);

  double shortcut_tolerance_param = 5.0;
  node->declare_parameter<double>("planning/shortcut_tolerance", 5.0);
  node->get_parameter("planning/shortcut_tolerance", shortcut_tolerance_param);

  world = new World(resolution_xy, resolution_z);
  // Initialize clearance_z_ explicitly with h_surf_car parameter
  world->setClearanceZ(h_surf_car);
  world->setHeightDiffThre(max_height_diff);
  world->setMaxSlopeDeg(max_slope_deg_param);
  // Pre-inflate map obstacles for safety
  world->inflateMap(obs_inflation_param);
  ROS_INFO("World initialized with resolution_xy=%.2f resolution_z=%.2f clearance_z=%.2f, inflation=%.2f", resolution_xy, resolution_z, h_surf_car, obs_inflation_param);

  // pf_rrt_star = new PFRRTStar(h_surf_car, world);
  // pf_rrt_star->setGoalThre(goal_thre);
  // ...
  
  astar_planner = new AStarPlanner(world);
  astar_planner->setMaxSlopeDeg(max_slope_deg_param);
  astar_planner->setWeightDistance(1.0);
  astar_planner->setObsInflation(obs_inflation_param);
  astar_planner->setShortcutTolerance(shortcut_tolerance_param);
  // astar_planner->setWeightSlope(0.5); 

  /*
  pf_rrt_star->setGoalThre(goal_thre);
  pf_rrt_star->setStepSize(step_size);
  pf_rrt_star->setFitPlaneArg(fit_plane_arg);
  pf_rrt_star->setFitPlaneRadius(radius_fit_plane);
  pf_rrt_star->setNeighborRadius(neighbor_radius);
  pf_rrt_star->setMaxHeightDiff(max_height_diff);
  pf_rrt_star->setMaxSlopeDeg(max_slope_deg_param);
  pf_rrt_star->setDisableShortcut(disable_shortcut_param);

  pf_rrt_star->goal_vis_pub_ = goal_vis_pub;
  pf_rrt_star->tree_vis_pub_ = tree_vis_pub;
  pf_rrt_star->tree_tra_pub_ = tree_tra_pub;
  */

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  while (rclcpp::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);

    geometry_msgs::msg::TransformStamped transform;
    int tf_lookup_tries = 0;
    while (true && rclcpp::ok())
    {
      try
      {
        transform = tf_buffer->lookupTransform("world", "base_link", tf2::TimePointZero);
        break;
      }
      catch (const tf2::TransformException& ex)
      {
        tf_lookup_tries++;
        // if ((tf_lookup_tries % 50) == 0)
          // ROS_WARN("TF lookup retrying: tries=%d", tf_lookup_tries);
        continue;
      }
    }
    if (tf_lookup_tries > 0)
      ROS_INFO_THROTTLE(50, "TF lookup recovered after %d tries", tf_lookup_tries);
    start_pt << transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z;

    rclcpp::spin_some(node);
    timeval t_call_start; gettimeofday(&t_call_start, NULL);
    callPlanner();
    timeval t_call_end; gettimeofday(&t_call_end, NULL);
    double ms_call = 1000 * (t_call_end.tv_sec - t_call_start.tv_sec) + 0.001 * (t_call_end.tv_usec - t_call_start.tv_usec);
    ROS_INFO_THROTTLE(50, "callPlanner time ms=%.2f", ms_call);
    rclcpp::Rate rate(50);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
