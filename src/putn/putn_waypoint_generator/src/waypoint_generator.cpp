#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using bfmt = boost::format;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2;
string waypoint_type = string("manual");
bool is_odom_ready;
nav_msgs::msg::Odometry odom;
nav_msgs::msg::Path waypoints;

std::deque<nav_msgs::msg::Path> waypointSegments;
rclcpp::Time trigged_time;

template<typename T>
inline void declare_if_missing(const std::string& name, const T& default_value) {
    if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
    }
}

void load_seg(int segid, const rclcpp::Time& time_base) {
    std::string seg_str = boost::str(bfmt("seg%d/") % segid);
    double yaw = 0.0;
    double time_from_start = 0.0;
    declare_if_missing<double>(seg_str + "yaw", yaw);
    declare_if_missing<double>(seg_str + "time_from_start", time_from_start);
    node->get_parameter(seg_str + "yaw", yaw);
    node->get_parameter(seg_str + "time_from_start", time_from_start);
    if (yaw < -3.1499999 || yaw > 3.14999999) return;
    if (time_from_start < 0.0) return;

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;
    declare_if_missing<std::vector<double>>(seg_str + "x", std::vector<double>{});
    declare_if_missing<std::vector<double>>(seg_str + "y", std::vector<double>{});
    declare_if_missing<std::vector<double>>(seg_str + "z", std::vector<double>{});
    node->get_parameter(seg_str + "x", ptx);
    node->get_parameter(seg_str + "y", pty);
    node->get_parameter(seg_str + "z", ptz);
    if (ptx.empty()) return;
    if (!(ptx.size() == pty.size() && ptx.size() == ptz.size())) return;

    nav_msgs::msg::Path path_msg;
    rclcpp::Time stamp = time_base + rclcpp::Duration::from_seconds(time_from_start);
    path_msg.header.stamp = stamp;

    double roll = 0.0, pitch = 0.0, baseyaw = 0.0;
    tf2::Quaternion base_q;
    tf2::fromMsg(odom.pose.pose.orientation, base_q);
    tf2::Matrix3x3(base_q).getRPY(roll, pitch, baseyaw);
    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::msg::PoseStamped pt;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, baseyaw + yaw);
        pt.pose.orientation = tf2::toMsg(q);
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));
        Eigen::Vector2d rdp;
        rdp.x() = std::cos(-baseyaw - yaw) * dp.x() + std::sin(-baseyaw - yaw) * dp.y();
        rdp.y() = -std::sin(-baseyaw - yaw) * dp.x() + std::cos(-baseyaw - yaw) * dp.y();
        pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
        path_msg.poses.push_back(pt);
    }
    waypointSegments.push_back(path_msg);
}

void load_waypoints(const rclcpp::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments.clear();
    declare_if_missing<int>("segment_cnt", seg_cnt);
    node->get_parameter("segment_cnt", seg_cnt);
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(i, time_base);
    }
}

void publish_waypoints() {
    waypoints.header.frame_id = std::string("world");
    waypoints.header.stamp = node->now();
    pub1->publish(waypoints);
    if (!waypoints.poses.empty()) {
        const auto& p = waypoints.poses.front().pose.position;
        RCLCPP_INFO(node->get_logger(), "[waypoint_generator] publish waypoints size=%zu first=(%.3f,%.3f,%.3f)", waypoints.poses.size(), p.x, p.y, p.z);
    } else {
        RCLCPP_INFO(node->get_logger(), "[waypoint_generator] publish waypoints size=0");
    }
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header = odom.header;
    init_pose.pose = odom.pose.pose;
    waypoints.poses.insert(waypoints.poses.begin(), init_pose);
    waypoints.poses.clear();
}

void publish_waypoints_vis() {
    nav_msgs::msg::Path wp_vis = waypoints;
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = std::string("world");
    poseArray.header.stamp = node->now();

    geometry_msgs::msg::Pose init_pose;
    init_pose = odom.pose.pose;
    poseArray.poses.push_back(init_pose);

    for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
        geometry_msgs::msg::Pose p;
        p = it->pose;
        poseArray.poses.push_back(p);
    }
    pub2->publish(poseArray);
    RCLCPP_INFO(node->get_logger(), "[waypoint_generator] publish waypoints_vis size=%zu", poseArray.poses.size());
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    is_odom_ready = true;
    odom = *msg;

    if (waypointSegments.size()) {
        rclcpp::Time expected_time(waypointSegments.front().header.stamp);
        if (rclcpp::Time(odom.header.stamp) >= expected_time) {
            waypoints = waypointSegments.front();

            std::stringstream ss;
            ss << bfmt("Series send %.3f from start:\n") % trigged_time.seconds();
            for (auto& pose_stamped : waypoints.poses) {
                ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                          pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                          pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                          pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                          pose_stamped.pose.orientation.z << std::endl;
            }
            RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());

            publish_waypoints_vis();
            publish_waypoints();

            waypointSegments.pop_front();
        }
    }
}

void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    trigged_time = node->now();
    node->get_parameter("waypoint_type", waypoint_type);
    RCLCPP_INFO(node->get_logger(),
                "[waypoint_generator] goal received: pos(%.3f, %.3f, %.3f) quat(%.3f, %.3f, %.3f, %.3f), waypoint_type=%s",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                waypoint_type.c_str());
    if (waypoint_type == string("manual") && waypoints.poses.empty() && std::fabs(msg->pose.position.z) < 1e-6) {
        geometry_msgs::msg::PoseStamped pt = *msg;
        waypoints.poses.clear();
        waypoints.poses.push_back(pt);
        RCLCPP_INFO(node->get_logger(), "[waypoint_generator] manual immediate publish size=%zu", waypoints.poses.size());
        publish_waypoints_vis();
        publish_waypoints();
        return;
    }
    
    if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(trigged_time);
    } else if (waypoint_type == string("manual-lonely-waypoint")) {
        if (msg->pose.position.z >= 0) {
            geometry_msgs::msg::PoseStamped pt = *msg;
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            RCLCPP_INFO(node->get_logger(), "[waypoint_generator] lonely push size=%zu", waypoints.poses.size());
            publish_waypoints_vis();
            publish_waypoints();
        }
    } else {
        if (msg->pose.position.z >= 0) {
            geometry_msgs::msg::PoseStamped pt = *msg;
            if (waypoint_type == string("noyaw")) {
                double roll = 0.0, pitch = 0.0, yaw = 0.0;
                tf2::Quaternion q;
                tf2::fromMsg(odom.pose.pose.orientation, q);
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                tf2::Quaternion nq;
                nq.setRPY(0.0, 0.0, yaw);
                pt.pose.orientation = tf2::toMsg(nq);
            }
            waypoints.poses.push_back(pt);
            RCLCPP_INFO(node->get_logger(), "[waypoint_generator] push size=%zu", waypoints.poses.size());
            publish_waypoints_vis();
        } else if (msg->pose.position.z > -1.0) {
            if (waypoints.poses.size() >= 1) {
                waypoints.poses.erase(std::prev(waypoints.poses.end()));
            }
            RCLCPP_INFO(node->get_logger(), "[waypoint_generator] pop size=%zu", waypoints.poses.size());
            publish_waypoints_vis();
        } else {
            if (waypoints.poses.size() >= 1) {
                RCLCPP_INFO(node->get_logger(), "[waypoint_generator] commit size=%zu", waypoints.poses.size());
                publish_waypoints_vis();
                publish_waypoints();
            }
        }
    }
}

void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!is_odom_ready) {
        RCLCPP_ERROR(node->get_logger(), "[waypoint_generator] No odom!");
        return;
    }

    trigged_time = rclcpp::Time(odom.header.stamp);

    node->get_parameter("waypoint_type", waypoint_type);

    RCLCPP_ERROR(node->get_logger(), "Pattern %s generated!", waypoint_type.c_str());
    if (waypoint_type == string("free")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
   } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(trigged_time);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("waypoint_generator");
    node->declare_parameter<std::string>("waypoint_type", string("manual"));
    node->get_parameter("waypoint_type", waypoint_type);
    RCLCPP_INFO(node->get_logger(), "[waypoint_generator] started, waypoint_type=%s", waypoint_type.c_str());
    auto sub1 = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, odom_callback);
    auto sub2 = node->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 10, goal_callback);
    auto sub3 = node->create_subscription<geometry_msgs::msg::PoseStamped>("traj_start_trigger", 10, traj_start_trigger_callback);
    pub1 = node->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
    pub2 = node->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);

    trigged_time = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
