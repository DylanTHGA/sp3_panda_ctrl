#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>

extern moveit::planning_interface::MoveGroupInterface* arm_ptr;
extern moveit::planning_interface::MoveGroupInterface* hand_ptr;

extern geometry_msgs::Quaternion default_orientation;
extern geometry_msgs::Pose place_pose;
extern std::vector<double> start_joints;
extern std::vector<double> observer_joints;
extern double table_z;
extern double hover_z;

void printHelp();
void pickCallback(const geometry_msgs::Point::ConstPtr &msg);
void initializeDefaultOrientation();
void initializePredefinedPoses();
bool moveToStartPosition();
