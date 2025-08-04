#include "panda_control_utils.h"
#include <ros/ros.h>
#include <cmath>

using moveit::planning_interface::MoveGroupInterface;

MoveGroupInterface* arm_ptr = nullptr;
MoveGroupInterface* hand_ptr = nullptr;

// Heights used for pick operations
double table_z = 0.79 + 0.118;
double hover_z = table_z + 0.2;

geometry_msgs::Quaternion default_orientation;
geometry_msgs::Pose place_pose;

// Predefined joint configurations for the start and observer pose
std::vector<double> start_joints = {
    0.0,
    -45.0 * M_PI / 180.0,
    0.0,
    -135.0 * M_PI / 180.0,
    0.0,
    90.0 * M_PI / 180.0,
    45.0 * M_PI / 180.0
};
std::vector<double> observer_joints = {
    0.0 * M_PI / 180.0,
    -50.0 * M_PI / 180.0,
    0.0 * M_PI / 180.0,
    -100.0 * M_PI / 180.0,
    0.0 * M_PI / 180.0,
    60.0 * M_PI / 180.0,
    45.0 * M_PI / 180.0
};

// CLI help text
void printHelp() {
  ROS_INFO_STREAM(R"EOS(
Available commands:
  help                        - show this help
  move_pose x y z             - move end-effector to x,y,z (m)
  set_orientation qw qx qy qz - change default orientation (quaternion)
  move_joints j1 ... j7       - move all 7 joints to given angles (degree)
  print_pose                  - print current end-effector pose
  print_joints                - print current joint angles in degrees
  open_gripper                - open the panda hand
  close_gripper               - close the panda hand
  stop                        - stop any current motion
  goto_start                  - move to predefined start joint configuration
  goto_observer               - move to predefined observer joint configuration
  goto_place                  - move to predefined placement pose (XYZ)
  pick_xy x y                 - pick object at (x,y) on table
  quit                        - shutdown node
)EOS");
}

// Called either via CLI ("pick_xy x y") or ROS topic ("auto_pick" does not work yet )
void pickCallback(const geometry_msgs::Point::ConstPtr &msg) {
  double x = msg->x;
  double y = msg->y;

  geometry_msgs::Pose above_pose;
  above_pose.position.x = x;
  above_pose.position.y = y;
  above_pose.position.z = hover_z;
  above_pose.orientation = default_orientation;

  geometry_msgs::Pose grasp_pose = above_pose;
  grasp_pose.position.z = table_z;

  ROS_INFO("[auto_pick] Move above target (%.2f, %.2f)", x, y);
  arm_ptr->setPoseTarget(above_pose);
  MoveGroupInterface::Plan plan1;
  if (!(arm_ptr->plan(plan1) && arm_ptr->execute(plan1))) {
    ROS_WARN("[auto_pick] Failed move above");
    return;
  }

  ROS_INFO("[auto_pick] Opening gripper...");
  hand_ptr->setNamedTarget("open");
  hand_ptr->move();

  ROS_INFO("[auto_pick] Lowering to grasp...");
  arm_ptr->setPoseTarget(grasp_pose);
  MoveGroupInterface::Plan plan2;
  if (!(arm_ptr->plan(plan2) && arm_ptr->execute(plan2))) {
    ROS_WARN("[auto_pick] Failed lowering");
    return;
  }

  ROS_INFO("[auto_pick] Closing gripper...");
  hand_ptr->setNamedTarget("close");
  hand_ptr->move();

  ROS_INFO("[auto_pick] Moving to place position...");
  arm_ptr->setPoseTarget(place_pose);
  MoveGroupInterface::Plan plan3;
  if (!(arm_ptr->plan(plan3) && arm_ptr->execute(plan3))) {
    ROS_WARN("[auto_pick] Failed to lift");
  }

  ROS_INFO("[auto_pick] Opening gripper...");
  hand_ptr->setNamedTarget("open");
  hand_ptr->move();

  ROS_INFO("[auto_pick] Returning to start configuration...");
  arm_ptr->setJointValueTarget(start_joints);
  MoveGroupInterface::Plan plan4;
  if (!(arm_ptr->plan(plan4) && arm_ptr->execute(plan4))) {
    ROS_WARN("[auto_pick] Failed to return to start");
  }
}
