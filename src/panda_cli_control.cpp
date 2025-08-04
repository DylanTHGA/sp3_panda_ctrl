#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

#include "panda_control_utils.h"

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "panda_cli_control");
  ros::AsyncSpinner spinner(1);  // Use async spinner to handle callbacks
  spinner.start();
  ros::NodeHandle nh;

  // Initialize MoveGroup interfaces for arm and gripper
  MoveGroupInterface arm("panda_arm");
  MoveGroupInterface hand("panda_hand");
  PlanningSceneInterface planning_scene_interface;

  arm_ptr = &arm;
  hand_ptr = &hand;

  // Configure MoveIt planning behavior
  arm.setPlanningTime(15.0);             // Set max planning time
  arm.setMaxVelocityScalingFactor(0.5);  // Limit max speed of the robot arm
  hand.setMaxVelocityScalingFactor(0.5); // Limit max speed of the gripper

  // Initialize orientation and predefined poses (defined in panda_control_utils)
  initializeDefaultOrientation();
  initializePredefinedPoses();
  moveToStartPosition();  
  printHelp();            

  // Subscribe to auto_pick topic (does not work yet and will be handled in the MA)
  ros::Subscriber sub = nh.subscribe("auto_pick", 1, pickCallback);

  std::string line;
  while (ros::ok())
  {
    // Command-line interface loop
    std::cout << "\n> " << std::flush;
    if (!std::getline(std::cin, line))
      break;

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    // Display available commands
    if (cmd == "help")
    {
      printHelp();
    }
    // Move end-effector to specified Cartesian pose
    else if (cmd == "move_pose")
    {
      double x, y, z;
      if (!(iss >> x >> y >> z))
      {
        ROS_WARN("Usage: move_pose x y z");
        continue;
      }
      geometry_msgs::Pose target;
      target.position.x = x;
      target.position.y = y;
      target.position.z = z;
      target.orientation = default_orientation;
      arm.setPoseTarget(target);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
      {
        ROS_WARN("Planning failed");
      }
    }
    // Manually set default orientation (quaternion) needs to be used in the SP-System and added in the pickCallback
    else if (cmd == "set_orientation")
    {
      double qw, qx, qy, qz;
      if (!(iss >> qw >> qx >> qy >> qz))
      {
        ROS_WARN("Usage: set_orientation qw qx qy qz");
        continue;
      }
      default_orientation.w = qw;
      default_orientation.x = qx;
      default_orientation.y = qy;
      default_orientation.z = qz;
      ROS_INFO("Default orientation set.");
    }
    // Move joints to a manually specified configuration (degrees)
    else if (cmd == "move_joints")
    { 
      std::vector<double> joints_deg(7);
      for (int i = 0; i < 7; ++i)
        if (!(iss >> joints_deg[i]))
        {
          ROS_WARN("Usage: move_joints j1...j7 (degrees)");
          joints_deg.clear();
          break;
        }

      if (joints_deg.size() == 7)
      {
        std::vector<double> joints_rad(7);
        for (int i = 0; i < 7; ++i)
          joints_rad[i] = joints_deg[i] * M_PI / 180.0;  
        arm.setJointValueTarget(joints_rad);
        MoveGroupInterface::Plan plan;
        if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
        {
          ROS_WARN("Planning failed");
        }
      }
    }
    // Print current pose of end-effector
    else if (cmd == "print_pose")
    {
      auto ps = arm.getCurrentPose();
      const auto &p = ps.pose.position;
      const auto &o = ps.pose.orientation;
      ROS_INFO("Pose: x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z);
      ROS_INFO("Ori: w=%.3f x=%.3f y=%.3f z=%.3f", o.w, o.x, o.y, o.z);
    }
    // Print current joint values (in degrees)
    else if (cmd == "print_joints")
    {
      auto st = arm.getCurrentState();
      const auto *jmg = st->getJointModelGroup("panda_arm");
      std::vector<double> jp;
      st->copyJointGroupPositions(jmg, jp);
      for (size_t i = 0; i < jp.size(); ++i)
        ROS_INFO("Joint %zu: %.1f°", i + 1, jp[i] * 180.0 / M_PI);
    }
    // Open gripper 
    else if (cmd == "open_gripper")
    {
      hand.setNamedTarget("open");
      hand.move();
    }
    // Close gripper 
    else if (cmd == "close_gripper")
    {
      hand.setNamedTarget("close");
      hand.move();
    }
    // Stop all motion immediately
    else if (cmd == "stop")
    {
      arm.stop();
      hand.stop();
      ROS_INFO("Stopped");
    }
    // Move to predefined "observer" joint configuration
    else if (cmd == "goto_observer")
    {
      arm.setJointValueTarget(observer_joints);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
        ROS_WARN("Failed to reach observer joints");
    }
    // Move to predefined "place" pose
    else if (cmd == "goto_place")
    {
      arm.setPoseTarget(place_pose);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
        ROS_WARN("Failed place");
    }
    // Move back to predefined start pose
    else if (cmd == "goto_start")
    {
      arm.setJointValueTarget(start_joints);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
        ROS_WARN("Failed goto_start");
    }
    // Pick object at specified x/y coordinate on table surface
    else if (cmd == "pick_xy")
    {
      double x, y;
      if (!(iss >> x >> y))
      {
        ROS_WARN("Usage: pick_xy x y");
        continue;
      }
      geometry_msgs::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = table_z; 
      pickCallback(boost::make_shared<geometry_msgs::Point>(pt));
    }
    // Exit the program
    else if (cmd == "quit")
    {
      ROS_INFO("Shutdown");
      break;
    }
    // Handle unknown commands
    else if (!cmd.empty())
    {
      ROS_WARN("Unknown cmd '%s'. Type 'help'", cmd.c_str());
    }
  }

  ros::shutdown();
  return 0;
}
