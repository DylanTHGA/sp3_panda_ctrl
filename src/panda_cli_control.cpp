#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;


MoveGroupInterface* arm_ptr  = nullptr;
MoveGroupInterface* hand_ptr = nullptr;

double table_z = 0.79 + 0.118;
double hover_z = table_z + 0.2;

geometry_msgs::Quaternion default_orientation;
geometry_msgs::Pose       place_pose;




std::vector<double> start_joints = {
  0.0, -45.0 * M_PI / 180.0, 0.0, -135.0 * M_PI / 180.0, 0.0, 90.0 * M_PI / 180.0, 45.0 * M_PI / 180.0
};
std::vector<double> observer_joints = {
  0.0 * M_PI / 180.0, -50.0 * M_PI / 180.0, 0.0 * M_PI / 180.0, -100.0 * M_PI / 180.0,
  0.0 * M_PI / 180.0, 60.0 * M_PI / 180.0, 45.0 * M_PI / 180.0
};


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

void initializeDefaultOrientation() {
  default_orientation.w = 0.0;
  default_orientation.x = 1.0;
  default_orientation.y = 0.0;
  default_orientation.z = 0.0;
}

void initializePredefinedPoses() {
  place_pose.position.x = 0.0;
  place_pose.position.y = - 0.6;
  place_pose.position.z = table_z;
  place_pose.orientation = default_orientation;
}

void moveToStartPosition() {
  arm_ptr->setJointValueTarget(start_joints);
  MoveGroupInterface::Plan plan;
  if (!(arm_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm_ptr->execute(plan))) {
    ROS_WARN("Failed goto_start");
  }
}

// Pick-Callback (kann via Topic oder CLI aufgerufen werden)
void pickCallback(const geometry_msgs::Point::ConstPtr &msg) {
  const double x = msg->x;
  const double y = msg->y;

  geometry_msgs::Pose above_pose;
  above_pose.position.x = x;
  above_pose.position.y = y;
  above_pose.position.z = hover_z;                
  above_pose.orientation = default_orientation;

  geometry_msgs::Pose above_place_pose;
  above_place_pose.position.x = 0.0;
  above_place_pose.position.y = -0.6;
  above_place_pose.position.z = hover_z;                
  above_place_pose.orientation = default_orientation;
  

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

  ROS_INFO("[auto_pick] Lifting back to hover...");
  arm_ptr->setPoseTarget(above_pose);
  MoveGroupInterface::Plan plan_lift;
  if (!(arm_ptr->plan(plan_lift) && arm_ptr->execute(plan_lift))) {
    ROS_WARN("[auto_pick] Failed to lift back to hover");
    return;
  }

  ROS_INFO("[auto_pick] Moving to place position...");
  arm_ptr->setPoseTarget(place_pose);
  MoveGroupInterface::Plan plan3;
  if (!(arm_ptr->plan(plan3) && arm_ptr->execute(plan3))) {
    ROS_WARN("[auto_pick] Failed move to place");
    return;
  }

  ROS_INFO("[auto_pick] Opening gripper...");
  hand_ptr->setNamedTarget("open");
  hand_ptr->move();

  ROS_INFO("[auto_pick] Moving to above place position...");
  arm_ptr->setPoseTarget(above_place_pose);
  MoveGroupInterface::Plan plan4;
  if (!(arm_ptr->plan(plan4) && arm_ptr->execute(plan4))) {
    ROS_WARN("[auto_pick] Failed move to place");
    return;
  }

  ROS_INFO("[auto_pick] Returning to start configuration...");
  arm_ptr->setJointValueTarget(start_joints);
  MoveGroupInterface::Plan plan5;
  if (!(arm_ptr->plan(plan5) && arm_ptr->execute(plan5))) {
    ROS_WARN("[auto_pick] Failed to return to start");
  }
}

// ----------------- main -----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "panda_cli_control");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;

  MoveGroupInterface arm("panda_arm");
  MoveGroupInterface hand("panda_hand");
  PlanningSceneInterface planning_scene_interface;

  arm_ptr = &arm;
  hand_ptr = &hand;

  arm.setPlanningTime(15.0);
  arm.setMaxVelocityScalingFactor(0.5);
  hand.setMaxVelocityScalingFactor(0.5);

  initializeDefaultOrientation();
  initializePredefinedPoses();
  moveToStartPosition();
  printHelp();

 // ros::Subscriber sub = nh.subscribe("auto_pick", 1, pickCallback);

  std::string line;
  while (ros::ok())
  {
    std::cout << "\n> " << std::flush;
    if (!std::getline(std::cin, line)) break;

    std::istringstream iss(line);
    std::string cmd; iss >> cmd;

    if (cmd == "help") {
      printHelp();
    }
    else if (cmd == "move_pose") {
      double x,y,z;
      if (!(iss >> x >> y >> z)) { ROS_WARN("Usage: move_pose x y z"); continue; }
      geometry_msgs::Pose target;
      target.position.x = x; target.position.y = y; target.position.z = z;
      target.orientation = default_orientation;
      arm.setPoseTarget(target);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
        ROS_WARN("Planning failed");
    }
    else if (cmd == "set_orientation") {
      double qw,qx,qy,qz;
      if (!(iss >> qw >> qx >> qy >> qz)) { ROS_WARN("Usage: set_orientation qw qx qy qz"); continue; }
      default_orientation.w = qw; default_orientation.x = qx;
      default_orientation.y = qy; default_orientation.z = qz;
      ROS_INFO("Default orientation set.");
    }
    else if (cmd == "move_joints") {
      std::vector<double> jd(7);
      for (int i=0;i<7;++i) if (!(iss >> jd[i])) { ROS_WARN("Usage: move_joints j1...j7 (deg)"); jd.clear(); break; }
      if (!jd.empty()) {
        std::vector<double> jr(7);
        for (int i=0;i<7;++i) jr[i] = jd[i] * M_PI / 180.0;
        arm.setJointValueTarget(jr);
        MoveGroupInterface::Plan plan;
        if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
          ROS_WARN("Planning failed");
      }
    }
    else if (cmd == "print_pose") {
      auto ps = arm.getCurrentPose();
      const auto &p = ps.pose.position;
      const auto &o = ps.pose.orientation;
      ROS_INFO("Pose: x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z);
      ROS_INFO("Ori:  w=%.3f x=%.3f y=%.3f z=%.3f", o.w, o.x, o.y, o.z);
    }
    else if (cmd == "print_joints") {
      auto st = arm.getCurrentState();
      const auto *jmg = st->getJointModelGroup("panda_arm");
      std::vector<double> jp;
      st->copyJointGroupPositions(jmg, jp);
      for (size_t i=0;i<jp.size();++i)
        ROS_INFO("Joint %zu: %.1f°", i+1, jp[i] * 180.0 / M_PI);
    }
    else if (cmd == "open_gripper")  { hand.setNamedTarget("open");  hand.move(); }
    else if (cmd == "close_gripper") { hand.setNamedTarget("close"); hand.move(); }
    else if (cmd == "stop") { arm.stop(); hand.stop(); ROS_INFO("Stopped"); }
    else if (cmd == "goto_observer") {
      arm.setJointValueTarget(observer_joints);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
        ROS_WARN("Failed observer");
    }
    else if (cmd == "goto_place") {
      arm.setPoseTarget(place_pose);
      MoveGroupInterface::Plan plan;
      if (!(arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS && arm.execute(plan)))
        ROS_WARN("Failed place");
    }
    else if (cmd == "goto_start") {
      moveToStartPosition();
    }
    else if (cmd == "pick_xy") {
      double x,y; if (!(iss >> x >> y)) { ROS_WARN("Usage: pick_xy x y"); continue; }
      geometry_msgs::Point pt; pt.x=x; pt.y=y; pt.z=table_z;
      pickCallback(boost::make_shared<geometry_msgs::Point>(pt));
    }
    else if (cmd == "quit") { ROS_INFO("Shutdown"); break; }
    else if (!cmd.empty())  { ROS_WARN("Unknown cmd '%s'. Type 'help'", cmd.c_str()); }
  }

  ros::shutdown();
  return 0;
}
