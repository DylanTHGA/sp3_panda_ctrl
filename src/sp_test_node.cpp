#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <limits>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sp_test_node");
  ros::NodeHandle nh;
  ros::Publisher pick_pub = nh.advertise<geometry_msgs::Point>("auto_pick", 10);  

  while (ros::ok())
  {
    double x, y;

    std::cout << "\nEnter target x y (e.g. 0.4 0.2), or Ctrl+C to quit:\n> ";
    std::cin >> x >> y;

    // Check for invalid input
    if (std::cin.fail()) {
      std::cin.clear(); // reset error state
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard bad input
      std::cerr << "Invalid input. Please enter two numbers.\n";
      continue;
    }

    geometry_msgs::Point target;
    target.x = x;
    target.y = y;

    ROS_INFO("Sending target: x=%.2f y=%.2f", target.x, target.y);
    pick_pub.publish(target);

    ros::spinOnce();
  }

  return 0;
}
