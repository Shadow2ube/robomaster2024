#include <iostream>
#include <thread>
#include <chrono>
#include <string>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>

#include <cv_bridge/cv_bridge.h>

geometry_msgs::Pose make_pose(float x, float y, float z, float ax, float ay, float az, float aw = 0) {
  auto p = geometry_msgs::Pose();
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = ax;
  p.orientation.y = ay;
  p.orientation.z = az;
  p.orientation.w = aw;
  return p;
}

void detect_callback(sensor_msgs::Image) {
  std::cout << "got an image" << std::endl;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "Invalid number of arguments. Expected 2" << std::endl;
    return 1;
  }

  double precision;
  std::string outpath;

  try {
    precision = atof(argv[1]);
    outpath = argv[2];
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
  }

  ros::init(argc, argv, "detect");
  ros::NodeHandle n;

  ros::Publisher target_pub = n.advertise<geometry_msgs::Pose>("found/target", 10);
  ros::Publisher found_pub = n.advertise<geometry_msgs::PoseArray>("found/all", 10);
  ros::Publisher img_pub = n.advertise<geometry_msgs::PoseArray>("camera/detection", 10);

  ros::Subscriber image_sub = n.subscribe(outpath, 10, detect_callback);
  ros::spin()

  return 0;
}
