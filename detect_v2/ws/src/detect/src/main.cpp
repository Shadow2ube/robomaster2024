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

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  
  ros::Rate loop_rate(10);
  
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
  
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
  
    ROS_INFO("%s", msg.data.c_str());
  
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
    */
    chatter_pub.publish(msg);
 
    ros::spinOnce();
 
    loop_rate.sleep();
    ++count;
  }

  using namespace std::chrono_literals;
  while (true) {
    std::this_thread::sleep_for(2ms);
  }

  return 0;
}
