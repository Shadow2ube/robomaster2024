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
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace cv::dnn;


ros::Publisher target_pub;
ros::Publisher found_pub;
ros::Publisher img_pub;

int SCORE_THRESHOLD = 1;

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

void detect_callback(sensor_msgs::Image img) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  float x_factor = input_image.cols / INPUT_WIDTH;
  float y_factor = input_image.rows / INPUT_HEIGHT;
  float *data = (float *)outputs[0].data;
  const int dimensions = 85;

  const int rows = 25200;
  for (int i = 0; i < rows; ++i) {
    float confidence = data[4];
    if (confidence >= SCORE_THRESHOLD) {
      float *classes_scores = data + 5;
      Mat scores(1, class_name.size(), CV_32FC1, classes_scores);

      Point class_id;
      double max_class_score;
      minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

      if (max_class_score > SCORE_THRESHOLD) {

        float cx = data[0];
        float cy = data[1];

        ROS_INFO("object of ID: %i (%f) at (%f, %f)", class_id, max_class_score, cx, cy)
      }
    }
    // Jump to the next row.
    data += 85;
  }

  img_pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv) {
  if (argc != 3) {
    ROS_ERROR("Invalid number of arguments. Expected 2");
    return 1;
  }

  double precision;
  std::string outpath;

  try {
    SCORE_THRESHOLD = atof(argv[1]);
    outpath = argv[2];
  } catch (std::exception &e) {
    ROS_ERROR("%s", e.what());
    return 2;
  }

  ros::init(argc, argv, "detect");
  ros::NodeHandle n;

  target_pub = n.advertise<geometry_msgs::Pose>("found/target", 10);
  found_pub = n.advertise<geometry_msgs::PoseArray>("found/all", 10);
  img_pub = n.advertise<sensor_msgs::Image>("camera/detection", 10);

  ros::Subscriber image_sub = n.subscribe(outpath, 1, detect_callback);

  ros::spin();

  return 0;
}
