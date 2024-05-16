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

#include <onnxruntime_cxx_api.h>

using namespace cv;
using namespace cv::dnn;

std::string model_path = "/opt/detect/model.onnx";
Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "YOLOv8");
Ort::SessionOptions options;
Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(options, 0));
Ort::Session session Ort::Session(env, model_path.c_str(), options);

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
