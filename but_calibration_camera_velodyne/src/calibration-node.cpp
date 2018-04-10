#include <cstdlib>
#include <cstdio>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "but_calibration_camera_velodyne/Calibrator.h"

#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>

#include <camera_info_manager/camera_info_manager.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <ros/console.h>

#define BREAK 1

using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;
using namespace But::calibration_camera_velodyne;

Mat projection_matrix;
Mat frame_rgb, frame_r_rgb, undisort2, diff;
Mat K;
Velodyne::Velodyne pointcloud;
bool doRefinement = false;

#include "but_calibration_camera_velodyne/RosCalibratorWrapper.h"

static const char *const NODE_NAME = "calibration_node";
using But::calibration_camera_velodyne::Calibrator;

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	ros::console::notifyLoggerLevelsChanged();
  }

  int c;
  while ((c = getopt(argc, argv, "r")) != -1) {
	switch (c) {
	  case 'r':doRefinement = true;
		break;
	  default:return EXIT_FAILURE;
	}
  }

  string CAMERA_FRAME_TOPIC;
  string CAMERA_INFO_TOPIC;
  string VELODYNE_TOPIC;

  // marker properties:
  float STRAIGHT_DISTANCE; // 23cm
  float RADIUS; // 8.25cmBREAK

  ros::NodeHandle n;
  n.getParam(
	  "/but_calibration_camera_velodyne/camera_frame_topic",
	  CAMERA_FRAME_TOPIC
  );
  n.getParam(
	  "/but_calibration_camera_velodyne/camera_info_topic",
	  CAMERA_INFO_TOPIC
  );
  n.getParam(
	  "/but_calibration_camera_velodyne/velodyne_topic",
	  VELODYNE_TOPIC
  );
  n.getParam(
	  "/but_calibration_camera_velodyne/marker/circles_distance",
	  STRAIGHT_DISTANCE
  );
  n.getParam(
	  "/but_calibration_camera_velodyne/marker/circles_radius",
	  RADIUS
  );

  ROS_INFO_STREAM("Subscribe");

  message_filters::Subscriber<sensor_msgs::Image>
	  image_sub(n, CAMERA_FRAME_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo>
	  info_sub(n, CAMERA_INFO_TOPIC, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2>
	  cloud_sub(n, VELODYNE_TOPIC, 1);

  ROS_INFO_STREAM(
	  "Subscribed:"
		  << "\n\t" << CAMERA_FRAME_TOPIC
		  << "\n\t" << CAMERA_INFO_TOPIC
		  << "\n\t" << VELODYNE_TOPIC
  );

  typedef sync_policies::ApproximateTime<sensor_msgs::Image,
										 sensor_msgs::CameraInfo,
										 sensor_msgs::PointCloud2> MySyncPolicy;

  Synchronizer<MySyncPolicy>
	  sync(MySyncPolicy(10), image_sub, info_sub, cloud_sub);

  RosCalibratorWrapper calibrator(STRAIGHT_DISTANCE, RADIUS);

  sync.registerCallback(boost::bind(
	  &RosCalibratorWrapper::callback,
	  &calibrator,
	  _1,
	  _2,
	  _3
  ));

  ros::spin();

  return EXIT_SUCCESS;
}
