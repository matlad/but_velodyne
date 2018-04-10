/*
 * manual_calibration.cpp
 *
 *  Created on: 27.2.2014
 *      Author: ivelas
 */

#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"
#include "but_calibration_camera_velodyne/Colorizer.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/tf.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>
using namespace But::calibration_camera_velodyne;
using std::string;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "coloring_node");

	string CAMERA_FRAME_TOPIC;
	string CAMERA_INFO_TOPIC;
	string VELODYNE_TOPIC;
	string VELODYNE_COLOR_TOPIC;
	std::vector<float> sixDoF;

	ros::NodeHandle n;
	n.getParam("/but_calibration_camera_velodyne/camera_frame_topic", CAMERA_FRAME_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/camera_info_topic", CAMERA_INFO_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/velodyne_topic", VELODYNE_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/velodyne_color_topic", VELODYNE_COLOR_TOPIC);
	n.getParam("/but_calibration_camera_velodyne/6DoF", sixDoF);

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(VELODYNE_COLOR_TOPIC, 1);

	auto colorizer = Colorizer(pub, sixDoF);

	// Subscribe input camera image
	image_transport::ImageTransport it(n);
	image_transport::Subscriber     sub = it.subscribe(
			CAMERA_FRAME_TOPIC,
			10,
			&Colorizer::imageCallback,
			&colorizer
	);

	ros::Subscriber info_sub = n.subscribe(
			CAMERA_INFO_TOPIC,
			10,
			&Colorizer::cameraInfoCallback,
			&colorizer
	);

	ros::Subscriber pc_sub = n.subscribe<sensor_msgs::PointCloud2>(
			VELODYNE_TOPIC,
			1,
			&Colorizer::pointCloudCallback,
			&colorizer
	);

	ros::spin();

	return EXIT_SUCCESS;
}
