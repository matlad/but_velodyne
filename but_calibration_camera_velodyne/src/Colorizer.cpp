/**
 * @file     Colorizer.cpp
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include "but_calibration_camera_velodyne/Colorizer.h"

#include <cstdlib>
#include <cstdio>
#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"

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
#include <but_calibration_camera_velodyne/macros.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace But::calibration_camera_velodyne;
using namespace But::calibration_camera_velodyne::Velodyne;

using namespace But::calibration_camera_velodyne;

void Colorizer::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,
													 sensor_msgs::image_encodings::BGR8);
  frame_rgb = cv_ptr->image;
}

void Colorizer::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  cv::Mat(3, 4, CV_64FC1, (void *) &msg->P).copyTo(projection_matrix);
  projection_matrix.convertTo(projection_matrix, CV_32FC1);
  cv::Mat(msg->D).copyTo(D);
  rotate(D, D, ROTATE_90_COUNTERCLOCKWISE);
  cv::Mat(3, 3, CV_64FC1, (void *) &msg->K).copyTo(K);

  ROS_DEBUG_STREAM("P: \n" << projection_matrix);
  ROS_DEBUG_STREAM("D: \n" << D);
  ROS_DEBUG_STREAM("K: \n" << K);
}

void Colorizer::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // if no rgb frame for coloring:
  if (frame_rgb.data == NULL) {
	return;
  }

  PointCloud<Velodyne::Point> pc;
  fromROSMsg(*msg, pc);

  // x := x, y := -z, z := y,
  Velodyne::Velodyne pointcloud = Velodyne::Velodyne(pc).transform(
	  0,
	  0,
	  0,
	  M_PI / 2,
	  0,
	  0
  );

  pointCloud = pointcloud;

  Mat tvec(1, 3, CV_32FC1, DoF.data());
  Mat rvec(1, 3, CV_32FC1, DoF.data() + 3);

  ROS_INFO_STREAM_ONCE("\nrvect: " << rvec << "\ntvect:" << tvec << "\n");






  rotate(frame_rgb, frame_rgb, ROTATE_90_CLOCKWISE);

  Mat Knew = K.clone();



//  cv::fisheye::undistortImage(
//          frame_r_rgb,
//          frame_rgb,
//          K,
//          D,
//          Knew
//  );
//


  cv::rectangle(frame_rgb, cv::Point(300, 100),cv::Point(500, 300),CV_RGB(255,0,0),CV_FILLED);
  cv::rectangle(frame_rgb, cv::Point(100, 200),cv::Point(300, 400),CV_RGB(0,255,0),CV_FILLED);
  cv::rectangle(frame_rgb, cv::Point(500, 200),cv::Point(700, 400),CV_RGB(0,0,255),CV_FILLED);
  cv::rectangle(frame_rgb, cv::Point(300, 400),cv::Point(500, 500),CV_RGB(255,0,255),CV_FILLED);

  //SHOW_IMAGE(frame_rgb,"undisort");



  PointCloud<PointXYZRGB>
	  color_cloud = colourByFishEye(frame_rgb, D, K, rvec, tvec);

  // reverse axix switching:
  Eigen::Affine3f transf = getTransformation(0, 0, 0, -M_PI / 2, 0, 0);
  transformPointCloud(color_cloud, color_cloud, transf);

  sensor_msgs::PointCloud2 color_cloud2;
  toROSMsg(color_cloud, color_cloud2);
  color_cloud2.header = msg->header;

  publisher.publish(color_cloud2);

}

PointCloud<PointXYZRGB> Colorizer::colourByFishEye(
	cv::Mat frame_rgb,
	cv::Mat D,
	cv::Mat K,
	cv::Mat rvec,
	cv::Mat tvec
) {
  PointCloud<PointXYZRGB> color_cloud;

  vector<Point3f> inputPointCloud;
  auto pointCloud = this->pointCloud.getPointCloud();

  for (auto point:pointCloud) {
	inputPointCloud.push_back(Point3f(point.x, point.y, point.z));
  }

  Mat projectedPoints;

  fisheye::projectPoints(
	  inputPointCloud,
	  projectedPoints,
	  rvec,
	  tvec,
	  K,
	  D
  );

  Vec3b rgbDefault(255, 0, 0);

  for (size_t iter = 0; iter < pointCloud.points.size(); iter++) {

	assert(iter <= INT_MAX);
	Point2i xy = projectedPoints.at<Point2f>(0, (int) iter);
	Vec3b rgb;

	if (xy.y < 0 || xy.x < 0 || xy.y > frame_rgb.rows ||
		xy.x > frame_rgb.cols) {
	  rgb = rgbDefault;
	} else {
	  rgb = Image::Image::atf(frame_rgb, xy);
	}

	PointXYZRGB pt_rgb(rgb.val[2], rgb.val[1], rgb.val[0]);
	pt_rgb.x = pointCloud.at(iter).x;
	pt_rgb.y = pointCloud.at(iter).y;
	pt_rgb.z = pointCloud.at(iter).z;

	color_cloud.push_back(pt_rgb);
  }
  return color_cloud;
}

Colorizer::Colorizer(const ros::Publisher &publisher,
					 const std::vector<float> &DoF)
	: publisher(publisher), DoF(DoF) {}

