/*
 * Velodyne.cpp
 *
 *  Created on: 26.11.2013
 *      Author: ivelas
 */

#include <vector>
#include <cmath>

#include "but_calibration_camera_velodyne/Velodyne.h"
#include "but_calibration_camera_velodyne/Exceptions.h"
#include "but_calibration_camera_velodyne/Image.h"

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <ros/assert.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;

namespace But::calibration_camera_velodyne
{

Velodyne::Velodyne::Velodyne(PointCloud<Point> _point_cloud) :
    point_cloud(_point_cloud)
{
  //getRings();
}

Velodyne::Velodyne Velodyne::Velodyne::transform(float x, float y, float z, float rot_x, float rot_y, float rot_z)
{
  Eigen::Affine3f transf = getTransformation(x, y, z, rot_x, rot_y, rot_z);
  PointCloud<Point> new_cloud;
  transformPointCloud(point_cloud, new_cloud, transf);
  return Velodyne(new_cloud);
}

Velodyne::Velodyne Velodyne::Velodyne::transform(vector<float> DoF)
{
  ROS_ASSERT(DoF.size() == 6);
  return transform(DoF[0], DoF[1], DoF[2], DoF[3], DoF[4], DoF[5]);
}

	Mat Velodyne::Velodyne::project(Mat projection_matrix,
									Rect frame,
									PointCloud<Point> *visible_points)
	{
		Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

		for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
			 pt < point_cloud.points.end(); pt++)
		{

			// behind the camera
			if (pt->z < 0)
			{
				continue;
			}

			float     intensity = pt->intensity;
			cv::Point xy        = Velodyne::project(*pt, projection_matrix);
			if (xy.inside(frame))
			{
				if (visible_points != NULL)
				{
					visible_points->push_back(*pt);
				}

				//cv::circle(plane, xy, 3, intensity, -1);
				plane.at<float>(xy) = intensity;
			}
		}

		Mat plane_gray;
		cv::normalize(plane, plane_gray, 0, 255, NORM_MINMAX, CV_8UC1);
		dilate(plane_gray, plane_gray, Mat());
		//Image::Image plane_img(plane_gray);
		//return plane_img.computeIDTEdgeImage();

		return plane_gray;
	}

	Mat
	Velodyne::Velodyne::project(Mat projection_matrix, Rect frame, Mat image)
	{
		Mat plane     = this->project(projection_matrix, frame, NULL);
		//equalizeHist(plane, plane);
		//equalizeHist(image, image);

		ROS_ASSERT(frame.width == image.cols && frame.height == image.rows);
		Mat empty     = Mat::zeros(frame.size(), CV_8UC1);

		Mat result_channel(frame.size(), CV_8UC3);
		Mat in[]      = {image, empty, plane};
		int from_to[] = {0, 0, 1, 1, 2, 2};
		mixChannels(in, 3, &result_channel, 1, from_to, 3);
		return result_channel;
	}

	vector<vector<Velodyne::Point *> > Velodyne::Velodyne::getRings()
	{
		vector<vector<Point *> > rings(Velodyne::Velodyne::RINGS_COUNT);
		for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
			 pt < point_cloud.points.end(); pt++)
		{
			ROS_ASSERT(pt->ring < RINGS_COUNT);
			pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);

			rings[pt->ring].push_back(&(*pt));
		}
		return rings;
	}

	void Velodyne::Velodyne::intensityByRangeDiff()
	{
		intensityByDiff(Processing::DISTORTIONS);
	}

	void Velodyne::Velodyne::intensityByIntensityDiff()
	{
		intensityByDiff(Processing::INTENSITY_EDGES);
	}

void Velodyne::Velodyne::intensityByDiff(Processing processing) {
  auto rings = this->getRings();
  vector<int> convolutionKernel = {-1, -1, 0, 1, 1};
	assert(convolutionKernel.size()%2 == 1);

  for (auto ring : rings) {
	Point *prev, *succ;
	vector<pair<Point *,float>> newIntensitys;

	if (ring.empty()) {
	  continue;
	}

	float last_intensity = (*ring.begin())->intensity;
	float new_intensity;

	for (auto pointPtPt = ring.begin() + 2; pointPtPt < ring.end() - 2; pointPtPt++) {

	  prev = *(pointPtPt - 1);
	  succ = *(pointPtPt + 1);

	  switch (processing) {
		case Processing::Z_DISTORTIONS:
		  new_intensity = 0;
		  for(int i = 0; i < convolutionKernel.size(); i++)
		  {
		    int shift = i - (convolutionKernel.size() / 2);
		    auto shiftPointPt = *(pointPtPt - shift);
			new_intensity += shiftPointPt->range * convolutionKernel[i];
		  }
		  new_intensity = abs(new_intensity);
		  newIntensitys.emplace_back(pair<Point *,float>(*pointPtPt,new_intensity));
		  break;
		case Processing::DISTORTIONS:
		  (*pointPtPt)->intensity =
			  MAX(
				  MAX(
				  	prev->range - (*pointPtPt)->range,
					succ->range - (*pointPtPt)->range
				  ),
				  0
			  ) * 10;

		  break;
		case Processing::INTENSITY_EDGES:
		  new_intensity =
			  MAX(
			  	MAX(
			  		last_intensity - (*pointPtPt)->intensity,
					succ->intensity - (*pointPtPt)->intensity
				),
				0
			  ) * 10;

		  last_intensity = (*pointPtPt)->intensity;
		  (*pointPtPt)->intensity = new_intensity;
		  break;
		case Processing::NONE:break;
		default:
		  throw NotImplementedException(
			  "Velodyne processing unknown.");
	  }
	}


	if(processing == Processing::Z_DISTORTIONS) {
	  for (auto x:newIntensitys) {
		x.first->intensity = x.second;
	  }
	}
	(*ring.begin())->intensity = (*(ring.begin()+3))->intensity;
	(*(ring.end() - 1))->intensity = (*(ring.end() - 3))->intensity;
	(*(ring.begin() +1))->intensity = (*(ring.begin()+3))->intensity;
	(*(ring.end() - 2))->intensity = (*(ring.end() - 3))->intensity;
  }
  normalizeIntensity(0.0, 1.0);
}

	PointCloud<PointXYZ> *Velodyne::Velodyne::toPointsXYZ()
	{
		PointCloud<PointXYZ>             *new_cloud = new PointCloud<PointXYZ>();
		for (PointCloud<Point>::iterator pt         = point_cloud.points
				.begin(); pt < point_cloud.points.end(); pt++)
		{
			new_cloud->push_back(PointXYZ(pt->x, pt->y, pt->z));
		}
		return new_cloud;
	}

PointCloud<PointXYZRGB> * Velodyne::Velodyne::toPointsXYZRGB() {

  auto new_cloud = new PointCloud<PointXYZRGB>();

  this->normalizeIntensity(0,1);


  float min_found = INFINITY;
  float max_found = -INFINITY;

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
	   pt < point_cloud.points.end(); pt++)
  {
	max_found = MAX(max_found, pt->range);
	min_found = MIN(min_found, pt->range);
  }



  for (auto point : point_cloud.points) {
    assert(point.intensity <= 1);
    uchar intensity = (point.intensity > 0.01) ? 255 : 0;
    auto range = (point.range - min_found) / (max_found - min_found) * 10;
    auto xyziPoint = PointXYZRGB(intensity,255 - intensity,0);
    xyziPoint.x = point.x;
    xyziPoint.y = point.y;
    xyziPoint.z = point.z;
	new_cloud->push_back(xyziPoint);
  }

  return new_cloud;
}

// all intensities to range min-max
	void Velodyne::Velodyne::normalizeIntensity(float min, float max)
	{
		float min_found = INFINITY;
		float max_found = -INFINITY;

		for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
			 pt < point_cloud.points.end(); pt++)
		{
			max_found = MAX(max_found, pt->intensity);
			min_found = MIN(min_found, pt->intensity);
		}

		for (PointCloud<Point>::iterator pt = point_cloud.points.begin();
			 pt < point_cloud.points.end(); pt++)
		{
			pt->intensity =
					(pt->intensity - min_found) / (max_found - min_found) *
					(max - min) + min;
//		cerr << pt->intensity << " ";
		}

//	cerr << endl;
	}

	/**
	 * @brief Vyfiltruje body jejichž intenzita je je nižší rovna hranici
	 * @param thresh hranice
	 * @return
	 */
	Velodyne::Velodyne Velodyne::Velodyne::threshold(float thresh)
	{
		PointCloud<Point> new_cloud;

		for (auto point : point_cloud.points)
		{
			if (point.intensity > thresh)
			{
				new_cloud.push_back(point);
			}
		}
		return Velodyne(new_cloud);
	}

	void Velodyne::Velodyne::detectPlanes(cv::Mat projection)
	{
		PointCloud<Point> visible_points;
		this->project(projection, Rect(0, 0, 640, 480), &visible_points);
		// ...
	}

	vector<Velodyne::Velodyne>
	Velodyne::Velodyne::depthSegmentation(int segment_counts)
	{
		vector<Velodyne> segments(segment_counts);

		Mat      ranges(point_cloud.size(), 1, CV_32FC1);
//	Mat indicies(point_cloud.size(), 1, CV_32SC1);
		for (int i = 0; i < point_cloud.size(); i++)
		{
			ranges.at<float>(i) = point_cloud[i].range;
		}
		//kmeans(ranges, segment_counts, indicies, TermCriteria(TermCriteria::MAX_ITER, 3, 0), 3, KMEANS_PP_CENTERS);

		Mat ranges_uchar;
		normalize(ranges, ranges_uchar, 0, 255, NORM_MINMAX, CV_8UC1);
		Mat indicies(point_cloud.size(), 1, CV_8UC1);
		cv::threshold(ranges_uchar,
					  indicies,
					  0,
					  1,
					  THRESH_BINARY + THRESH_OTSU);

		for (int i = 0; i < point_cloud.size(); i++)
		{
			segments[indicies.at<uchar>(i)].push_back(point_cloud[i]);
		}

		return segments;
	}

	PointCloud<PointXYZRGB>
	Velodyne::Velodyne::colour(cv::Mat frame_rgb, cv::Mat P)
	{
		PointCloud<PointXYZRGB> color_cloud;

		for (PointCloud<Point>::iterator pt = this->point_cloud.begin();
			 pt < this->point_cloud.end(); pt++)
		{

			Point2f xy = Velodyne::Velodyne::projectf(*pt, P);

			Vec3b       rgb = Image::Image::atf(frame_rgb, xy);
			PointXYZRGB pt_rgb(rgb.val[2], rgb.val[1], rgb.val[0]);
			pt_rgb.x = pt->x;
			pt_rgb.y = pt->y;
			pt_rgb.z = pt->z;

			color_cloud.push_back(pt_rgb);
		}
		return color_cloud;
	}

	PointCloud<PointXYZRGB>
	Velodyne::Velodyne::colourByFishEye(cv::Mat frame_rgb,
										cv::Mat D,
										cv::Mat K,
										cv::Mat rvec,
										cv::Mat tvec)
	{
		PointCloud<PointXYZRGB> color_cloud;

		vector<Point3f> inputPointCloud;

		for (auto point:this->point_cloud)
		{
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

	Vec3b	rgbDefault(255,0,0);

		for (size_t iter = 0; iter < this->point_cloud.points.size(); iter++)
		{

			assert(iter <= INT_MAX);
			Point2i xy = projectedPoints.at<Point2f>(0, (int) iter);
			Vec3b       rgb;

			if (xy.y < 0 || xy.x<0 || xy.y > frame_rgb.rows || xy.x > frame_rgb.cols)
			{
				     rgb = rgbDefault;
			}
			else
			{
				      rgb = Image::Image::atf(frame_rgb, xy);
			}

			PointXYZRGB pt_rgb(rgb.val[2], rgb.val[1], rgb.val[0]);
			pt_rgb.x = this->point_cloud.at(iter).x;
			pt_rgb.y = this->point_cloud.at(iter).y;
			pt_rgb.z = this->point_cloud.at(iter).z;

			color_cloud.push_back(pt_rgb);
		}
		return color_cloud;
	}

Velodyne::Velodyne Velodyne::Velodyne::transform(cv::Mat tvec, cv::Mat rvec) {
  assert(tvec.cols == 1);
  assert(tvec.rows == 3);
  assert(rvec.cols == 1);
  assert(rvec.rows == 3);
  return transform(
	  tvec.at<float>(0, 0), tvec.at<float>(0, 1), tvec.at<float>(0, 2),
	  rvec.at<float>(0, 0), rvec.at<float>(0, 1), rvec.at<float>(0, 2)
  );
}

void Velodyne::Velodyne::view(float trashhold, const char *windowTitle) {
  auto cloud_ptr = ::pcl::PointCloud<pcl::PointXYZRGB>::Ptr(this->toPointsXYZRGB());

  boost::shared_ptr<::pcl::visualization::PCLVisualizer> viewer(
	  new ::pcl::visualization::PCLVisualizer(windowTitle)
  );

  viewer->setBackgroundColor(0, 0, 0);
  ::pcl::visualization::PointCloudColorHandlerRGBField<::pcl::PointXYZRGB> rgb(cloud_ptr);

  viewer->addPointCloud<::pcl::PointXYZRGB>(cloud_ptr, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(::pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(0.3);
  viewer->initCameraParameters();
  while (!viewer->wasStopped()) {
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(
		100000));
  }
}

void Velodyne::Velodyne::viewMarker(vector<Point3f> centers3D, vector<float> radii3D, const char *windowTitle) {
  auto cloud_ptr = ::pcl::PointCloud<pcl::PointXYZRGB>::Ptr(this->toPointsXYZRGB());

  boost::shared_ptr<::pcl::visualization::PCLVisualizer> viewer(
	  new ::pcl::visualization::PCLVisualizer(windowTitle)
  );



  viewer->setBackgroundColor(0, 0, 0);
  ::pcl::visualization::PointCloudColorHandlerRGBField<::pcl::PointXYZRGB> rgb(cloud_ptr);

  viewer->addPointCloud<::pcl::PointXYZRGB>(cloud_ptr, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(::pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(0.3);


  ModelCoefficients circle_1;
  circle_1.values.resize (7);    // We need 7 values
  circle_1.values[0] = centers3D[0].x;
  circle_1.values[1] = centers3D[0].y;
  circle_1.values[2] = centers3D[0].z;
  circle_1.values[3] = 0;
  circle_1.values[4] = 0;
  circle_1.values[5] = 0.001;
  circle_1.values[6] = radii3D[0];

  ModelCoefficients circle_2;
  circle_2.values.resize (7);    // We need 7 values
  circle_2.values[0] = centers3D[1].x;
  circle_2.values[1] = centers3D[1].y;
  circle_2.values[2] = centers3D[1].z;
  circle_2.values[3] = 0;
  circle_2.values[4] = 0;
  circle_2.values[5] = 0.001;
  circle_2.values[6] = radii3D[1];

  ModelCoefficients circle_3;
  circle_3.values.resize (7);    // We need 7 values
  circle_3.values[0] = centers3D[2].x;
  circle_3.values[1] = centers3D[2].y;
  circle_3.values[2] = centers3D[2].z;
  circle_3.values[3] = 0;
  circle_3.values[4] = 0;
  circle_3.values[5] = 0.001;
  circle_3.values[6] = radii3D[2];

  ModelCoefficients circle_4;
  circle_4.values.resize (7);    // We need 7 values
  circle_4.values[0] = centers3D[3].x;
  circle_4.values[1] = centers3D[3].y;
  circle_4.values[2] = centers3D[3].z;
  circle_4.values[3] = 0;
  circle_4.values[4] = 0;
  circle_4.values[5] = 0.001;
  circle_4.values[6] = radii3D[3];

  viewer->addCylinder(circle_4,"c4");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "c4");
  viewer->addCylinder(circle_3,"c3");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "c3");
  viewer->addCylinder(circle_2,"c2");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "c2");
  viewer->addCylinder(circle_1,"c1");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0, "c1");

  viewer->initCameraParameters();
  while (!viewer->wasStopped()) {
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(
		100000));
  }
}

Velodyne::Velodyne::Velodyne(const Velodyne &orig) {
   point_cloud = VPointCloud(orig.point_cloud);
}

}
