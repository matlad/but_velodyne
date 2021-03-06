/*
 * Calibration3DMarker.cpp
 *
 *  Created on: 2.4.2014
 *      Author: ivelas
 */

#include "but_calibration_camera_velodyne/Calibration3DMarker.h"

#include <ros/assert.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;

namespace but::calibration_camera_velodyne {

Calibration3DMarker::Calibration3DMarker(
    cv::Mat _frame_gray,
    cv::Mat _P,
    ::PointCloud<velodyne::Point> _pc,
    double _circ_distance,
    double _radius
) :
    frame_gray(_frame_gray),
    P(_P),
    pc(_pc),
    circ_distance(_circ_distance),
    radius(_radius) {

  velodyne::Velodyne scan(pc);

  // Identifikace vertikálních hran ve scanu
  scan.intensityByRangeDiff();
  PointCloud<velodyne::Point> visible_cloud;


  //TODO remove manipulaci s P

  float a = P.at<float>(0, 0);
  float b = P.at<float>(0, 2);
  float c = P.at<float>(1, 1);
  float d = P.at<float>(1, 2);

  P.at<float>(0, 0) = (float) 4.13365814e+02;
  P.at<float>(0, 2) = (float) 3.29170532e+02;
  P.at<float>(1, 1) = (float) 3.82713135e+02;
  P.at<float>(1, 2) = (float) 3.22920532e+02;

  // Výřez
  scan.project(P, Rect(0, 0, 640, 480), &visible_cloud);
  velodyne::Velodyne visible_scan(visible_cloud);
  visible_scan.normalizeIntensity();
  //visible_scan.view();

  P.at<float>(0, 0) = a;
  P.at<float>(0, 2) = b;
  P.at<float>(1, 1) = c;
  P.at<float>(1, 2) = d;

  // Odfiltrování bodů které nejsou hranou
  velodyne::Velodyne thresholded_scan = visible_scan.threshold(0.1);
  PointCloud<PointXYZ>::Ptr xyz_cloud_ptr(thresholded_scan.toPointsXYZ());

  //thresholded_scan.view();

  SampleConsensusModelPlane<PointXYZ>::Ptr model_p(
      new ::SampleConsensusModelPlane<PointXYZ>(xyz_cloud_ptr));
  RandomSampleConsensus<PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(0.05);
  ransac.computeModel();

  std::vector<int> inliers_indicies;
  ransac.getInliers(inliers_indicies);

  copyPointCloud<PointXYZ>(*xyz_cloud_ptr, inliers_indicies, plane);

  // ---------------- REMOVE LINES ----------------
  ROS_DEBUG("REMOVE LINES");

  for (int i = 0; i < 2; i++) {
    PointCloud<PointXYZ>::Ptr plane_ptr(new PointCloud<PointXYZ>(plane));
    SampleConsensusModelLine<PointXYZ>::Ptr model_l(
        new SampleConsensusModelLine<PointXYZ>(plane_ptr));
    RandomSampleConsensus<PointXYZ> ransac_l(model_l);
    ransac_l.setDistanceThreshold(0.02);
    ransac_l.computeModel();
    vector<int> line_inliers;
    ransac_l.getInliers(line_inliers);
    if (line_inliers.empty()) {
      continue;
    }
    PointCloud<PointXYZ> plane_no_line;
    remove_inliers(*plane_ptr, line_inliers, plane_no_line);
    plane = plane_no_line;
    // Velodyne::Velodyne::view(plane_ptr,"remover lines");
  }

}

bool Calibration3DMarker::detectCirclesInImage(
    vector<Point2f> &centers,
    vector<double> &radiuses
) {
  image::Image img(frame_gray);
  image::Image img_edge(img.computeEdgeImage());
  return img_edge.detect4Circles(Calibration3DMarker::CANNY_THRESH,
                                 Calibration3DMarker::CENTER_THRESH_DISTANCE,
                                 centers,
                                 radiuses);
}

/**
 * @brief
 * @param [out]centers středy detekovyných kruhů
 * @param [out]radiuses poloměry detekovaných kruhů
 * @return
 */
bool Calibration3DMarker::detectCirclesInPointCloud(
    vector<Point3f> &centers,
    vector<double> &radiuses
) {
  PointCloud<PointXYZ>::Ptr detection_cloud(new PointCloud<PointXYZ>);
  *detection_cloud += this->plane;

  double tolerance = 0.03; // 3cm
  int round = 1;
  vector<PointXYZ> spheres_centers;
  bool detected = false;
  for (int iterations = 0; iterations < 64; iterations++) {
    /*cerr << endl << " =========== ROUND " << round++ << " =========== "
     << endl << endl;
     cerr << "detection_cloud size: " << detection_cloud->size() << endl;*/
    spheres_centers = detect4spheres(detection_cloud, radiuses);

    if (spheres_centers.size() == 4) {
      order4spheres(spheres_centers);
      /*cerr << "ordered centers: " << endl;
       for (size_t i = 0; i < spheres_centers.size(); i++) {
       cerr << spheres_centers[i] << endl;
       }*/
      if (verify4spheres(spheres_centers, this->circ_distance, tolerance)) {
        spheres_centers = refine4centers(spheres_centers, detection_cloud);
        detected = true;
        break;
      }
    }
    vector<PointXYZ> possible_centers =
        generate_possible_centers(spheres_centers, this->circ_distance);
    generate_possible_points(this->plane,
                             detection_cloud,
                             possible_centers,
                             this->circ_distance,
                             0.01);
  }

  if (!detected) {
    return false;
  }

  for (size_t i = 0; i < spheres_centers.size(); i++) {
    centers.push_back(Point3f(spheres_centers[i].x,
                              spheres_centers[i].y,
                              spheres_centers[i].z));
  }
  return true;
}

vector<PointXYZ> Calibration3DMarker::detect4spheres(
    PointCloud<PointXYZ>::Ptr plane,
    vector<double> &radiuses
) {

  radiuses.clear();
  vector<PointXYZ> centers;
  std::vector<int> inliers_indicies;
  PointCloud<PointXYZ> *four_spheres = new PointCloud<PointXYZ>();
  double tolerance = 0.02;
  for (int i = 0; i < 4; i++) {
    SampleConsensusModelSphere<PointXYZ>::Ptr model_s(
        new SampleConsensusModelSphere<PointXYZ>(plane));
    model_s->setRadiusLimits(0.08, 0.09);
    RandomSampleConsensus<PointXYZ> ransac_sphere(model_s);
    ransac_sphere.setDistanceThreshold(tolerance);
    ransac_sphere.computeModel();
    inliers_indicies.clear();
    ransac_sphere.getInliers(inliers_indicies);

    if (inliers_indicies.size() == 0) {
      continue;
    }
    Eigen::VectorXf coeficients;
    ransac_sphere.getModelCoefficients(coeficients);
    //cerr << i + 1 << ". circle: " << coeficients << endl << endl;

    PointCloud<PointXYZ>::Ptr outliers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr inliers(new PointCloud<PointXYZ>);
    remove_inliers<PointXYZ>(*plane, inliers_indicies, *outliers);
    copyPointCloud<PointXYZ>(*plane, inliers_indicies, *inliers);
    plane = outliers;
    //Velodyne::Velodyne::view(plane);

    *four_spheres += *inliers;
    PointXYZ middle(coeficients(0), coeficients(1), coeficients(2));
    four_spheres->push_back(middle);
    centers.push_back(middle);

    double radius = coeficients(3);
    radiuses.push_back(radius);
  }
  PointCloud<PointXYZ>::Ptr four_spheres_ptr(four_spheres);
  return centers;
}

/*
 * Indexes of circles in marker:
 *
 * 0 1
 * 2 3
 */
bool orderX(PointXYZ p1, PointXYZ p2) {
  return p1.x < p2.x;
}

bool orderY(PointXYZ p1, PointXYZ p2) {
  return p1.y < p2.y;
}

void Calibration3DMarker::order4spheres(vector<PointXYZ> &spheres_centers) {
  ROS_ASSERT(spheres_centers.size() == 4);
  sort(spheres_centers.begin(), spheres_centers.end(), orderY);
  sort(spheres_centers.begin(), spheres_centers.begin() + 2, orderX);
  sort(spheres_centers.begin() + 2, spheres_centers.begin() + 4, orderX);
}

double euclid_dist(const PointXYZ p1, const PointXYZ p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

bool Calibration3DMarker::verify4spheres(
    const vector<PointXYZ> &spheres_centers,
    double straight_distance,
    double delta
) {
  ROS_ASSERT(spheres_centers.size() == 4);

  vector<pair<int, int> > neighbour_indexes;
  neighbour_indexes.push_back(pair<int, int>(0, 1));
  neighbour_indexes.push_back(pair<int, int>(1, 3));
  neighbour_indexes.push_back(pair<int, int>(3, 2));
  neighbour_indexes.push_back(pair<int, int>(2, 0));

  bool res = true;
  for (vector<pair<int, int> >::iterator neighbors = neighbour_indexes.begin();
       neighbors < neighbour_indexes.end();
       neighbors++) {
    double error = abs(
        euclid_dist(spheres_centers[neighbors->first],
                    spheres_centers[neighbors->second]) - straight_distance);
    //cerr << "error: " << error << endl;
    if (error > delta)
      res = false;
  }
  return res;
}

/*
 * All points around the all found centers:
 * x x x
 * x   x
 * x x x
 */
vector<PointXYZ> Calibration3DMarker::generate_possible_centers(
    const vector<PointXYZ> &spheres_centers,
    double straight_distance
) {
  vector<PointXYZ> possible_centers;

  for (auto c = spheres_centers.begin(); c < spheres_centers.end(); c++) {
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx || dy) { // omitting found center (may be false detection)
          PointXYZ new_center = *c;
          new_center.x += dx * straight_distance;
          new_center.y += dy * straight_distance;
          possible_centers.push_back(new_center);
        }
      }
    }
  }

  return possible_centers;
}

void Calibration3DMarker::generate_possible_points(
    PointCloud<PointXYZ> &plane,
    PointCloud<PointXYZ>::Ptr detection_cloud,
    const vector<PointXYZ> &possible_centers,
    double radius,
    double tolerance
) {

  detection_cloud->clear();
  for (PointCloud<PointXYZ>::iterator pt = plane.begin(); pt < plane.end();
       pt++) {
    int votes = 0;
    for (vector<PointXYZ>::const_iterator center = possible_centers.begin();
         center < possible_centers.end();
         center++) {
      if (euclid_dist(*pt, *center) < radius + tolerance) {
        votes++;
      }
    }
    if (votes > 0) {
      detection_cloud->push_back(*pt);
    }
  }
}

vector<PointXYZ> Calibration3DMarker::refine4centers(
    vector<PointXYZ> centers,
    PointCloud<PointXYZ>::Ptr detection_cloud
) {

  float z_coord = 0;
  for (PointCloud<PointXYZ>::iterator pt = detection_cloud->begin();
       pt < detection_cloud->end(); pt++) {
    z_coord += pt->z;
  }
  z_coord /= detection_cloud->size();

  for (vector<PointXYZ>::iterator c = centers.begin(); c < centers.end(); c++) {
    c->z = z_coord;
  }

  return centers;
}

}
