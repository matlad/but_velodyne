cmake_minimum_required(VERSION 2.8.3)
project(but_calibration_camera_velodyne)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -g")

find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  rospy  
  roscpp
  cv_bridge
  image_transport
  image_geometry
  pcl_ros
  velodyne_pointcloud
  roslint
)

catkin_package(CATKIN_DEPENDS std_msgs )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
message("Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(calibrationNonRos
               src/Image.cpp
               include/but_calibration_camera_velodyne/Image.h
               src/Velodyne.cpp
               include/but_calibration_camera_velodyne/Velodyne.h
               src/Similarity.cpp
               include/but_calibration_camera_velodyne/Similarity.h
               src/Calibration3DMarker.cpp
               include/but_calibration_camera_velodyne/Calibration3DMarker.h
               src/Calibrator.cpp
               include/but_calibration_camera_velodyne/Calibrator.h
               src/Camera.cpp
               include/but_calibration_camera_velodyne/Camera.h
               src/Calibration6DoF.cpp
               include/but_calibration_camera_velodyne/Calibration6DoF.h
               include/but_calibration_camera_velodyne/macros.h
               src/calibration.cpp
               src/FishEyeCamera.cpp
               include/but_calibration_camera_velodyne/FishEyeCamera.h

               src/Colorizer.cpp #todo remove
               include/but_calibration_camera_velodyne/Colorizer.h #todo remove
               src/types.cpp)
target_link_libraries(calibrationNonRos ${PCL_LIBRARIES})
target_link_libraries(calibrationNonRos ${catkin_LIBRARIES})

add_executable(calibration
               src/calibration-node.cpp
               src/Image.cpp
               include/but_calibration_camera_velodyne/Image.h
               src/Velodyne.cpp
               include/but_calibration_camera_velodyne/Velodyne.h
               src/Similarity.cpp
               include/but_calibration_camera_velodyne/Similarity.h
               src/Calibration3DMarker.cpp
               include/but_calibration_camera_velodyne/Calibration3DMarker.h
               src/Calibrator.cpp
               include/but_calibration_camera_velodyne/Calibrator.h
               src/RosCalibratorWrapper.cpp
               include/but_calibration_camera_velodyne/RosCalibratorWrapper.h
               src/Camera.cpp
               include/but_calibration_camera_velodyne/Camera.h
               src/Calibration6DoF.cpp
               include/but_calibration_camera_velodyne/Calibration6DoF.h
               include/but_calibration_camera_velodyne/macros.h
               src/FishEyeCamera.cpp
               include/but_calibration_camera_velodyne/FishEyeCamera.h
               include/but_calibration_camera_velodyne/Constants.h
               src/types.cpp
               include/but_calibration_camera_velodyne/types.h
               src/Colorizer.cpp #todo remove
               include/but_calibration_camera_velodyne/Colorizer.h #todo remove
               )
target_link_libraries(calibration ${PCL_LIBRARIES}) 
target_link_libraries(calibration ${catkin_LIBRARIES})

add_executable(coloring
        src/coloring-node.cpp
        include/but_calibration_camera_velodyne/macros.h
        include/but_calibration_camera_velodyne/Constants.h

        src/Image.cpp
        include/but_calibration_camera_velodyne/Image.h
        src/Velodyne.cpp
        include/but_calibration_camera_velodyne/Velodyne.h
        src/Colorizer.cpp
        include/but_calibration_camera_velodyne/Colorizer.h
        src/FishEyeCamera.cpp
        include/but_calibration_camera_velodyne/FishEyeCamera.h
        src/RosColoringWrapper.cpp
        include/but_calibration_camera_velodyne/RosColoringWrapper.h
        )
target_link_libraries(coloring ${PCL_LIBRARIES}) 
target_link_libraries(coloring ${catkin_LIBRARIES})

add_executable(coloringNonRos
        src/coloring.cpp
        include/but_calibration_camera_velodyne/macros.h
        include/but_calibration_camera_velodyne/Constants.h

        src/Image.cpp
        include/but_calibration_camera_velodyne/Image.h
        src/Velodyne.cpp
        include/but_calibration_camera_velodyne/Velodyne.h
        src/Colorizer.cpp
        include/but_calibration_camera_velodyne/Colorizer.h
        src/FishEyeCamera.cpp
        include/but_calibration_camera_velodyne/FishEyeCamera.h
        )
target_link_libraries(coloringNonRos ${PCL_LIBRARIES})
target_link_libraries(coloringNonRos ${catkin_LIBRARIES})

roslint_cpp()

