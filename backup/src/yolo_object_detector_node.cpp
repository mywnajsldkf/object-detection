/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <darknet_ros/YoloObjectDetector.hpp>
#include <ros/ros.h>
// Jeongin added
#include <librealsense2/rs.hpp>
//#include <iostream>

int main(int argc, char** argv) {
  // this node name is "darknet_ros"
  ros::init(argc, argv, "darknet_ros");
  // make class to use functions about node : nodeHandle
  ros::NodeHandle nodeHandle("~");
  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);

  
  // jeongin added
  // rs2::pipeline p;
  // p.start();
  
  ros::spin();
  return 0;
}
