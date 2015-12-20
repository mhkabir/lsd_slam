#ifndef ROSINPUTSTREAMTHREAD_H
#define ROSINPUTSTREAMTHREAD_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>

#include "IOWrapper/InputStream.h"
#include "util/Undistorter.h"

namespace lsd_slam {

/**
 * Image stream provider using ROS messages.
 */
class ROSInputStreamThread : public InputStream {
 public:
  ROSInputStreamThread();
  ~ROSInputStreamThread();

  /**
   * Starts the thread.
   */
  void run();

  void setCalibration(std::string file);

  /**
   * Thread main function.
   */
  void operator()();

  // get called on ros-message callbacks
  void vidCb(const sensor_msgs::ImageConstPtr img);
  void infoCb(const sensor_msgs::CameraInfoConstPtr info);
  void odomCb(const nav_msgs::OdometryConstPtr odom);

 private:

  bool haveCalib;
  Undistorter* undistorter;

  ros::NodeHandle nh_;

  std::string vid_channel;
  ros::Subscriber vid_sub;

  std::string odom_channel;
  ros::Subscriber odom_sub;

  int lastSEQ;
};

}

#endif
