#ifndef INPUTSTREAM_H
#define INPUTSTREAM_H

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"
#include <ros/ros.h>

namespace lsd_slam {

class InputStream {
 public:
  virtual ~InputStream() {};

  /**
   * Starts the thread.
   */
  virtual void run() {};

  virtual void setCalibration(std::string file) {};

  /**
   * Gets the NotifyBuffer to which incoming images are stored.
   */
  inline NotifyBuffer<TimestampedMat>* getImgBuffer() {
    return imageBuffer;
  };

  inline NotifyBuffer<TimestampedPose>* getPoseBuffer() {
    return poseBuffer;
  }

  /**
   * Gets the Camera Calibration. To avoid any dependencies, just as simple float / int's.
   */
  inline float fx() {return fx_;}
  inline float fy() {return fy_;}
  inline float cx() {return cx_;}
  inline float cy() {return cy_;}
  inline int width() {return width_;}
  inline int height() {return height_;}

 protected:
  float fx_, fy_, cx_, cy_;
  int width_, height_;

  NotifyBuffer<TimestampedMat>* imageBuffer;
  NotifyBuffer<TimestampedPose>* poseBuffer;
};

}

#endif
