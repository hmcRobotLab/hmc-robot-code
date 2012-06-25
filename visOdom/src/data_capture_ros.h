#ifndef __data_capture_hpp__
#define __data_capture_hpp__

#include <fovis.hpp>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace fovis_ros_kinect
{

class DataCapture
{
  public:
    DataCapture();
    ~DataCapture();

    bool initialize(ros::NodeHandle nh);

    bool startDataCapture();

    bool stopDataCapture();

    bool captureOne();

    fovis::DepthImage* getDepthImage() {
      return depth_image;
    }

    const fovis::CameraIntrinsicsParameters& getRgbParameters() const {
      return rgb_params;
    }

    const uint8_t* getGrayImage() {
      return gray_buf;
    }

  private:
    fovis::DepthImage* depth_image;

    int width;
    int height;

    fovis::CameraIntrinsicsParameters rgb_params;

    image_transport::ImageTransport *itnh;
    image_transport::Subscriber graySub;
    image_transport::Subscriber depthSub;
    bool capturing;
    void processGray(const sensor_msgs::ImageConstPtr& msg);
    void processDepth(const sensor_msgs::ImageConstPtr& msg);

    float* depth_data;

    uint8_t* gray_buf;
};

}

#endif
