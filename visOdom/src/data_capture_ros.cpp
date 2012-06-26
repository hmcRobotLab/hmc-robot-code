#include "data_capture_ros.h"
#include "odomPubRos.h"

#define CHECK_STATUS(rc, msg) if((rc) != XN_STATUS_OK) { \
  fprintf(stderr, "%s: %s\n", (msg), xnGetStatusString(rc)); return false; }

namespace fovis_ros_kinect
{

DataCapture::DataCapture()
{
  capturing = false;
  width = 640;
  height = 480;

  memset(&rgb_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
  rgb_params.width = width;
  rgb_params.height = height;

  // FROM THE EXAMPLE: TODO
  // TODO read these values from the camera somehow, instead of hard-coding it
  // Unfortunately, the OpenNI API doesn't seem to expose them.
  rgb_params.fx = 528.49404721; 
  rgb_params.fy = rgb_params.fx;
  rgb_params.cx = width / 2.0;
  rgb_params.cy = height / 2.0;

  depth_image = new fovis::DepthImage(rgb_params, width, height);
  depth_data = new float[width * height];
  gray_buf = new uint8_t[width * height];
}

DataCapture::~DataCapture()
{
  delete[] depth_data;
  delete[] gray_buf;
}

bool DataCapture::initialize(ros::NodeHandle nh)
{
  // TODO make the params not hardcoded
  printf("Connecting to ROS services !!!\n ");

  itnh = new image_transport::ImageTransport(nh);
  graySub = itnh->subscribe("/camera/rgb/image_raw",1,&DataCapture::processGray,this);
  depthSub = itnh->subscribe("/camera/depth/image_raw",1,&DataCapture::processDepth,this);
  //depthSub = itnh->subscribe("/camera/depth/image_raw",1,processDepth);

  return true;
}

bool DataCapture::startDataCapture()
{
  capturing = true;
  return true;
}

bool DataCapture::stopDataCapture()
{
  capturing = false;
  return true;
}

bool DataCapture::captureOne()
{
  return capturing;
}

void DataCapture::processGray(const sensor_msgs::ImageConstPtr& msg)
{
  printf("capturing");
  if (capturing)
    memcpy(gray_buf,&(msg->data),width*height);
}

void DataCapture::processDepth(const sensor_msgs::ImageConstPtr& msg)
{
  if (capturing) {
    memcpy(depth_data,&(msg->data),width*height);
    depth_image->setDepthImage(depth_data);
  }
}

}
