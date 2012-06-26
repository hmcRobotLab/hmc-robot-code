#include "data_capture_ros.h"
#include "odomPubRos.h"

namespace fovis_ros_kinect
{

  const std::string rstring = "/camera/rgb/image_raw";
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
  printf("deleting cap\n");
  delete[] depth_data;
  delete[] gray_buf;
  delete itnh;
}

void DataCapture::processGray(const sensor_msgs::ImageConstPtr& msg)
{
  capturing=true;
  //printf("processgray\n");
  //memcpy(gray_buf,&(msg->data),width*height);
  for(int i = 0;i<width*height;++i)
  {
    gray_buf[i] = msg->data[i];
    //std::cout << gray_buf[i] << " " ;
  }
}

void DataCapture::processDepth(const sensor_msgs::ImageConstPtr& msg)
{
  //printf("processdepth\n");
  //memcpy(depth_data,&(msg->data),width*height);

  int num_depth_pixels = width * height;
  for(int i=0; i<num_depth_pixels; i++) {
    uint16_t d = msg->data[i];
    if(d != 0) {
      depth_data[i] = d * 1e-3;
    } else {
      depth_data[i] = NAN;
    }
  }



  depth_image->setDepthImage(depth_data);
}

}
