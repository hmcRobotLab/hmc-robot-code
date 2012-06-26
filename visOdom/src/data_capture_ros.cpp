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
}

void DataCapture::processGray(const sensor_msgs::ImageConstPtr& msg)
{
  capturing = true;
  //memcpy(gray_buf,&(msg->data),width*height);
  for(int i = 0;i<width*height;++i)
  {
    gray_buf[i] = msg->data[i];
    //std::cout << int(gray_buf[i]) << " " ;
  }



  //std::cout << msg->encoding << std::endl;
  //std::cout << msg->width << std::endl;
  //std::cout << msg->height << std::endl;
  //std::cout << msg->step << std::endl;

  //int num_rgb_pixels = width * height;
  //for(int i=0; i<num_rgb_pixels; i++) {
  //  gray_buf[i] = (int)round(0.2125 * (msg->data)[i*3] + 
  //      0.7154 * (msg->data)[(i*3)+1]+ 
  //      0.0721 * (msg->data)[(i*3)+2]);
  //}


}

void DataCapture::processDepth(const sensor_msgs::ImageConstPtr& msg)
{
  //printf("processdepth\n");
  //memcpy(depth_data,&(msg->data),width*height);

  //std::cout << msg->encoding << std::endl;
  //std::cout << msg->width << std::endl;
  //std::cout << msg->height << std::endl;
  //std::cout << msg->step << std::endl;
  int num_depth_pixels = width * height;
  for(int i=0; i<num_depth_pixels; ++i) {
    float d = 0;// = (msg->data[i*4]);
    memcpy(&d,&(msg->data)[i*4],sizeof(d));
    if( (abs(d) > .001)&&!(isnan(d)) ) {
      depth_data[i] = d;
    } else {
      depth_data[i] = NAN;
    }
  }



  depth_image->setDepthImage(depth_data);
}

}
