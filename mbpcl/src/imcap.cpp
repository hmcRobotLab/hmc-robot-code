#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <boost/filesystem.hpp>
#include <sstream>

cv::Mat rgbImage;
sensor_msgs::Image depthMsg;
bool cap = false;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  rgbImage = bridge.imgMsgToCv(msg, "bgr8");
  std::cout << "image " << std::endl;
}

  int width = 640;
  int height = 480;

void depthCb(const sensor_msgs::ImageConstPtr& msg)
{
  cap = true;
  depthMsg = *msg;
  std::cout << "image depth " << std::endl;
}

void saveDepth(boost::filesystem::path dir)
{
  int num_depth_pixels = width * height;
  float* depth_data = new float[num_depth_pixels];

  std::cout << (dir/"depth.pgm").c_str() << std::endl;
  FILE* file = fopen( (dir/"depth.pgm").c_str(), "w");
  fprintf( file, "P2\n");
  fprintf( file, "%i %i\n", depthMsg.width,depthMsg.height);
  fprintf( file, "10000\n");

  for(int y = 0; y < 480; ++y) {
    for(int x = 0; x<640; ++x) {
      float d = 0;
      memcpy(&d,&(depthMsg.data)[(y*2560)+(x*4)],sizeof(d));
      double x;
      if( (d<.015) || isnan(d) )
        x = 0;
      else 
        x = d;
      fprintf( file, "%f ",x);
      }
    fprintf( file,"\n");
  }
  fclose( file );
  std::cout << "done" << std::endl;
  //cv::imwrite("testdepth.ppm",image);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_color", 1, imageCallback);
  image_transport::Subscriber sub2 = it.subscribe("camera/depth/image", 1, depthCb);

  boost::filesystem::path basePath("./data");
  int i = 0;
  ros::Time last = ros::Time::now();
  while(true) {
    ros::spinOnce();

    if (ros::Time::now() - last > ros::Duration(1))
    {
      if (cap) {
        std::stringstream ext;
        ext << i;
        cv::imwrite( ( (basePath/ext.str())/"rgb.png").c_str(),rgbImage);
        saveDepth( (basePath/ext.str()));
        ++i;
      }
      last = ros::Time::now();
    }
  }
}
