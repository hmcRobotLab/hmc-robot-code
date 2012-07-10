#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  cv::Mat image = bridge.imgMsgToCv(msg, "bgr8");
  cv::imwrite("testrgb.png",image);
}

  int width = 640;
  int height = 480;

void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
  int num_depth_pixels = width * height;
  float* depth_data = new float[num_depth_pixels];

  FILE* file = fopen( "testdepth.pgm", "w");
  fprintf( file, "P2\n");
  fprintf( file, "%i %i\n", msg->width,msg->height);
  fprintf( file, "10000\n");

  for(int y = 0; y < 480; ++y) {
    for(int x = 0; x<640; ++x) {
      float d = 0;
      memcpy(&d,&(msg->data)[(y*2560)+(x*4)],sizeof(d));
      int x;
      if( (d<.015) || isnan(d) )
        x = 0;
      else 
        x = d * 1000;
      fprintf( file, "%i ",x);
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
  image_transport::Subscriber sub2 = it.subscribe("camera/depth/image", 1, imageCallback2);
  ros::spin();
}
