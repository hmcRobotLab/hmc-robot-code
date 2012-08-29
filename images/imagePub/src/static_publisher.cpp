#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cstddef>
#include <stdlib.h>

int main(int argc, char** argv)
{
  if (argc != 2 && argc != 4) {
    std::cout << "Usage: static_publisher <path_to_image>" << std::endl;
    exit(1);
  }

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("static_image", 1);

  cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
  sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");
  std::cout << "Message Constructed; Preparing to Publish" << std::endl;
  pub.publish(msg);
  std::cout << "Publishing" << std::endl;
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
