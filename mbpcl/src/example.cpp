// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>



void convertToPC(std::string rgbname,std::string depthname,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  cv::Mat rgbImage = cv::imread(rgbname,1);
  //cv::Mat depthImage = cv::imread(depthname,0);
  //cv::MatIterator_<uint16_t> dIt = depthImage.begin<uint16_t>();
  //int i =0;
  //for(;dIt < depthImage.end<uint16_t>(); ++dIt,++i)
  //{
  //  if (i%640 == 0)
  //    std::cout << std::endl;
  //  std::cout << *dIt << " ";
  //}

  //std::cout << i << std::endl;
  //std::cout << 640 * 480 << std::endl;

  char* test = "./testimages/testdepth.pgm";
  std::ifstream depthFile;
  depthFile.open(test,std::ifstream::in);
  std::vector<std::vector<int> > depths( 480, std::vector<int>(640) );
  for (int y = 0; y < 480; ++y) {
    for (int x = 0; x< 640; ++x) {
      depthFile >> depths[x][y];
    }
  }

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  cloud -> points.resize(640 * 480);
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin();
  //std::cout << depthImage.at<uint16_t>(0,0) << std::endl;

  //for(int y = 0; y < depthImage.rows;++y) {
  //  for(int x = 0; x< depthImage.cols;++x) {
  //    uint8_t depth = depthImage.at<uint8_t>(y,x);
  //    pcl::PointXYZRGB& pt = *pt_iter++;
  //    if (depth == 0)
  //      continue;
  //    
  //    pt.x = (x-319.5) * depth * .00190476;
  //    pt.y = (y-239.5) * depth * .00190476;
  //    pt.z = depth; 

  //    //pcl::RGBValue color;
  //    //color.Red = rgbImage.at<int>(x*3,y);
  //    //color.Green = rgbImage.at<int>((x*3)+1,y);
  //    //color.Blue = rgbImage.at<int>((x*3)+2,y);
  //    pt.r = rgbImage.at<uint8_t>(y,x*3);
  //    pt.g = rgbImage.at<uint8_t>(y,(x*3)+1);
  //    pt.b = rgbImage.at<uint8_t>(y,(x*3)+2);
  //  }
  //}
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  convertToPC("testimages/testrgb.png","testimages/testdepth.pgm",cloud);
  pcl::io::savePCDFileASCII("test.pcd",*cloud);
}

