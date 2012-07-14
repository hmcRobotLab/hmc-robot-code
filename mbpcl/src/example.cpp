// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/png_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

void convertToPC(std::string rgbname,std::string depthname,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  cv::Mat rgbImage = cv::imread(rgbname,1);
  std::vector<double> floats;
  std::ifstream depthfile(depthname.c_str());
  std::string inputline = "";
  std::getline(depthfile,inputline);
  std::getline(depthfile,inputline);
  std::getline(depthfile,inputline);
  copy( std::istream_iterator<double>(depthfile),
      std::istream_iterator<double>(),
      std::back_inserter(floats));
  std::cout << floats.size() << std::endl;

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  cloud -> points.resize(640 * 480);
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin();

  for(int y = 0; y < rgbImage.rows;++y) {
    for(int x = 0; x< rgbImage.cols;++x) {
      double depth = floats[ (y*640) + x ]; 
      pcl::PointXYZRGB& pt = *pt_iter++;
      if (depth == 0)
        continue;
      
      pt.x = (x-319.5) * depth * .00190476;
      pt.y = -(y-239.5) * depth * .00190476;
      pt.z = depth; 

      pt.b = rgbImage.at<uint8_t>(y,x*3);
      pt.g = rgbImage.at<uint8_t>(y,(x*3)+1);
      pt.r = rgbImage.at<uint8_t>(y,(x*3)+2);
    }
  }
}
void convertToPC(boost::filesystem::path dir)
{
  boost::regex pngPat(".*.png");
  boost::regex pgmPat(".*.pgm");

  std::string pngName;
  std::string pgmName;
  for(boost::filesystem::directory_iterator iter(dir),end;
      iter != end;
      ++iter)
  {
    std::string name = iter->path().leaf().c_str();
    if (boost::regex_match(name,pngPat))
      pngName = iter->path().c_str();
    if (boost::regex_match(name,pgmPat))
      pgmName = iter->path().c_str();
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  convertToPC(pngName,pgmName,cloud);
  pcl::io::savePCDFileBinary( (dir / "cloud.pcd").c_str(),*cloud);
}

bool beenProcessed(boost::filesystem::path dir)
{
  boost::regex pattern(".*.pcd");
  for(boost::filesystem::directory_iterator iter(dir),end;
      iter != end;
      ++iter)
  {
    std::string name = iter->path().leaf().c_str();
    if (boost::regex_match(name,pattern))
      return true;
  }
  return false;
}


void pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, 
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
  grid.setLeafSize(0.07,0.07,0.07);
  grid.setInputCloud(cloud_src);
  grid.filter(*src);
  grid.setInputCloud(cloud_tgt);
  grid.filter(*tgt);

  std::cout << "Voxeled: " << src->size() << std::endl;


  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setMaxCorrespondenceDistance(1.0);
  icp.setMaximumIterations(150);
  icp.setTransformationEpsilon(1e-5);
  icp.setEuclideanFitnessEpsilon(1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  icp.setInputTarget (tgt);
  icp.setInputCloud (src);
  icp.align(*output);
  std::cout << "Aligned: " << icp.hasConverged() << " Score: " << icp.getFitnessScore() << std::endl;

  Eigen::Matrix4f trans;
  trans = (icp.getFinalTransformation()).inverse();
  pcl::transformPointCloud(*cloud_tgt, *output, trans);
  std::cout << trans << std::endl;
  *output += *cloud_src;
}

void alignDir(boost::filesystem::path dir,pcl::PointCloud<pcl::PointXYZRGB>::Ptr agg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>( (dir/"cloud.pcd").c_str() , *cloud2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

  Eigen::Matrix4f pairTransform;
  pairAlign(agg,cloud2,temp,pairTransform);
  *agg=*temp;
  std::cout << "agg size: " << agg->size() << std::endl;
}

int
main (int argc, char** argv)
{
  boost::filesystem::path p("data/");
  std::vector<boost::filesystem::path> dirs;
  for(boost::filesystem::directory_iterator it(p);
      it != boost::filesystem::directory_iterator() ;
      ++it)
  {
    if(boost::filesystem::is_directory(*it)) {
      dirs.push_back( (*it) );
      std::cout << *it << std::endl;
    }
  }

  std::sort(dirs.begin(),dirs.end());

  for(std::vector<boost::filesystem::path>::iterator it = dirs.begin();
      it < dirs.end();
      ++it)
  {
    if (!beenProcessed(*it))
    {
      std::cout << "converting " << *it << std::endl;
      convertToPC(*it);
    }
  }



  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>( (dirs[0]/"cloud.pcd").c_str() , *finalcloud);

  for(std::vector<boost::filesystem::path>::iterator it = ++dirs.begin();
      it < dirs.end();
      ++it)
  {
    if (beenProcessed(*it))
    {
      std::cout << "adding " << *it << std::endl;
      alignDir(*it,finalcloud);
    }
  }

  pcl::io::savePCDFileBinary("final.pcd",*finalcloud);
  //pcl::io::savePNGFile("test.png",*finalcloud);
  //
  //
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::io::loadPCDFile<pcl::PointXYZRGB>("testimages/1/cloud.pcd", *cloud1);
  //pcl::io::loadPCDFile<pcl::PointXYZRGB>("testimages/2/cloud.pcd", *cloud2);
  //
  //std::cout << "Loaded "
  //          << cloud1->width * cloud1->height
  //          << std::endl;
  //std::cout << "Loaded "
  //          << cloud2->width * cloud2->height
  //          << std::endl;

  //Eigen::Matrix4f pairTransform;
  //pairAlign (cloud1,cloud2,cloud, pairTransform,false);

  //pcl::io::savePCDFileBinary("test.pcd",*cloud);
  return 1;
}
