/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "LinearMath/btMatrix3x3.h"
#include "geometry_msgs/Quaternion.h"

//#include <boost/gil/gil_all.hpp>
//#include <boost/gil/extension/io/png_dynamic_io.hpp>
//#include <boost/gil/extension/io/jpeg_dynamic_io.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
 
/**
 * @brief Map generation node.
 */
class MapGenerator 
{

  public:
    bool gotPath;
    std::vector<geometry_msgs::PoseStamped> poses;
    MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
    {
      gotPath=false;
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
      path_sub_ = n.subscribe("slam_path", 1, &MapGenerator::pathCallback, this);
    }

    void pathCallback(const nav_msgs::Path& path)
    {
      if (!gotPath) {
        gotPath = true;
        poses = path.poses;
      }
    }

    //void drawLine(int x1, int y1, int x2, int y2, boost::gil::rgb8_pixel_t color, boost::gil::rgb8_image_t img)
    //{
    //  const bool steep = (abs(y2-y1)>abs(x2-x1));
    //  if (steep)
    //  {
    //    std::swap(x1,y1);
    //    std::swap(x2,y2);
    //  }

    //  if (x1>x2)
    //  {
    //    std::swap(x1,x2);
    //    std::swap(y1,y2);
    //  }
    //  const int dx = x2-x1;
    //  const int dy = abs(y2-y1);

    //  float error = dx / 2.0f;
    //  const int ystep = (y1 < y2) ? 1 : -1;
    //  int y = (int)y1;

    //  const int maxX = (int)x2;

    //  for(int x=(int)x1; x<maxX; x++)
    //  {
    //    if(steep)
    //      boost::gil::view(img)(y,x) = color;
    //    else
    //    {
    //      std::cout << x << " " << y << std::endl;
    //      boost::gil::view(img)(x,y) = color;
    //    }
    //    error -= dy;
    //    if (error < 0)
    //    {
    //      y+= ystep;
    //      error += dx;
    //    }
    //  }
    //}
    

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);


      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }


      //boost::gil::rgb8_image_t img(map->info.width,map->info.height);
      //boost::gil::rgb8_pixel_t black(0,0,0);
      //boost::gil::rgb8_pixel_t white(255,255,255);
      //boost::gil::rgb8_pixel_t gray(205,205,205);
      //boost::gil::rgb8_pixel_t green(0,255,0);
      cv::Mat_<cv::Vec3b> img(map->info.height,map->info.width, cv::Vec3b(100,100,200));//CV_8UC3);// = cv::Mat::zeroes( map->info.width, map->info.height, cv::CV_8UC3);
      cv::Scalar black(0,0,0);
      cv::Scalar white(255,255,255);
      cv::Scalar gray(205,205,205);

      fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;

          if (map->data[i] == 0) { //occ [0,0.1)
            //boost::gil::view(img)(x,y) = white;
            //img.at<uchar>(255,255,255);//cv::Scalar>(cv::Point(x,y))=white;
            img(y,x)=cv::Vec3b(255,255,255);
            fputc(254, out);
          } else if (map->data[i] == +100) { //occ (0.65,1]
            //img.at<cv::Scalar>(cv::Point(x,y))=black;
            img(y,x)=cv::Vec3b(0,0,0);
            //boost::gil::view(img)(x,y) = black;
            fputc(000, out);
          } else { //occ [0.1,0.65]
            //img.at<cv::Scalar>(cv::Point(x,y))=gray;
            img(y,x)=cv::Vec3b(155,155,155);
            //boost::gil::view(img)(x,y) = gray;
            fputc(205, out);
          }

        }
      }


      fclose(out);

      //cv::Point lp(map->info.width/2,map->info.height/2);
      cv::Point lp(0,0);
      
      //cv::Point lp(poses.front().pose.position.x+2000,poses.front().pose.position.y+2000);//   map->info.width/2,map->info.height/2);
      int r = 100;
      int g = 0;
      int b = 250;
      int x = -1;


      double dist = 0;
      double lastx = 0;
      double lasty = 0;
      for( std::vector<geometry_msgs::PoseStamped>::iterator it = poses.begin();
          it < poses.end();
          ++it) {
        int cx = (int) (it->pose.position.x / map->info.resolution)+map->info.width/2 + (map->info.width - 4002)/2;
        int cy = (-(int)(it->pose.position.y/ map->info.resolution))+map->info.height/2 + (map->info.height- 4000)/2;
        cv::Point cp(cx,cy);
        double dd= sqrt ( pow( lastx- (it->pose.position.x),2) + pow(lasty-(it->pose.position.y),2));
        double ddb = sqrt ( pow( (cx) -lp.x,2) + pow( (cy) -lp.y,2));
        if(lp.x !=0 && lp.y!=0)
        {
          if (dd > 1)
            std::cout << " ! " << dd << std::endl;
          if (lastx != 0)
            dist += dd;
          cv::line(img, lp, cp, cv::Scalar(b,g,r), 2, 8);
        }
        if (g == 255)
          x = -1;
        if (g==0)
          x=1;
        lastx=it->pose.position.x;
        lasty=it->pose.position.y;
        g+=x;
        lp = cp;
      }
      double temp;
      std::stringstream ss;
      ss << "Distance: " << dist;
      cv::putText(img,ss.str(),cv::Point(50,50),cv::FONT_HERSHEY_PLAIN,3,cv::Scalar(0,0,0),4,8,false);
      ss.str("");
      ss << "Time: " << poses.front().header.stamp - poses.back().header.stamp ;
      cv::putText(img,ss.str(),cv::Point(50,90),cv::FONT_HERSHEY_PLAIN,3,cv::Scalar(0,0,0),4,8,false);

      ss.str("");
      ros::param::get("/slam_gmapping/stt",temp);
      ss << "stt: " << temp;
      std::cout << " >> " << temp << std::endl;
      cv::putText(img,ss.str(),cv::Point(50,130),cv::FONT_HERSHEY_PLAIN,3,cv::Scalar(0,0,0),4,8,false);

      ss.str("");
      ros::param::get("/slam_gmapping/str",temp);
      ss << "str: " << temp;
      cv::putText(img,ss.str(),cv::Point(50,170),cv::FONT_HERSHEY_PLAIN,3,cv::Scalar(0,0,0),4,8,false);

      ss.str("");
      ros::param::get("/slam_gmapping/srt",temp);
      ss << "srt: " << temp;
      cv::putText(img,ss.str(),cv::Point(50,210),cv::FONT_HERSHEY_PLAIN,3,cv::Scalar(0,0,0),4,8,false);

      ss.str("");
      ros::param::get("/slam_gmapping/srr",temp);
      ss << "srr: " << temp;
      cv::putText(img,ss.str(),cv::Point(50,250),cv::FONT_HERSHEY_PLAIN,3,cv::Scalar(0,0,0),4,8,false);



      std::string cvname = mapname_ + ".png";
      ROS_INFO("Writing bitmap to %s", cvname.c_str());
      cv::imwrite(cvname,img);

      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

      std::cout << "Distance traveled: "<< dist << " meters" << std::endl;

      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

       */

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      btMatrix3x3 mat(btQuaternion(orientation.x, orientation.y, orientation.z, orientation.w));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    ros::Subscriber path_sub_;
    bool saved_map_;

};

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }
  
  MapGenerator mg(mapname);

  while(!mg.saved_map_)
    ros::spinOnce();

  return 0;
}

