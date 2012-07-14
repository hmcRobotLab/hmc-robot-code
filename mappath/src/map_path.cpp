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

#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/gil/extension/io/jpeg_dynamic_io.hpp>

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


      boost::gil::rgb8_image_t img(map->info.width,map->info.height);
      boost::gil::rgb8_pixel_t black(0,0,0);
      boost::gil::rgb8_pixel_t white(255,255,255);
      boost::gil::rgb8_pixel_t gray(205,205,205);
      boost::gil::rgb8_pixel_t green(0,255,0);

      fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;

          if (map->data[i] == 0) { //occ [0,0.1)
            boost::gil::view(img)(x,y) = white;
            fputc(254, out);
          } else if (map->data[i] == +100) { //occ (0.65,1]
            boost::gil::view(img)(x,y) = black;
            fputc(000, out);
          } else { //occ [0.1,0.65]
            boost::gil::view(img)(x,y) = gray;
            fputc(205, out);
          }

        }
      }


      fclose(out);

      for( std::vector<geometry_msgs::PoseStamped>::iterator it = poses.begin();
          it < poses.end();
          ++it) {
            boost::gil::view(img)( (int) (it->pose.position.x / 0.05)+2000 , (-(int)(it->pose.position.y/ 0.05))+2000 ) = green;
      }
      boost::gil::png_write_view( mapname_ + ".png",const_view(img));

      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


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


