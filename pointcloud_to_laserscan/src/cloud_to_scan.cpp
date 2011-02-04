/*                                                                                                                       
 * Copyright (c) 2010, Willow Garage, Inc.                                                                               
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its                                                
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

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"

#include "pcl_ros/transforms.h"

namespace pointcloud_to_laserscan
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  class CloudToScan : public nodelet::Nodelet
  {
    public:
      //Constructor
      CloudToScan(): min_height_(0.10), max_height_(0.15), laser_frame_id_("/openi_depth_frame")
    {
    };

    private:
      virtual void onInit()
      {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& private_nh = getPrivateNodeHandle();

        private_nh.getParam("min_height", min_height_);
        private_nh.getParam("max_height", max_height_);

        private_nh.getParam("laser_frame_id", laser_frame_id_);
        pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
        sub_ = nh.subscribe<PointCloud>("cloud", 10, &CloudToScan::callback, this);
      }

      void callback(const PointCloud::ConstPtr& cloud)
      {
        //fist, we want to transform the point cloud, but only if its in a different frame
        PointCloud transformed_cloud;
        if(!tf_listener_.waitForTransform(laser_frame_id_, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2)))
        {
          ROS_ERROR("Failed to get a transform for the pointcloud in time, doing nothing");
          return;
        }

        pcl_ros::transformPointCloud(laser_frame_id_, *cloud, transformed_cloud, tf_listener_);

        sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
        NODELET_DEBUG("Got cloud");
        //Copy Header
        output->header = transformed_cloud.header;
        output->header.frame_id = laser_frame_id_;
        output->angle_min = -M_PI/2;
        output->angle_max = M_PI/2;
        output->angle_increment = M_PI/180.0/2.0;
        output->time_increment = 0.0;
        output->scan_time = 1.0/30.0;
        output->range_min = 0.45;
        output->range_max = 10.0;

        uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
        output->ranges.assign(ranges_size, output->range_max + 1.0);

        for (PointCloud::const_iterator it = transformed_cloud.begin(); it != transformed_cloud.end(); ++it)
        {
          const float &x = it->x;
          const float &y = it->y;
          const float &z = it->z;

          if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
          {
            NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;  
          }

          if (z > max_height_ || z < min_height_)
          {
            NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", z, min_height_, max_height_);
            continue;
          }

          double angle = atan2(y, x);
          if (angle < output->angle_min || angle > output->angle_max)
          {
            NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
            continue;
          }

          int index = (angle - output->angle_min) / output->angle_increment;
          //printf ("index xyz( %f %f %f) angle %f index %d\n", x, y, z, angle, index);
          double range_sq = x*x+y*y;
          if (output->ranges[index] * output->ranges[index] > range_sq)
            output->ranges[index] = sqrt(range_sq);

        }
        pub_.publish(output);
      }


      double min_height_, max_height_;
      std::string laser_frame_id_;

      ros::Publisher pub_;
      ros::Subscriber sub_;
      tf::TransformListener tf_listener_;

  };

  PLUGINLIB_DECLARE_CLASS(pointcloud_to_laserscan, CloudToScan, pointcloud_to_laserscan::CloudToScan, nodelet::Nodelet);
}
