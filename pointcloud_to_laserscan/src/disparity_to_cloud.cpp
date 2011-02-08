/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

/*
 * Node to take kinect's image + disparity image and publish a point cloud.
 *
 * Author: Stuart Glaser
 */

#include <ros/ros.h>

#include <cv_bridge/CvBridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include "pcl/ros/conversions.h"

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>

// TODO: this include should be removed when this CvBridge ticket is closed:
// https://code.ros.org/trac/ros-pkg/ticket/4745
#include <actionlib/enclosure_deleter.h>

#include "pcl/filters/voxel_grid.h"

// Kinect camera parameters, from Hauke
const int KINECT_WIDTH = 640;
const int KINECT_HEIGHT = 480; 
const int KINECT_PRINCIPLE_U = 320;  // Principle point in pixel coordinates
const int KINECT_PRINCIPLE_V = 240;
//const double KINECT_F = 570.342;
const double KINECT_F = 525.0;
const double KINECT_BASELINE = 0.075;


// Converts pixel/disparity coordinates to position using the
// Kinect's camera parameters.
//
// Axes: x right, y down, z into the image
void uvd_to_xyz(const Eigen3::Vector3d &uvd, Eigen3::Vector3d &xyz)
{
  xyz[2] = (KINECT_F * KINECT_BASELINE) / uvd[2];
  xyz[0] = xyz[2] / KINECT_F * (uvd[0] - KINECT_PRINCIPLE_U);
  xyz[1] = xyz[2] / KINECT_F * (uvd[1] - KINECT_PRINCIPLE_V);
}

void xyz_to_uvd(const Eigen3::Vector3d &xyz, Eigen3::Vector3d &uvd)
{
  uvd[2] = (KINECT_F * KINECT_BASELINE) / xyz[2];
  uvd[0] = (KINECT_F / xyz[2]) * xyz[0] + KINECT_PRINCIPLE_U;
  uvd[1] = (KINECT_F / xyz[2]) * xyz[1] + KINECT_PRINCIPLE_V;
}

void uvd_to_fake_depth(const Eigen3::Vector3d &uvd, Eigen3::Vector3d &xyz, double fake_depth)
{
  xyz[2] = fake_depth;
  xyz[0] = xyz[2] / KINECT_F * (uvd[0] - KINECT_PRINCIPLE_U);
  xyz[1] = xyz[2] / KINECT_F * (uvd[1] - KINECT_PRINCIPLE_V);
}


typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

class DisparityToCloud
{
public:
  DisparityToCloud(ros::NodeHandle nh, ros::NodeHandle p_nh)
    : image_synchronizer_(ApproxTimeSync(10), sub_camera_info_, sub_disparity_)
  {
    pub_points_ = nh.advertise<sensor_msgs::PointCloud2>("the_points2", 2);
    image_synchronizer_.registerCallback(boost::bind(&DisparityToCloud::imageCB, this, _1, _2));

    sub_camera_info_.subscribe(nh, "camera_info", 2);
    sub_disparity_.subscribe(nh, "disparity", 2);
  }

  ~DisparityToCloud()
  {
    sub_camera_info_.unsubscribe();
    sub_disparity_.unsubscribe();
  }

  void imageCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info,
               const stereo_msgs::DisparityImage::ConstPtr &disparity_msg)
  {
    sensor_msgs::CvBridge bridge;
    cv::Mat disparity = bridge.imgMsgToCv(actionlib::share_member(disparity_msg, disparity_msg->image), "passthrough");  // "32FC1"

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > ();
    cloud->header.stamp = disparity_msg->header.stamp;
    cloud->header.frame_id = disparity_msg->header.frame_id;
    cloud->height = disparity_msg->image.height;
    cloud->width = disparity_msg->image.width;
    cloud->is_dense = false;

    cloud->points.resize (cloud->height * cloud->width);
    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin ();
    for (int v = 0; v < (int)cloud->height; ++v)
    {
      for (int u = 0; u < (int)cloud->width; ++u, ++pt_iter)
      {
        pcl::PointXYZ& pt = *pt_iter;

        float disp = disparity.at<float>(v, u);
        
        // Check for invalid measurements
        if (std::isnan(disp))
        {
          pt.x = pt.y = pt.z = disp;

          /*
          Eigen3::Vector3d xyz;
          uvd_to_fake_depth(Eigen3::Vector3d(u, v, disp), xyz, 3.0);

          pt.x = xyz[0];
          pt.y = xyz[1];
          pt.z = xyz[2];
          */

        }
        else
        {
          Eigen3::Vector3d xyz;
          uvd_to_xyz(Eigen3::Vector3d(u, v, disp), xyz);
          
          pt.x = xyz[0];
          pt.y = xyz[1];
          pt.z = xyz[2];
        }
      }
    }

    //let's downsample the pointcloud
    boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg = boost::make_shared<sensor_msgs::PointCloud2> ();
    pcl::toROSMsg(*cloud, *cloud_msg);

    pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    voxel_filter.setFilterFieldName ("z");
//    voxel_filter.setFilterLimits (0.05, 5);
    voxel_filter.filter(downsampled_cloud);

    pub_points_.publish(downsampled_cloud);
    //pub_points_.publish(cloud_msg);
  }

private:

  ros::Publisher pub_points_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disparity_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo,
                                                          stereo_msgs::DisparityImage> ApproxTimeSync;
  message_filters::Synchronizer<ApproxTimeSync> image_synchronizer_;
};

// TODO: should probably be a nodelet.  Might increase the number of synchronized messages
int main(int argc, char** argv)
{
  ros::init(argc, argv, "disparity_to_cloud");
  ros::NodeHandle nh, p_nh("~");
  DisparityToCloud dtc(nh, p_nh);
  ros::spin();
  return 0;
}
