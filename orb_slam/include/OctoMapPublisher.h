/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OCTOMAPPUBLISHER_H
#define OCTOMAPPUBLISHER_H

#include <memory>

#include<ros/ros.h>

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>

#include <octomap_msgs/GetOctomap.h>

#include <pcl_ros/point_cloud.h>

#include <tf/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include "orb_slam/SaveOctomap.h"

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"


namespace ORB_SLAM
{

/**
 * This class provides services for providing octomaps and publishes point clouds
 */
class OctoMapPublisher
{
public:
  OctoMapPublisher(Map* pMap);

  void Publish();

private:

  bool save(string filename);

  void refreshMap(octomap::OcTree& octoMap);

  void reset(octomap::OcTree& octoMap);

  void mapPointsToOctomap(const std::vector<std::shared_ptr<MapPoint>> &vpMPs, octomap::OcTree& m_octoMap);

  void mapPointsToPCL(const std::vector<std::shared_ptr<MapPoint>> &vpRefMPs, pcl::PointCloud<pcl::PointXYZ>& pclCloud);

  void octomapToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map){
    octomapToOccupancyGrid(octree, map, -1.0*std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  }

  void octomapToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, const double minZ_, const double maxZ_ );

  bool octomapBinarySrv(octomap_msgs::GetOctomapRequest  &req,
                        octomap_msgs::GetOctomapResponse &res);

  bool octomapFullSrv(octomap_msgs::GetOctomapRequest  &req,
                      octomap_msgs::GetOctomapResponse &res);

  bool octomapSaveSrv(orb_slam::SaveOctomapRequest  &req,
                      orb_slam::SaveOctomapResponse &res);

  bool occupancySrv(nav_msgs::GetMapRequest  &req,
                                      nav_msgs::GetMapResponse &res);

  void publishPointCloud();
  void publishOctomap();
  void publishProjectedMap();

  ros::NodeHandle nh;

  double m_projectionMinHeight;
  double m_octomapResolution;

  Map* mpMap;

  //Store the last update counter that have been processed
  unsigned int mbLastMapUpdateIdx;

  const char* MAP_FRAME_ID;
  const char* CAMERA_FRAME_ID;
  const char* BASE_LINK_FRAME_ID;

  ros::Publisher publisherPCL;
  ros::Publisher publisherOctomapFull;
  ros::Publisher publisherOctomapBinary;
  ros::Publisher publisherProjected;

  ros::ServiceServer m_octomapSaveService;
  ros::ServiceServer m_octomapBinaryService;
  ros::ServiceServer m_octomapFullService;
  ros::ServiceServer m_occupancyService;

  tf::TransformListener m_tf_listener;
};

} //namespace ORB_SLAM

#endif // OCTOMAPPUBLISHER_H
