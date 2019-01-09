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

#include "OctoMapPublisher.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTreeKey.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>

#define DEFAULT_OCTOMAP_RESOLUTION 0.5

#define PROJECTION_MIN_HEIGHT 0.01


namespace ORB_SLAM
{

using namespace octomap;

void OctoMapPublisher::reset(octomap::OcTree& octoMap)
{
  octoMap.clear();
  mbLastMapUpdateIdx = mpMap->GetMapUpdateIdx() - 1; //decrease in order to force an update
//  ParameterServer* ps = ParameterServer::instance();
//  m_octoMap.setClampingThresMin(ps->get<double>("octomap_clamping_min"));
//  m_octoMap.setClampingThresMax(ps->get<double>("octomap_clamping_max"));
//  m_octoMap.setResolution(ps->get<double>("octomap_resolution"));
//  m_octoMap.setOccupancyThres(ps->get<double>("octomap_occupancy_threshold"));
//  m_octoMap.setProbHit(ps->get<double>("octomap_prob_hit"));
//  m_octoMap.setProbMiss(ps->get<double>("octomap_prob_miss"));

}

bool OctoMapPublisher::octomapBinarySrv(octomap_msgs::GetOctomapRequest  &req,
                                        octomap_msgs::GetOctomapResponse &res)
{
  static octomap::OcTree octoMap(m_octomapResolution);
  refreshMap(octoMap);
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = MAP_FRAME_ID;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(octoMap, res.map)){
    return false;
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool OctoMapPublisher::octomapFullSrv(octomap_msgs::GetOctomapRequest  &req,
                                      octomap_msgs::GetOctomapResponse &res)
{
  static octomap::OcTree octoMap(m_octomapResolution);
  refreshMap(octoMap);
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = MAP_FRAME_ID;
  res.map.header.stamp = ros::Time::now();

  if (!octomap_msgs::fullMapToMsg(octoMap, res.map)){
    return false;
  }

  return true;
}

bool OctoMapPublisher::save(string filename)
{
  static octomap::OcTree octoMap(m_octomapResolution);
  refreshMap(octoMap);

  filename.append(".ot");

  std::ofstream outfile(filename.c_str(), std::ios_base::out | std::ios_base::binary);
  if (outfile.is_open()){
    ROS_DEBUG("Writing octomap to %s", filename.c_str());
    octoMap.write(outfile);
    outfile.close();
    ROS_DEBUG("color tree written %s", filename.c_str());
    return true;
  }
  else {
    ROS_ERROR("could not open  %s for writing", filename.c_str());
    return false;
  }
}

bool OctoMapPublisher::octomapSaveSrv(orb_slam::SaveOctomapRequest  &req,
                                      orb_slam::SaveOctomapResponse &res)
{
  return save(req.filename);
}

bool OctoMapPublisher::occupancySrv(nav_msgs::GetMapRequest  &req,
                                    nav_msgs::GetMapResponse &res)
{
  static octomap::OcTree octoMap(m_octomapResolution);
  refreshMap(octoMap);
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = MAP_FRAME_ID;
  res.map.header.stamp = ros::Time::now();

  //XXX no fixed limits
  octomapToOccupancyGrid(octoMap, res.map, m_projectionMinHeight, std::numeric_limits<double>::max());

  return true;
}


OctoMapPublisher::OctoMapPublisher(Map* pMap):nh("~"),
    m_projectionMinHeight(PROJECTION_MIN_HEIGHT),
    m_octomapResolution(DEFAULT_OCTOMAP_RESOLUTION),
    mpMap(pMap),
    mbLastMapUpdateIdx(0),
    MAP_FRAME_ID("/orb_slam/map"),
    CAMERA_FRAME_ID("/ORB_SLAM/Camera"),
    BASE_LINK_FRAME_ID("ORB_base_link")
{

  nh.param<double>("occupancy_projection_min_height", m_projectionMinHeight, m_projectionMinHeight);
  nh.param<double>("octomap_resolution", m_octomapResolution, m_octomapResolution);

  mbLastMapUpdateIdx = mpMap->GetMapUpdateIdx()-1; //decrease in order to force an update

  //Configure Publisher
  publisherPCL = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
  publisherOctomapFull = nh.advertise<octomap_msgs::Octomap>("octomap_full", 10);
  publisherOctomapBinary = nh.advertise<octomap_msgs::Octomap>("octomap_binary", 10);
  publisherProjected = nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, 10);

  //Configure Services
  m_octomapSaveService = nh.advertiseService("save_octomap", &OctoMapPublisher::octomapSaveSrv, this);
  m_octomapBinaryService = nh.advertiseService("octomap_binary", &OctoMapPublisher::octomapBinarySrv, this);
  m_octomapFullService = nh.advertiseService("octomap_full", &OctoMapPublisher::octomapFullSrv, this);
  m_occupancyService = nh.advertiseService("occupancy_grid", &OctoMapPublisher::occupancySrv, this);
}

void OctoMapPublisher::octomapToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, const double minZ_, const double maxZ_ )
{
  map.info.resolution = octree.getResolution();
  double minX, minY, minZ;
  double maxX, maxY, maxZ;
  octree.getMetricMin(minX, minY, minZ);
  octree.getMetricMax(maxX, maxY, maxZ);
  ROS_DEBUG("Octree min %f %f %f", minX, minY, minZ);
  ROS_DEBUG("Octree max %f %f %f", maxX, maxY, maxZ);
  minZ = std::max(minZ_, minZ);
  maxZ = std::min(maxZ_, maxZ);

  octomap::point3d minPt(minX, minY, minZ);
  octomap::point3d maxPt(maxX, maxY, maxZ);
  octomap::OcTreeKey minKey, maxKey, curKey;

  if (!octree.coordToKeyChecked(minPt, minKey))
  {
    ROS_ERROR("Could not create OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
    return;
  }
  if (!octree.coordToKeyChecked(maxPt, maxKey))
  {
    ROS_ERROR("Could not create OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
    return;
  }

  map.info.width = maxKey[0] - minKey[0] + 1;
  map.info.height = maxKey[1] - minKey[1] + 1;

  // might not exactly be min / max:
  octomap::point3d origin =   octree.keyToCoord(minKey, octree.getTreeDepth());
  map.info.origin.position.x = origin.x() - octree.getResolution() * 0.5;
  map.info.origin.position.y = origin.y() - octree.getResolution() * 0.5;

  map.info.origin.orientation.x = 0.;
  map.info.origin.orientation.y = 0.;
  map.info.origin.orientation.z = 0.;
  map.info.origin.orientation.w = 1.;

  // Allocate space to hold the data
  map.data.resize(map.info.width * map.info.height, -1);

  //init with unknown
  for(std::vector<int8_t>::iterator it = map.data.begin(); it != map.data.end(); ++it) {
      *it = -1;
  }

  // iterate over all keys:
  unsigned i, j;
  for (curKey[1] = minKey[1], j = 0; curKey[1] <= maxKey[1]; ++curKey[1], ++j)
  {
    for (curKey[0] = minKey[0], i = 0; curKey[0] <= maxKey[0]; ++curKey[0], ++i)
    {
      for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; ++curKey[2])
      { //iterate over height
        octomap::OcTreeNode* node = octree.search(curKey);
        if (node)
        {
          bool occupied = octree.isNodeOccupied(node);
          if(occupied){
            map.data[map.info.width * j + i] = 100;
            break;
          }else{
            map.data[map.info.width * j + i] = 0;
          }
        }
      }
    }
  }
}

void OctoMapPublisher::publishProjectedMap()
{
  static nav_msgs::OccupancyGrid msgOccupancy;
  if (publisherProjected.getNumSubscribers() > 0)
  {
    static octomap::OcTree octoMap(m_octomapResolution);
    ros::Time now = ros::Time::now();
    refreshMap(octoMap);
    msgOccupancy.header.frame_id = MAP_FRAME_ID;
    msgOccupancy.header.stamp = now;

    octomapToOccupancyGrid(octoMap, msgOccupancy, m_projectionMinHeight, std::numeric_limits<double>::max());

    publisherProjected.publish(msgOccupancy);

  }
}

void OctoMapPublisher::publishPointCloud()
{
  static sensor_msgs::PointCloud2 msgPointCloud;
  //save computation if their is no subscriber
  if (publisherPCL.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    vector<std::shared_ptr<MapPoint>> vMapPoints = mpMap->GetAllMapPoints();
    vector<std::shared_ptr<MapPoint>> vRefMapPoints = mpMap->GetReferenceMapPoints();
    mapPointsToPCL(vRefMapPoints, pclCloud);
    pcl::toROSMsg(pclCloud, msgPointCloud);
    msgPointCloud.header.frame_id = CAMERA_FRAME_ID;
    msgPointCloud.header.stamp = ros::Time::now();
    msgPointCloud.header.seq++;
    publisherPCL.publish(msgPointCloud);
  }
}

void OctoMapPublisher::publishOctomap()
{
  if (publisherOctomapBinary.getNumSubscribers() > 0 || publisherOctomapFull.getNumSubscribers() > 0)
  {
    static octomap::OcTree octoMap(m_octomapResolution);
    ros::Time now = ros::Time::now();
    refreshMap(octoMap);

    if (publisherOctomapBinary.getNumSubscribers() > 0)
    {
      static octomap_msgs::Octomap msgOctomapBinary;
      msgOctomapBinary.header.frame_id = MAP_FRAME_ID;
      msgOctomapBinary.header.stamp = now;

      if (octomap_msgs::binaryMapToMsg(octoMap, msgOctomapBinary))
      {
        publisherOctomapBinary.publish(msgOctomapBinary);
      }
    }
    if (publisherOctomapFull.getNumSubscribers() > 0)
    {
      static octomap_msgs::Octomap msgOctomapFull;
      msgOctomapFull.header.frame_id = MAP_FRAME_ID;
      msgOctomapFull.header.stamp = now;
      if (octomap_msgs::fullMapToMsg(octoMap, msgOctomapFull))
      {
        publisherOctomapFull.publish(msgOctomapFull);
      }
    }
  }
}

void OctoMapPublisher::Publish()
{
  //XXX think about special treatment
  publishPointCloud();

  if(mpMap->isMapUpdated(mbLastMapUpdateIdx))
  {
    publishOctomap();

    publishProjectedMap();

    //reset update indicator
    mbLastMapUpdateIdx = mpMap->GetMapUpdateIdx();
  }
}

void OctoMapPublisher::refreshMap(octomap::OcTree& octoMap)
{
  reset(octoMap);
  vector<std::shared_ptr<MapPoint>> vMapPoints = mpMap->GetAllMapPoints();

  mapPointsToOctomap(vMapPoints, octoMap);
}

void OctoMapPublisher::mapPointsToOctomap(const vector<std::shared_ptr<MapPoint>> &vpMPs, octomap::OcTree& octoMap)
{
  Pointcloud pointCloud;

  for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
  {
    if(vpMPs[i]->isBad()){
        continue;
    }
    cv::Mat pos = vpMPs[i]->GetWorldPos();

    pointCloud.push_back(pos.at<float>(0), pos.at<float>(1),pos.at<float>(2));
  }

  if(pointCloud.size() == 0)
  {
    return;
  }

  octomap::point3d origin;

  try{

    tf::StampedTransform transform_in_target_frame;

    m_tf_listener.lookupTransform(BASE_LINK_FRAME_ID, CAMERA_FRAME_ID, ros::Time(0) , transform_in_target_frame);

    octomap::pose6d frame = octomap::poseTfToOctomap(transform_in_target_frame);

    //TODO test discretize=true
    //TODO potential option insert only current like used for point cloud and integrate octomap
    octoMap.insertPointCloud(pointCloud, origin, frame, -1, false, false);
  }catch(...){
    ROS_ERROR("TF from %s to %s not available for point cloud generation", BASE_LINK_FRAME_ID, CAMERA_FRAME_ID);
    octoMap.insertPointCloud(pointCloud, origin, -1, false, false);
  }

}

void OctoMapPublisher::mapPointsToPCL(const vector<std::shared_ptr<MapPoint>> &vpRefMPs, pcl::PointCloud<pcl::PointXYZ>& pclCloud)
{
  //only current (local) mapping points are taking into account
  for(size_t i=0, iend=vpRefMPs.size(); i<iend;i++)
  {
    if(vpRefMPs[i]->isBad())
    {
        continue;
    }
    cv::Mat pos = vpRefMPs[i]->GetWorldPos();

    pcl::PointXYZ point(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    pclCloud.points.push_back(point);
  }
  pclCloud.height = 1; //unstructured cloud
  pclCloud.width = pclCloud.points.size();
}

} //namespace ORB_SLAM
