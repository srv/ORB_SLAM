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

#include "Map.h"

namespace ORB_SLAM
{

Map::Map()
{
    mbMapUpdateIdx= false;
    mnMaxKFid = 0;
}

void Map::AddKeyFrame(std::shared_ptr<KeyFrame>pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    mbMapUpdateIdx++;
}

void Map::AddMapPoint(std::shared_ptr<MapPoint> pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspMapPoints.insert(pMP);
    mbMapUpdateIdx++;
}

void Map::EraseMapPoint(std::shared_ptr<MapPoint> pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspMapPoints.erase(pMP);
    mbMapUpdateIdx++;
}

void Map::EraseKeyFrame(std::shared_ptr<KeyFrame>pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    mbMapUpdateIdx++;
}

void Map::SetReferenceMapPoints(const vector<std::shared_ptr<MapPoint>> &vpMPs)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
    mbMapUpdateIdx++;
}

vector<std::shared_ptr<KeyFrame>> Map::GetAllKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<std::shared_ptr<KeyFrame>>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<std::shared_ptr<MapPoint>> Map::GetAllMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<std::shared_ptr<MapPoint>>(mspMapPoints.begin(),mspMapPoints.end());
}

int Map::MapPointsInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mspMapPoints.size();
}

int Map::KeyFramesInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<std::shared_ptr<MapPoint>> Map::GetReferenceMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<std::shared_ptr<MapPoint>>(mvpReferenceMapPoints.begin(),mvpReferenceMapPoints.end());
}

unsigned int Map::GetMapUpdateIdx()
{
    return mbMapUpdateIdx;
}

bool Map::isMapUpdated(unsigned int refIdx)
{
    return mbMapUpdateIdx != refIdx;
}

void Map::SetFlagAfterBA()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mbMapUpdateIdx++;

}

unsigned int Map::GetMaxKFid()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
}

} //namespace ORB_SLAM
