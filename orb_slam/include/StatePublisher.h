/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* Copyright (C) 2015 Christopher-Eyk Hrabia <christopher-eyk.hrabia@dai-labor.de> (Technische Universität Berlin)
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

#ifndef STATEPUBLISHER_H
#define STATEPUBLISHER_H

#include<ros/ros.h>

#include"Tracking.h"
#include <orb_slam/ORBState.h>

namespace ORB_SLAM
{

/**
 * This class publishes the current SLAM state
 */
class StatePublisher
{
public:
   StatePublisher(Tracking* tracking);

   void Publish();

private:

   ros::NodeHandle nh;

   Tracking* mpTracking;

   ros::Publisher mPublisher;

   orb_slam::ORBState mMsgState;

};

} //namespace ORB_SLAM

#endif // STATEPUBLISHER_H
