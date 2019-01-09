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

#include "StatePublisher.h"


namespace ORB_SLAM
{

StatePublisher::StatePublisher(Tracking* tracking):nh("~"), mpTracking(tracking) {

  //Configure Publisher
  mPublisher = nh.advertise<orb_slam::ORBState>("state", 10);

  mMsgState.header.frame_id = "ORB_SLAM";

}

void StatePublisher::Publish()
{
  //save computation if their is no subscriber
  if(mPublisher.getNumSubscribers() > 0 )
  {
    mMsgState.state = static_cast<uint16_t>(mpTracking->mState);
    mMsgState.header.seq++;
    mMsgState.header.stamp = ros::Time::now();
    mPublisher.publish(mMsgState);
  }
}


} //namespace ORB_SLAM
