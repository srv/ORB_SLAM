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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <memory>

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace ORB_SLAM
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<std::shared_ptr<KeyFrame>> &vpKF, const std::vector<std::shared_ptr<MapPoint>> &vpMP, int nIterations = 5, bool *pbStopFlag=NULL);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL);
    void static LocalBundleAdjustment(std::shared_ptr<KeyFrame> pKF, bool *pbStopFlag=NULL);
    int static PoseOptimization(Frame* pFrame);

    void static OptimizeEssentialGraph(Map* pMap, std::shared_ptr<KeyFrame> pLoopKF, std::shared_ptr<KeyFrame> pCurKF, g2o::Sim3 &Scurw,
                                       LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       std::map<std::shared_ptr<KeyFrame>, set<std::shared_ptr<KeyFrame>> > &LoopConnections);


    static int OptimizeSim3(std::shared_ptr<KeyFrame> pKF1, std::shared_ptr<KeyFrame> pKF2, std::vector<std::shared_ptr<MapPoint>> &vpMatches1, g2o::Sim3 &g2oS12, float th2 = 10);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
