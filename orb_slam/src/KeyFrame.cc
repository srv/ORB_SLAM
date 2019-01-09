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

#include "KeyFrame.h"
#include "Converter.h"
#include <ros/ros.h>

namespace ORB_SLAM
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mfGridElementWidthInv(F.mfGridElementWidthInv),
    mfGridElementHeightInv(F.mfGridElementHeightInv), mnTrackReferenceForFrame(0),mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnRelocQuery(0),fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), mBowVec(F.mBowVec),
    im(F.im), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX), mnMaxY(F.mnMaxY), mK(F.mK),
    mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn), mDescriptors(F.mDescriptors.clone()),
    mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB), mpORBvocabulary(F.mpORBvocabulary), mFeatVec(F.mFeatVec),
    mbFirstConnection(true), mpParent(NULL), mbNotErase(false), mbToBeErased(false), mbBad(false),
    mnScaleLevels(F.mnScaleLevels), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mpMap(pMap)
{
    mnId=nNextId++;

    mnGridCols=FRAME_GRID_COLS;
    mnGridRows=FRAME_GRID_ROWS;
    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Rcw,const cv::Mat &tcw)
{
    boost::mutex::scoped_lock lock(mMutexPose);
    Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    tcw.copyTo(Tcw.col(3).rowRange(0,3));

    Ow=-Rcw.t()*tcw;
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    boost::mutex::scoped_lock lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    Ow = -Rcw.t()*tcw;
}

cv::Mat KeyFrame::GetPose()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
    cv::Mat Rwc = (Tcw.rowRange(0,3).colRange(0,3)).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    twc.copyTo(Twc.rowRange(0,3).col(3));
    return Twc.clone();
}

cv::Mat KeyFrame::GetProjectionMatrix()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return mK*Tcw.rowRange(0,3);
}

cv::Mat KeyFrame::GetCameraCenter()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetRotation()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(std::shared_ptr<KeyFrame>pKF, const int &weight)
{
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    vector<pair<int,std::shared_ptr<KeyFrame>> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<std::shared_ptr<KeyFrame>,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<std::shared_ptr<KeyFrame>> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<std::shared_ptr<KeyFrame>>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<std::shared_ptr<KeyFrame>> KeyFrame::GetConnectedKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    set<std::shared_ptr<KeyFrame>> s;
    for(map<std::shared_ptr<KeyFrame>,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<std::shared_ptr<KeyFrame>> KeyFrame::GetVectorCovisibleKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<std::shared_ptr<KeyFrame>> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<std::shared_ptr<KeyFrame>>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<std::shared_ptr<KeyFrame>> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    boost::mutex::scoped_lock lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<std::shared_ptr<KeyFrame>>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<std::shared_ptr<KeyFrame>>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<std::shared_ptr<KeyFrame>>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(std::shared_ptr<KeyFrame>pKF)
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(std::shared_ptr<MapPoint> pMP, const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mvpMapPoints[idx]=NULL;
}

void KeyFrame::EraseMapPointMatch(std::shared_ptr<MapPoint> pMP)
{
    int idx = pMP->GetIndexInKeyFrame(shared_from_this());
    if(idx>=0)
        mvpMapPoints[idx]=NULL;
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, std::shared_ptr<MapPoint> pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<std::shared_ptr<MapPoint>> KeyFrame::GetMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    set<std::shared_ptr<MapPoint>> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        std::shared_ptr<MapPoint> pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);

    int nPoints=0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(mvpMapPoints[i])
            nPoints++;
    }

    return nPoints;
}

vector<std::shared_ptr<MapPoint>> KeyFrame::GetMapPointMatches()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mvpMapPoints;
}

std::shared_ptr<MapPoint> KeyFrame::GetMapPoint(const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

cv::KeyPoint KeyFrame::GetKeyPointUn(const size_t &idx) const
{
    return mvKeysUn[idx];
}

int KeyFrame::GetKeyPointScaleLevel(const size_t &idx) const
{
    return mvKeysUn[idx].octave;
}

cv::Mat KeyFrame::GetDescriptor(const size_t &idx)
{
    return mDescriptors.row(idx).clone();
}

cv::Mat KeyFrame::GetDescriptors()
{
    return mDescriptors.clone();
}

vector<cv::KeyPoint> KeyFrame::GetKeyPoints() const
{
    return mvKeys;
}

vector<cv::KeyPoint> KeyFrame::GetKeyPointsUn() const
{
    return mvKeysUn;
}

cv::Mat KeyFrame::GetCalibrationMatrix() const
{
    return mK.clone();
}

DBoW2::FeatureVector KeyFrame::GetFeatureVector()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mFeatVec;
}

DBoW2::BowVector KeyFrame::GetBowVector()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mBowVec;
}

cv::Mat KeyFrame::GetImage()
{
    boost::mutex::scoped_lock lock(mMutexImage);
    return im.clone();
}

void KeyFrame::UpdateConnections()
{
    map<std::shared_ptr<KeyFrame>,int> KFcounter;

    vector<std::shared_ptr<MapPoint>> vpMP;

    {
        boost::mutex::scoped_lock lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<std::shared_ptr<MapPoint>>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        std::shared_ptr<MapPoint> pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<std::shared_ptr<KeyFrame>,size_t> observations = pMP->GetObservations();

        for(map<std::shared_ptr<KeyFrame>,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    std::shared_ptr<KeyFrame> pKFmax=NULL;
    int th = 15;

    vector<pair<int,std::shared_ptr<KeyFrame>> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<std::shared_ptr<KeyFrame>,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(shared_from_this(),mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(shared_from_this(),nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<std::shared_ptr<KeyFrame>> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        boost::mutex::scoped_lock lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<std::shared_ptr<KeyFrame>>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(shared_from_this());
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(std::shared_ptr<KeyFrame>pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(std::shared_ptr<KeyFrame>pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}


void KeyFrame::ChangeParent(std::shared_ptr<KeyFrame>pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(shared_from_this());
}

set<std::shared_ptr<KeyFrame>> KeyFrame::GetChilds()
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspChildrens;
}

std::shared_ptr<KeyFrame> KeyFrame::GetParent()
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(std::shared_ptr<KeyFrame>pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(std::shared_ptr<KeyFrame>pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<std::shared_ptr<KeyFrame>> KeyFrame::GetLoopEdges()
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}


void KeyFrame::SetBadFlag()
{   
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<std::shared_ptr<KeyFrame>,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(shared_from_this());

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(shared_from_this());
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        boost::mutex::scoped_lock lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<std::shared_ptr<KeyFrame>> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            std::shared_ptr<KeyFrame> pC;
            std::shared_ptr<KeyFrame> pP;

            for(set<std::shared_ptr<KeyFrame>>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                std::shared_ptr<KeyFrame> pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<std::shared_ptr<KeyFrame>> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<std::shared_ptr<KeyFrame>>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<std::shared_ptr<KeyFrame>>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(shared_from_this());
        mbBad = true;
    }


    mpMap->EraseKeyFrame(shared_from_this());
    mpKeyFrameDB->erase(shared_from_this());
}

bool KeyFrame::isBad()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(std::shared_ptr<KeyFrame> pKF)
{
    bool bUpdate = false;
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(mvKeysUn.size());

    int nMinCellX = floor((x-mnMinX-r)*mfGridElementWidthInv);
    nMinCellX = max(0,nMinCellX);
    if(nMinCellX>=mnGridCols)
        return vIndices;

    int nMaxCellX = ceil((x-mnMinX+r)*mfGridElementWidthInv);
    nMaxCellX = min(mnGridCols-1,nMaxCellX);
    if(nMaxCellX<0)
        return vIndices;

    int nMinCellY = floor((y-mnMinY-r)*mfGridElementHeightInv);
    nMinCellY = max(0,nMinCellY);
    if(nMinCellY>=mnGridRows)
        return vIndices;

    int nMaxCellY = ceil((y-mnMinY+r)*mfGridElementHeightInv);
    nMaxCellY = min(mnGridRows-1,nMaxCellY);
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(abs(kpUn.pt.x-x)<=r && abs(kpUn.pt.y-y)<=r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

float KeyFrame::ComputeSceneMedianDepth(int q)
{
    vector<std::shared_ptr<MapPoint>> vpMapPoints;
    cv::Mat Tcw_;
    {
    boost::mutex::scoped_lock lock(mMutexFeatures);
    boost::mutex::scoped_lock lock2(mMutexPose);
    vpMapPoints = mvpMapPoints;
    Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(mvpMapPoints.size());
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            std::shared_ptr<MapPoint> pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
