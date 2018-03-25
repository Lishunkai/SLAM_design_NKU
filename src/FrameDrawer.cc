/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include<mutex>

extern cv::Mat PoseTcw; // 声明，在这里要用到一个全局变量。这个全局变量是在别的函数中定义的

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                // 画特征点匹配跟踪的结果
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size(); // KeyPoints in current frame
        for(int i=0;i<n;i++)
        {
            // 针对每个ORB特征点，画绿色的方框
            if(vbVO[i] || vbMap[i]) // Tracked MapPoints in current frame
            {
                
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1); // 绿
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1); // 蓝
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

// 画这一帧的稀疏深度图
cv::Mat FrameDrawer::DrawSparseDepthMap()
{
    cv::Mat im = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        cv::putText(im,"SLAM Initializing",cvPoint(im.cols/2-200,im.rows/2),cv::FONT_HERSHEY_SIMPLEX,1.5,cvScalar(255,255,255),2);     
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        const int n = vCurrentKeys.size(); // number of KeyPoints in the current frame

        // 针对每个ORB特征点，画深度
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            // vbVO[i] and vbMap[i] are tracked MapPoints in current frame
            // A match to a MapPoint (vbMap[i]) in the map, or a match to a "visual odometry" MapPoint (vbVO[i]) created in the last frame
            {   
                // 这里的bug调了7个小时才调出来。
                // 原来的写法是：
                // vector<cv::Mat> WorldPosOfCurrentMapPoints;
                // WorldPosOfCurrentMapPoints[i].push_back(pTracker->mCurrentFrame.mvpMapPoints[i]->GetWorldPos());
                // 即：将特征点的世界坐标储存在 WorldPosOfCurrentMapPoints[i]中;
                // 这样操作的结果是：WorldPosOfCurrentMapPoints[i]可以直接输出，但无法访问其矩阵的元素，错误类型为segmentation fault，或值非常大/小。
                // 这是一个非常奇怪的问题。我调了很长时间，最终觉得原因很可能是：
                // pTracker->mCurrentFrame.mvpMapPoints[i]->GetWorldPos()中的东西都是和指针有关的，即*和&。
                // 这样在通过push_back()函数，将GetWorldPos()赋值给WorldPosOfCurrentMapPoints[i]的时候，相当于把GetWorldPos()的数据类型也赋值给WorldPosOfCurrentMapPoints[i]了
                // *和&的特点是，它们都指向同一个内存空间，只是变量的别名不同。对任何别名的改动，都会造成对该内存中值的改动。
                // 这样以来，WorldPosOfCurrentMapPoints[i]可以通过cout查看，但如果用.at<double>(a,b)读取的时候，就一定会出问题。
                // 因为pTracker、mCurrentFrame、mvpMapPoints其实都是在别处不断使用、更新的变量，更重要的是它们在不同的线程中。
                // 因此，用.at<double>(a,b)访问的时候，很可能该处内存值不是你想要的值，这可能就是读取的值非常大或非常小的原因。
                // 而且这些变量在多线程实现中都用mutex保护起来了，当你访问该变量的时候，可能它不允许你访问。这可能就是segmentation fault的原因。

                int A = 225-Depth[i]*10;
                int B = 225-Depth[i]*10;
                cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(225,A,B),-1); // 绿
            // else
            //     cv::circle(im,vCurrentKeys[i].pt,3,cv::Scalar(0,0,255),-1); // 红
            }
        }

        Depth.clear(); // 清空数组  一定要清空，否则数组会越变越大，访问的时候下标也容易有错
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        cv::Mat CameraCenter = (pTracker->mCurrentFrame).GetCameraCenter();
        
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];

            if(pMP)
            {
                // 将当前能看到的所有MapPoint的世界坐标保存下来
                // PointWorldPos = pTracker->mCurrentFrame.mvpMapPoints[i]->GetWorldPos();
                // PosX.push_back((pTracker->mCurrentFrame.mvpMapPoints[i]->GetWorldPos()).at<double>(0,0));
                // PosY.push_back((pTracker->mCurrentFrame.mvpMapPoints[i]->GetWorldPos()).at<double>(1,0));
                // PosZ.push_back((pTracker->mCurrentFrame.mvpMapPoints[i]->GetWorldPos()).at<double>(2,0));

                cv::Mat PC = ((pTracker->mCurrentFrame).mvpMapPoints[i])->GetWorldPos() - CameraCenter;
                float dist = cv::norm(PC); // 计算范数
                Depth.push_back(dist);
                // 以上两句话放在if的作用域中，是为了保证pMP是有定义的，否则pMP可能是一个空指针，导致core dumped
                
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {   
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
            else // 若地图点是未定义的，则保存初始值
            {
                Depth.push_back(0);
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM