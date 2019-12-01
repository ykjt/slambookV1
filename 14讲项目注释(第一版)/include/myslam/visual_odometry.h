/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    VOState     state_;     // current VO status //枚举类型  初始化为INITIALIZING
    Map::Ptr    map_;       // map with all frames and map points //构造函数中，初始化一个这个
    
    Frame::Ptr  ref_;       // reference key-frame 初始化为空指针
    Frame::Ptr  curr_;      // current frame   初始化为空指针
    
    cv::Ptr<cv::ORB> orb_;  // orb detector and computer 该指针成员函数在构造函数中已经赋值，用来提取特征点;  VisualOdometry在系统中只有一个，所以这个对象也只有一个。每帧都使用这个对象进行提取就好了，不用没帧都创建一个，浪费时间
    vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame 当前帧的关键点
    Mat                     descriptors_curr_;  // descriptor in current frame 当前帧的描述子
    
    //在匹配器中，所需要的匹配变成了地图点和帧中的关键点
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher  初始化为new cv::flann::LshIndexParams ( 5,10,2 )
    vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points 匹配的3d点
    vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr) 匹配上的2d点
   
    SE3 T_c_w_estimated_;    // the estimated pose of current frame   由pnp求解的初始位姿，还没有优化
    int num_inliers_;        // number of inlier features in icp  icp中内点的个数，初始化为0（在程序中这个地方是PNP中内点的个数）
    int num_lost_;           // number of lost times 初始化为0。这个参数是记录 连续跟踪丢失帧的次数
    
    // parameters  下面这些参数在该构造函数中，就从配置文件中读取并附值
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;     // number of pyramid levels
    float match_ratio_;     // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times 最大连续丢失次数
    int min_inliers_;       // minimum inliers  最小的内点数
    double key_frame_min_rot;   // minimal rotation of two key-frames两个关键帧的最小旋转角度
    double key_frame_min_trans; // minimal translation of two key-frames两个关键帧的最小平移
    double  map_point_erase_ratio_; // remove map point ratio, 地图点剔除的比例. 定义 匹配率=用匹配次数/可见次数 如果匹配率小于这个map_point_erase_ratio_则从地图中删除该点
    
public: // functions 
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    
protected:  
    // inner operation 
    void extractKeyPoints();
    void computeDescriptors(); 
    void featureMatching();
    void poseEstimationPnP(); 
    void optimizeMap(); //新增,对地图进行优化，包括删除不在视野内的点，在匹配数量减少时添加新点
    
    void addKeyFrame();
    void addMapPoints();  //新增
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
    
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point ); //新增
    
};
}

#endif // VISUALODOMETRY_H