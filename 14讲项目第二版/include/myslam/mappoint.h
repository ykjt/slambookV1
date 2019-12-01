#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {

struct Frame;

struct Feature;

/**
 * 路标点类
 * 特征点在三角化之后形成路标点
 */
struct MapPoint {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0;  // ID
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero();  // Position in world  地图点的世界坐标
    std::mutex data_mutex_;
    int observed_times_ = 0;  // being observed by feature matching algo.  记录观测次数
    std::list<std::weak_ptr<Feature>> observations_; //记录哪个feature可以看到该地图点（feature有存储哪一帧持有该feature，所以这里的地图点与feame也关联起来了的）

    MapPoint() {}

    MapPoint(long id, Vec3 position);
    //返回该点的世界坐标
    Vec3 Pos() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    ////设置该地图点的 pose      优化位姿 后端线程 会访问该变量
    void SetPos(const Vec3 &pos) {//设置该地图点的 pose  此处考虑多线程同时访问该pose数据的情况，对数据加索，从而对数据进行保护（同时只能有一个人来访问改变量）
        std::unique_lock<std::mutex> lck(data_mutex_);// unique_lock 加锁，没有mutex所有的权限（根据mutex的属性来判断是否能够加锁， 自动加锁自动解锁）
        pos_ = pos;
    };

    //给该地图点设置观测
    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex_);// unique_lock 加锁
        observations_.push_back(feature);//因为可能有多帧图像(左右)中的feature可以观测到该地图点，所以这里将这些帧的feature都存储在该地图点的观测容器中
        observed_times_++;//记录观测次数
    }

    void RemoveObservation(std::shared_ptr<Feature> feat);

    //返回该点的观测   哪些feature可以看到该路标点
    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    // factory function
    static MapPoint::Ptr CreateNewMappoint(); //静态函数（类函数），类名直接调用
};
}  // namespace myslam

#endif  // MYSLAM_MAPPOINT_H
