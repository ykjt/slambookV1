#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
    
class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long      id_;        // ID  当前地图点的id号
    static unsigned long factory_id_;    // factory id  记录添加到了哪个id号了，静态成员变量，存储在全局区
    bool        good_;      // wheter a good point  初始化时为  true
    Vector3d    pos_;       // Position in world  地图点在世界坐标系点的位置
    Vector3d    norm_;      // Normal of viewing direction 观测方向  保存的是参考关键帧观测到该点的方向向量。（那么，如果下一帧来了，通过计算该点和下一帧的方向向量，那么就可以知道下一帧和参考关键帧看到该点的视差角）
    Mat         descriptor_; // Descriptor for matching  该地图点的描述子
    
    list<Frame*>    observed_frames_;   // key-frames that can observe this point  存储 可以观察这一点的关键帧

    int         matched_times_;     // being an inliner in pose estimation  记录匹配次数 初始化为1
    int         visible_times_;     // being visible in current frame 在当前帧中可见  初始化为1。记录该点被观测到的次数
    
    MapPoint();
    MapPoint( //构造函数   构造一个地图点
        unsigned long id, 
        const Vector3d& position, 
        const Vector3d& norm, 
        Frame* frame=nullptr, 
        const Mat& descriptor=Mat() 
    );
    //返回3d点位置，由Vector3d类型变为Point3f类型
    inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }
    
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint( 
        const Vector3d& pos_world, 
        const Vector3d& norm_,
        const Mat& descriptor,
        Frame* frame );
};
}

#endif // MAPPOINT_H