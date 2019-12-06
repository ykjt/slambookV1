#include "myslam/frame.h"

namespace myslam
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr)
{

}

Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth ) //id不能为空，必须传入参数。其他的向参有默认参数。  默认参数的设置时必须在声明中写
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0; //静态变量，存储在全局区，整个系统就一个
    return Frame::Ptr( new Frame(factory_id++) );//这里根据id创建一个新的帧，并将该帧返回;
}
//传入一个彩色图像中的像素坐标，返回对应深度图中的深度值
double Frame::findDepth ( const cv::KeyPoint& kp )
{
    //像素坐标
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    //从深度图中取出深度值
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {//深度值不为0,则是有效的深度值
        return double(d)/camera_->depth_scale_;//取出的值除以深度相机的尺度因子，得到实际的深度  米 为单位
    }
    else 
    {//如果当前点深度值为0 ，检查周围的点的深度值，如果周围的点有深度则返回周围点的深度值
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        //遍历当前点附近的点
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;
}

//相机坐标系的（0,0,0）在世界坐标系下的位置
Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();  //相机坐标系的（0,0,0）在世界坐标系下的位置
}
// check if a point is in this frame
//判断一个世界坐标系下的3D点 是否在该帧 中
bool Frame::isInFrame ( const Vector3d& pt_world ) //传入地图点在世界坐标系点的位置
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );//返回该点在相机坐标系下的三维点
    if ( p_cam(2,0)<0 ) //深度小于0 则返回
        return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );//返回该点在该帧图像中的像素坐标
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}

}