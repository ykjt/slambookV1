

#include "myslam/camera.h"
#include "myslam/config.h"

namespace myslam
{

Camera::Camera()//在构造函数中对相机的参数做赋值。因为在这个把配置文件的参数读出来，赋值给一个camera对象，这种面向对象开发。
{
    fx_ = Config::get<float>("camera.fx");//因为这个get()函数是一个静态成员函数，直接调用
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}
/*
 * 传入世界坐标系的点  和 Tcw 返回该点在相机坐标系下的三维点
 * */
Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w )
{
    return T_c_w*p_w;
}
/*
 * 传入相机坐标系的 三维点 和 变换矩阵 得到世界坐标系下的三维点
 * */
Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{   //Pw = Twc * Pc
    return T_c_w.inverse() *p_c;
}
//传入相机坐标系下的三维坐标 返回 像素坐标
Vector2d Camera::camera2pixel ( const Vector3d& p_c )
{
    /* *  u = fx * (x/z) + cx
       *  v = fy * (y/z) + cy
     * */
    return Vector2d (
        fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
        fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
    );
}
//传入像素坐标(u, v)和深度d (Z) 得到相机坐标系下的三维点(x,y,x)
Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{/*
 *  x = (u - cx) * z /fx
 *  y = (v - cy) * z /fy
 * */
    return Vector3d (
        ( p_p ( 0,0 )-cx_ ) *depth/fx_,
        ( p_p ( 1,0 )-cy_ ) *depth/fy_,
        depth
    );
}
//传入世界坐标系的三维点和Tcw ，返回该点在那一帧图像中的像素坐标
Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
{
    return camera2pixel ( world2camera ( p_w, T_c_w ) );
}
/*
 * 传入像素坐标和对应的深度值 和 变换矩阵 得到 世界坐标系下的三维点
 * */
Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
{
    //先由像素坐标和深度值得到相机坐标系下的三维点
    //再将相机坐标系下的点通过SE3得到世界坐标系下的三维点
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


}
