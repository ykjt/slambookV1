#include "myslam/g2o_types.h"

namespace myslam
{
void EdgeProjectXYZRGBD::computeError()
{/*
 * 测量值是相机坐标系下的点，观测值是世界坐标系下的点，左乘外参得到的相机坐标系下的计算值。误差值是测量值减去观测值。

这条边绑定了两个节点，分别是世界坐标系下的点，和位姿；因此雅克比矩阵需要分别对二者求偏导。

对前者（对世界坐标系下的点求偏导），误差e=P测量  -  （R*（P世界）+t），P测量、t看成是常量，因此误差关于世界坐标系下的点的偏导，就是-R，因此，雅克比矩阵如下定义：
_jacobianOplusXi = - T.rotation().toRotationMatrix();
 对于后者（对位姿求偏导），误差e=P测量  - （R*（P世界）+t），P测量、t看成是常量，误差关于扰动小量位姿的偏导，就是 - （R*（P世界）+t）关于扰动小量位姿的偏导，这个在书的76页已经讲过了，答案是[I,-P'^]。（注意对调前三列和后三列）也就是：
    _jacobianOplusXj ( 0,0 ) = 0;....................
 * */
    const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*> ( _vertices[0] );
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[1] );
    _error = _measurement - pose->estimate().map ( point->estimate() ); //误差 = 观测 - 估计值
}

void EdgeProjectXYZRGBD::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[1] );
    g2o::SE3Quat T ( pose->estimate() );
    g2o::VertexSBAPointXYZ* point = static_cast<g2o::VertexSBAPointXYZ*> ( _vertices[0] );
    Eigen::Vector3d xyz = point->estimate();
    Eigen::Vector3d xyz_trans = T.map ( xyz );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    //EdgeProjectXYZRGBD绑定两种顶点，第一种是世界坐标系下的点，第二种是位姿；
    //EdgeProjectXYZRGBD的观测值是相机坐标系下的点；
    //误差值为相机坐标系下的观测点减去计算点。误差对世界坐标系下的点求偏导，即-P'对于P求偏导，即为R
    _jacobianOplusXi = - T.rotation().toRotationMatrix();      

    //误差值为相机坐标系下的观测点减去计算点P'。误差对位姿求偏导，即-P'对于扰动小量求偏导，即为[I,-P'^]，左右两部分内容交换
    _jacobianOplusXj ( 0,0 ) = 0;
    _jacobianOplusXj ( 0,1 ) = -z;   
    _jacobianOplusXj ( 0,2 ) = y;
    _jacobianOplusXj ( 0,3 ) = -1;
    _jacobianOplusXj ( 0,4 ) = 0;
    _jacobianOplusXj ( 0,5 ) = 0;

    _jacobianOplusXj ( 1,0 ) = z;
    _jacobianOplusXj ( 1,1 ) = 0;
    _jacobianOplusXj ( 1,2 ) = -x;
    _jacobianOplusXj ( 1,3 ) = 0;
    _jacobianOplusXj ( 1,4 ) = -1;
    _jacobianOplusXj ( 1,5 ) = 0;

    _jacobianOplusXj ( 2,0 ) = -y;
    _jacobianOplusXj ( 2,1 ) = x;
    _jacobianOplusXj ( 2,2 ) = 0;
    _jacobianOplusXj ( 2,3 ) = 0;
    _jacobianOplusXj ( 2,4 ) = 0;
    _jacobianOplusXj ( 2,5 ) = -1;
}

//观测值是相机坐标系下的点的g2o边，仅仅附着于位姿节点。误差函数如下定义：
void EdgeProjectXYZRGBDPoseOnly::computeError()
{
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    _error = _measurement - pose->estimate().map ( point_ );//误差 = 观测 - 估计值
}
//这也就是上面的EdgeProjectXYZRGBD，只绑定位姿节点。因此雅克比矩阵，也就只有其中的一项，也就是EdgeProjectXYZRGBD的_jacobianOplusXj：
void EdgeProjectXYZRGBDPoseOnly::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    //EdgeProjectXYZRGBDPoseOnly绑定一种顶点，即位姿。观测值是相机坐标系下的点
    //误差值为相机坐标系下的观测点减去计算点P'，误差关于位姿求偏导，即-P'对于扰动小量求偏导，即为[I,-P'^]，左右两部分内容交换
    _jacobianOplusXi ( 0,0 ) = 0;
    _jacobianOplusXi ( 0,1 ) = -z;
    _jacobianOplusXi ( 0,2 ) = y;
    _jacobianOplusXi ( 0,3 ) = -1;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = 0;

    _jacobianOplusXi ( 1,0 ) = z;
    _jacobianOplusXi ( 1,1 ) = 0;
    _jacobianOplusXi ( 1,2 ) = -x;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1;
    _jacobianOplusXi ( 1,5 ) = 0;

    _jacobianOplusXi ( 2,0 ) = -y;
    _jacobianOplusXi ( 2,1 ) = x;
    _jacobianOplusXi ( 2,2 ) = 0;
    _jacobianOplusXi ( 2,3 ) = 0;
    _jacobianOplusXi ( 2,4 ) = 0;
    _jacobianOplusXi ( 2,5 ) = -1;
}


//computeError函数中，描述了error的计算过程，即观测的值，减去计算出的值。
void EdgeProjectXYZ2UVPoseOnly::computeError()
{
//    存储顶点信息，比如二元边的话，_vertices[] 的大小为2，存储顺序和调用setVertex(int, vertex) 是设定的int 有关（0 或1）

    //顶点数组中取出顶点，转换成位姿指针类型，其实左边的pose类型可以写为auto
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );//在添加关联的时候 ，边设置连接的顶点edge->setVertex ( 0, pose ); _vertices[0]指的就是这个
    //误差计算，测量值减去估计值，也就是重投影误差
    //估计值计算方法是T*p,得到相机坐标系下坐标，然后在利用camera2pixel()函数得到像素坐标。
    _error = _measurement - camera_->camera2pixel ( pose->estimate().map(point_) );
    //error = 测量值 - （与这个测量值匹配的3D坐标 变换到 像素坐标的值）
    //误差 = 观测 - 投影
}
// 线性增量函数，也就是雅克比矩阵J的计算方法
void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
{
    /**
 * 这里说一下整体思路：
 * 重投影误差的雅克比矩阵在书中P164页式7.45已经呈现，所以这里就是直接构造，
 * 构造时发现需要变换后的空间点坐标，所以需要先求出。
 */
    //首先还是从顶点取出位姿    //取出顶点，将其强制类型转换为位姿类型。
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    //这由位姿构造一个四元数形式T
    g2o::SE3Quat T ( pose->estimate() );
    //用T求得变换后的3D点坐标。T*p
    Vector3d xyz_trans = T.map ( point_ );
    //OK，到这步，变换后的3D点xyz坐标就分别求出来了，后面的z平方，纯粹是为了后面构造J时方便定义的，因为需要多处用到
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    //EdgeProjectXYZ2UVPoseOnly绑定一种节点，即位姿；
    //误差值为像素坐标系下的观测点减去计算点（u，v），误差关于位姿求导数，即需要进一步划分
    //误差对于位姿求导数，即-（u，v）对于位姿求导数，即-（u，v）先对P'求偏导，P'再对扰动小量位姿求偏导。

    //直接各个元素构造J就好了，对照式7.45是一模一样的，2*6的矩阵。
    //此处交换了前三列和后三列，因为书中推公式的时候，是平移在前，旋转在后，而在g2o中是旋转在前，平移在后
    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->fx_;
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx_;
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx_;
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx_;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->fx_;

    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy_;
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->fy_;
    _jacobianOplusXi ( 1,2 ) = -x/z *camera_->fy_;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z *camera_->fy_;
    _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->fy_;
}
}
//
// * 在EdgeProjectXYZ2UV源码中，
// * 第一项绑定的是世界坐标系下的坐标节点，
// * 第二项是位姿节点，（源码不像这里只绑定位姿节点），
// * 所以，源码中，_jacobianOplusXj是上面的雅克比矩阵，而_jacobianOplusXi是误差关于世界坐标系下坐标的偏导，即-(u,v)先对P'求偏导，再由P'对P求偏导（也就是R）。


