//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "myslam/common_include.h"

namespace myslam {

/**  内联函数
 * linear triangulation with SVD
 * @param poses     poses,  左右相机的位姿。 左： pose[0] ， 右： pose[1]
 * @param points    points in normalized plane （左右相机的 归一化平面的3D 点）. 左侧相机的归一化的点 point[0]        右侧 point[1]
 * @param pt_world  triangulated point in the world  在世界坐标系下的三维点
 * @return true if success
 */
 //双目相机的地图点三角化.向该函数中传入一对左右相机中的点，生成这个点对应的地图点
inline bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4); //A(行数， 列数)
    VecX b(2 * poses.size());//b(行数)   1列
    b.setZero();

    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }

    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
}

// converters
inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

}  // namespace myslam

#endif  // MYSLAM_ALGORITHM_H
