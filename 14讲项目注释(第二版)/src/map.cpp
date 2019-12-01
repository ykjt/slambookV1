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

#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam {
//往地图中添加一个关键帧
void Map::InsertKeyFrame(Frame::Ptr frame) {
    current_frame_ = frame;//将传入的帧，赋值给map中的当前帧
    //查找该帧是否已经在地图中.在就更新，不在就插入
    if (keyframes_.find(frame->id_) == keyframes_.end()) {
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    } else {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    //判断活跃的关键帧数量是否大于 设定的数量（7）。如果大于，就要控制其数量在范围内（类似滑动窗口）
    if (active_keyframes_.size() > num_active_keyframes_) {
        // 将旧的关键帧置为不活跃状态。
        //同时还要移除这些被移除帧对应的地图点
        RemoveOldKeyframe();
    }
}
//向地图中添加 一个地图点
void Map::InsertMapPoint(MapPoint::Ptr map_point) {
    //landmarks_  是一个map类型。键值对的形式
    if (landmarks_.find(map_point->id_) == landmarks_.end()) {//如果没有查找到的话，那返回末尾的这个迭代器
        landmarks_.insert(make_pair(map_point->id_, map_point));//没有这个地图点，就说明这个地图点没有添加过。插入
        active_landmarks_.insert(make_pair(map_point->id_, map_point));//将该点插入到活跃的地图点中
    } else {//如果发现插入的地图点已经存在
        landmarks_[map_point->id_] = map_point;//根据地图点的id更新该地图点
        active_landmarks_[map_point->id_] = map_point;//并且更新活跃地图点
    }
}
//移除老的关键帧 // 将旧的关键帧置为不活跃状态
void Map::RemoveOldKeyframe() {
    if (current_frame_ == nullptr) return;

    // 寻找与当前帧最近与最远的两个关键帧
    double max_dis = 0, min_dis = 9999;//记录active_keyframe与current_frame之间的最大和最小距离
    double max_kf_id = 0, min_kf_id = 0;//记录active_keyframe与current_frame之间的最大和最小距离 的 关键帧的keyframe_id

    auto Twc = current_frame_->Pose().inverse();//当前帧的位姿
    //遍历活跃关键帧  找与当前帧最近与最远的两个关键帧
    for (auto& kf : active_keyframes_) {
        if (kf.second == current_frame_) continue;//如果遍历到当前帧，则continue

        //计算当前遍历关键帧 和 当前帧 位姿的差   （求距离）
        // SE3  :    (T_c1_w * T_w_curr )
        //                               .log() 转换为李代数
        //                                        .norm() 再求其二范数
        auto dis = (kf.second->Pose() * Twc).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;  // 最近阈值
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
        // 如果存在很近的帧，优先删掉最近的
        frame_to_remove = keyframes_.at(min_kf_id);//根据keyframe_id 在keyframes_容器中查找到该 关键帧
    } else {
        // 删掉最远的
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
    // remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_remove->keyframe_id_);//将这一关键帧从活跃的关键帧中（窗口中）删除
    //将这一帧从窗口中剔除后，还有从地图中剔除 该帧对应的地图点 的观测
    for (auto feat : frame_to_remove->features_left_) {//剔除左侧图像对应的地图点的观测
        auto mp = feat->map_point_.lock(); //weak_ptr可以使用一个非常重要的成员函数lock()从被观测的shared_ptr获得一个可用的shared_ptr对象
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }
    for (auto feat : frame_to_remove->features_right_) {//剔除右侧图像对应的地图点的观测
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }
    //清扫地图，
    CleanMap();
}
//清扫地图，
void Map::CleanMap() {
    int cnt_landmark_removed = 0;//记录剔除地图点的个数
    //遍历地图中的所有活跃的地图点（窗口中的地图点），如果其被观测的数量为0,那么剔除该点
    for (auto iter = active_landmarks_.begin();
         iter != active_landmarks_.end();) {
        if (iter->second->observed_times_ == 0) {//观测次数为0
            iter = active_landmarks_.erase(iter);//删除
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
}

}  // namespace myslam
