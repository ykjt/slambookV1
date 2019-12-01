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

#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam {

MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}
//生成一个地图点
MapPoint::Ptr MapPoint::CreateNewMappoint() {//静态函数（类函数），类名直接调用
    static long factory_id = 0;//记录生成地图点的序号。初始时为0
    //生成一个地图点
    MapPoint::Ptr new_mappoint(new MapPoint);//调用默认的构造函数生成地图点
    new_mappoint->id_ = factory_id++;//设置当前地图点的id序号
    return new_mappoint;
}
//传入一个feature，移除该feature对  地图点的 观测
void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
    std::unique_lock<std::mutex> lck(data_mutex_);//拿到锁，并加锁

    //遍历可以看到该地图点的 所有 feature
    for (auto iter = observations_.begin(); iter != observations_.end();
         iter++) {
        //找到观测中 和 要剔除观测的那个feature
        if (iter->lock() == feat) { //weak_ptr可以使用一个非常重要的成员函数lock()从被观测的shared_ptr获得一个可用的shared_ptr对象
            observations_.erase(iter);//移除该feature对这个地图点的观测
            feat->map_point_.reset();
            observed_times_--;//设置该地图点的 观测次数 减一
            break;
        }
    }
}

}  // namespace myslam
