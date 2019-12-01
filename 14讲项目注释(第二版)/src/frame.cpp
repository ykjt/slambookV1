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

#include "myslam/frame.h"

namespace myslam {

Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0; //用来记录生成帧的序号到哪里了。 静态变量存储在内存的全局区，整个程序中只有一个
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++; //用来给当前帧的id赋值
    return new_frame;
}
//将该帧设置为关键帧
void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;//静态变量，存储在内存的全局区。 用来记录关键帧的序号，初始化时为0
    is_keyframe_ = true;//设置该帧的is_keyframe_变量为真
    keyframe_id_ = keyframe_factory_id++;//添加一个关键帧进来，序号加1,并给该帧的keyframe_id赋值
}

}
