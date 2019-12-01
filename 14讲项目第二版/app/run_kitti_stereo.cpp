//
// Created by gaoxiang on 19-5-4.
//
//添加一个测试

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"
//用Google的gflags优雅的解析命令行参数
//使用简介： https://blog.csdn.net/lanmolei814/article/details/78478124
DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));//创建一个VisualOdometry 对象，并使用VisualOdometry类型的指针指向这个对象

    assert(vo->Init() == true);//assert宏的原型定义在<assert.h>中，其作用是如果它的条件返回错误，则终止程序执行。

    vo->Run();

    return 0;
}
