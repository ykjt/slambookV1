//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"

namespace myslam {
//构造函数
VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

//初始化视觉里程计
bool VisualOdometry::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) { //SetParameterFile静态的，直接使用类名调用..Config是单例设计模式
        return false;
    }

    dataset_ = // Dataset::Ptr dataset_ ;
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir"))); //从配置文件中根据传入的参数读取对应值，再将该值传入Dataset的构造函数的参数列表，返回一个Dataset类型的对象，并赋值给这个智能指针
    CHECK_EQ(dataset_->Init(), true); //CHECK_XX宏 glog内容     此处初始化数据集对象

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend); ///初始化时，开一条线程
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);///开了一条线程

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run() {
    while (1) {
        LOG(INFO) << "VO is running";
        if (Step() == false) {   //循环执行step（）函数
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();//从数据集中读取一帧图片，然后构造为一个frame返回
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();

    bool success = frontend_->AddFrame(new_frame); //在前端中添加一帧

    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

}  // namespace myslam
