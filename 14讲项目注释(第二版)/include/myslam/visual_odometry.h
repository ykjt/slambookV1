#pragma once  //c++防止重复导入
#ifndef MYSLAM_VISUAL_ODOMETRY_H //使用预处理指令防止重复导入 头文件
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/frontend.h"
#include "myslam/viewer.h"

namespace myslam {

/**
 * VO 对外接口
 */
class VisualOdometry {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; //Eigen 字节对齐

    //https://blog.csdn.net/yang9649/article/details/53503729
    typedef std::shared_ptr<VisualOdometry> Ptr;//以往我也经常用typedef，但是从来没有在类里面用过。今天算是学了一招了。C++引入“仅在类内部起作用的类型别名”的初衷应该不难理解：通过限制该类型别名的作用域来防止冲突。


    /// constructor with config file
    VisualOdometry(std::string &config_path);//传入配置文件路径，构造这个视觉里程计的类

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();

    /// 获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

   private:
    bool inited_ = false;
    std::string config_file_path_;  //配置文件路径，初始化时，传入的参数对这个成员变量赋值

    Frontend::Ptr frontend_ = nullptr;//前端
    Backend::Ptr backend_ = nullptr;//后端
    Map::Ptr map_ = nullptr;//地图
    Viewer::Ptr viewer_ = nullptr;//可视化

    // dataset
    Dataset::Ptr dataset_ = nullptr;//数据集
};
}  // namespace myslam

#endif  // MYSLAM_VISUAL_ODOMETRY_H
