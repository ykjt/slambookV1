#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

/**
 * 配置类，使用SetParameterFile确定配置文件
 * 然后用Get得到对应值
 * 单例模式
 */
class Config {
   private:
    static std::shared_ptr<Config> config_; //静态的Config类型的指针
    cv::FileStorage file_;

    Config() {}  // private constructor makes a singleton
   public:
    ~Config();  // close the file when deconstructing

    // set a new config file
    static bool SetParameterFile(const std::string &filename); //静态的函数 在声明出说明

    // access the parameter values
    template <typename T>  //T 模板参数类型
    static T Get(const std::string &key) {//静态的函数.   传入一个字符串 返回配置文件中对应的参数
        return T(Config::config_->file_[key]); //T（）  根据模板参数类型进行 类型强制转换
    }
};
}  // namespace myslam

#endif  // MYSLAM_CONFIG_H
