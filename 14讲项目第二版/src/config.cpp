#include "myslam/config.h"

namespace myslam {

///单例模式
bool Config::SetParameterFile(const std::string &filename) {
    if (config_ == nullptr)//静态的Config类型的指针为空，表示第一次，应当创建该类并让这个指针指向它
        config_ = std::shared_ptr<Config>(new Config);
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);//打开文件，并将该；类赋值给congif_指针指向的Config对象的file_变量
    if (config_->file_.isOpened() == false) {
        //C++ 中 Log()函数，能够自动根据时间记录日志信息，要求不定参数类型和参数个数。
        LOG(ERROR) << "parameter file " << filename << " does not exist.";
        config_->file_.release();//将file_对象释放掉，防止内存泄露
        return false;
    }
    return true;
}

Config::~Config() {
    //析构函数，如果该对象被销毁时，先查看该对象的成员对象是否销毁，没有销毁的话，就销毁。防止内存泄漏
    if (file_.isOpened())
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;//静态成员变量必须初始化

}
