#include "myslam/config.h"

namespace myslam 
{
    
void Config::setParameterFile( const std::string& filename )
{
    if ( config_ == nullptr )
        config_ = shared_ptr<Config>(new Config);//new一个Config对象，然后这个智能指针指向这个对象
        config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ ); //可以读取一个yaml文件，且访问其中任意一个字段 //初始化一个FileStorage对象，并赋值给Config对象的成员函数
    if ( config_->file_.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();//释放掉file_指向的内存空间
        return;
    }
}

Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();//释放掉file_指向的内存空间
}


//静态成员变量必须初始化
shared_ptr<Config> Config::config_ = nullptr;

}