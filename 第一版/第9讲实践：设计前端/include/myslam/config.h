#ifndef CONFIG_H  //条件编译指令，防止重复引入
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    static std::shared_ptr<Config> config_;  //静态成员变量，存在内存中的全局区，只有一份;这个成员变量是一个指向Config类对象智能指针
    cv::FileStorage file_;//一个FileStorage类型的成员变量
    
    Config () {} // private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing 
    
    // set a new config file 
    static void setParameterFile( const std::string& filename ); //静态成员函数，直接用类名即可调用，不必创建对象来调用
    
    // access the parameter values，通过模板函数，获得yaml的配置文件中的任意类型的参数值

    //通过向模板参数传入 T，返回T类型的值 。为什么用模板，因为配置文件里面的参数有各种类型的，有字符串、int double等
    template< typename T >
    static T get( const std::string& key )//静态成员函数，直接用类名即可调用，不必创建对象来调用
    {
        return T( Config::config_->file_[key] );
    }  //之后可以先创建一个yaml文件，利用setParameterFile读取，然后用get来获得文件中的参数
};
}

#endif // CONFIG_H