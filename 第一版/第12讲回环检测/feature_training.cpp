#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

/***************************************************
 * 本节演示了如何根据data/目录下的十张图训练字典
 * #include "DBoW3/DBoW3.h"  要把DBow3这个c++开源的库安装上  https://github.com/resalinas/DBow3 .或者在书中代码提供的包中安装(cmake安装)
 * ************************************************/

int main( int argc, char** argv )
{
    // read the image 
    cout<<"reading images... "<<endl;
    //这里就是十张图片，所以处理的时候是直接读到这个变量里面。
    //在gen_vocab_large.cpp 中，是读入文件名，在构建描述子的时候才读入，因为这个是构建比较大的词袋模型，读入的图片多，消耗内存庞大
    vector<Mat> images; 
    for ( int i=0; i<10; i++ )
    {
        string path = "./data/"+to_string(i+1)+".png";
        images.push_back( imread(path) );
    }

    // detect ORB features
    cout<<"detecting ORB features ... "<<endl;
    //调用接口。这里是构建的ORB的词袋模型。也可以构建其他特征类型的词袋，比如brisk,surf等
    //dbow3支持4种描述类型：orb brisk akaze(opencv3) surf
    //所以，如果使用别人训练好的字典，请注意使用的特征类型是否一致
    Ptr< Feature2D > detector = ORB::create();
    vector<Mat> descriptors;
    for ( Mat& image:images )
    {
        vector<KeyPoint> keypoints; 
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor );
    }
    //创建字典了
    // create vocabulary 
    cout<<"creating vocabulary ... "<<endl;
    DBoW3::Vocabulary vocab;//创建一个对象
    vocab.create( descriptors );//传入描述子
    cout<<"vocabulary info: "<<vocab<<endl;
    vocab.save( "vocabulary.yml.gz" );//保存到文件
    cout<<"done"<<endl;
    
    return 0;
}