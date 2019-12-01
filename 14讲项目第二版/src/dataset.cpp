#include "myslam/dataset.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace myslam {
//构造函数
Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}
//初始化 从calib文件读取数据，对camera进行参数初始化
bool Dataset::Init() {
    // read camera intrinsics and extrinsics  读取相机内部和外部参数
    ifstream fin(dataset_path_ + "/calib.txt");
    if (!fin) {
        LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        char camera_name[3];
        for (int k = 0; k < 3; ++k) {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }
        Mat33 K;//相机内参
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        K = K * 0.5;

        //double fx, double fy, double cx, double cy, double baseline,const SE3 &pose
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.squaredNorm(), SE3(SO3(), t)));

        /*
         * -----------------------------------------------
            fx = 353.546---fy = 353.546---cx = 300.944---cy = 91.5552--baseline = 0
            Camera 0 extrinsics: 0 0 0
            -----------------------------------------------
            fx = 353.546---fy = 353.546---cx = 300.944---cy = 91.5552--baseline = 0.288531
            Camera 1 extrinsics: -0.537151         0         0
            -----------------------------------------------
            fx = 353.546---fy = 353.546---cx = 300.944---cy = 91.5552--baseline = 0.00376528
            Camera 2 extrinsics:   0.0610306 -0.00143972  0.00620322
            -----------------------------------------------
            fx = 353.546---fy = 353.546---cx = 300.944---cy = 91.5552--baseline = 0.225087
            Camera 3 extrinsics:  -0.474418 0.00187031  0.0033185
         *
         * */
        cameras_.push_back(new_camera);

        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }
    fin.close();
    current_image_index_ = 0;
    return true;
}

Frame::Ptr Dataset::NextFrame() {
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat image_left, image_right;
    // read images
    image_left =
        cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    image_right =
        cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);

    if (image_left.data == nullptr || image_right.data == nullptr) {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();//创建一个新的帧，赋值给new_frame . 静态函数，通过类名即可调用
    new_frame->left_img_ = image_left_resized;//给这一帧图像的左右图像 赋值
    new_frame->right_img_ = image_right_resized;
    current_image_index_++; //记录读取到哪一张图片了
    return new_frame;
}

}  // namespace myslam