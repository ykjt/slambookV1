//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam {

Frontend::Frontend() {
    /*maxCorners 最大角点数目
    qualityLevel角点可以接受的最小特征值,一般0.1或者0.01,不超过1
    minDistance 加点之间的最小距离
    blockSize倒数自相关矩阵的邻域范围
    useHarrisDetector 是否使用角点检测
    khessian自相关矩阵的相对权重系数 一般为0.04
    static Ptr<GFTTDetector> create( int maxCorners=1000, double qualityLevel=0.01, double minDistance=1, int blockSize=3, bool useHarrisDetector=false, double k=0.04 );
     * */
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING://初始化
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();//跟踪
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;//将当前帧赋值给上一帧。    接着读取下一帧
    return true;
}
//跟踪
bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());//估计当前帧pose初值
    }

    int num_track_last = TrackLastFrame();//光流法 跟踪上一帧
    tracking_inliers_ = EstimateCurrentPose();//估计当前帧的位姿，并返回跟踪上的feature的  内点数

    //此处根据 跟踪上的内点数，更新跟踪的状态
    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        status_ = FrontendStatus::LOST;
    }
    //判断是否将该帧加入关键帧中
    InsertKeyframe();

    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    //可视化view中加入该帧
    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool Frontend::InsertKeyframe() {
    //判断是否将该帧加入关键帧中
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {//如果跟踪上的内点数大于这个值（80），说明这两帧之间很近，不必加入关键帧了
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();//将该帧设置为关键帧
    map_->InsertKeyFrame(current_frame_);//在地图中加入该关键帧

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    //因为这是新进来的一帧，然后对这一帧中的feature对应的地图点添加观测
    SetObservationsForKeyFrame();

    //在当前帧的左图中重新提取特征点
    DetectFeatures();  // detect new features
    // track in right image
    //使用光流法，在右图中跟踪左图中的特征点
    FindFeaturesInRight();

    // triangulate map points
    //三角化地图点
    TriangulateNewPoints();

    // update backend because we have a new keyframe
    //更新后端，因为我们有一个新的关键帧（当前帧加入到了关键帧中了嘛）
    backend_->UpdateMap();//通知后端去优化吧

    //可视化部分 更新可视化界面
    if (viewer_) viewer_->UpdateMap();

    return true;
}
//添加观测
void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_left_) {//遍历当前帧左图中的feature
        auto mp = feat->map_point_.lock();//取出这个feature对应的地图点
        if (mp) mp->AddObservation(feat);//给该地图点添加观测   观测到了这个帧的这个feature
    }
}


//三角化地图点
int Frontend::TriangulateNewPoints() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};//取出 左右相机的pose()  //visual_odometry中已经从配置文件中读出数据

    SE3 current_pose_Twc = current_frame_->Pose().inverse();   //current_frame_->Pose() 是  Tcw  （此处的作用：三角化出来的点是在相机坐标下的，要转到世界坐标系下就要乘以这个Twc）
    int cnt_triangulated_pts = 0;//记录三角化的点数
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {//遍历当前帧左图中的feature
        //weak_ptr指针也可以间接操纵shared_ptr指针：
        //1）lock() ，weak_ptr指针调用lock()方法会获得shared_ptr的返回值。通过这个shared_ptr来进行操作
        //2）expired()，用来探测shared_ptr的有效性。shared_ptr一旦被释放，指针就会被置为null。
        //原文链接：https://blog.csdn.net/tsh123321/article/details/88966550
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left_->pixel2camera(//传入图像中的像素坐标，返回相机坐标系下归一化平面的三维点
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(//传入图像中的像素坐标，返回相机坐标系下归一化平面的三维点
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();
            //传入左右相机的位姿，要三角化点的相机坐标系下归一化平面的三维点，以及接收三维点
            if (triangulation(poses, points, pworld) && pworld[2] > 0) {//三角化，并且深度值大于0
                auto new_map_point = MapPoint::CreateNewMappoint();//这里生成了一个空的地图点（只有id有值），下面就给这个空的地图点的各个属性赋值

                pworld = current_pose_Twc * pworld;//上面调用函数生成的三维点是在相机坐标系下的三维点，和真实的世界坐标系下的三维点还差一个平移旋转（此处转到世界坐标系下）
                new_map_point->SetPos(pworld);//设置该地图点的世界坐标
                //下面给地图点添加 是由哪个feature观测到的
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);//设置该点的观测
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);//因为这个地方，这一帧的左右图像都观测到了，都添加

                //下面添加 该帧中 特征点 观测到的 地图点
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;

                // 将该地图点添加进地图中
                map_->InsertMapPoint(new_map_point);//在visual_odometry的init中，就给map_这个指针赋值了的。
                cnt_triangulated_pts++;//三角化出的点的数量加一
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

//在这里，已经知道了当前帧中的feature对应的地图点了，然后一开始也对当前帧有一个初始的估计的位姿。所以在这里可以通过优化得到更加准确的当前帧的位姿
int Frontend::EstimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp) {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}
//跟踪上一帧。这个函数中，通过使用 光流法 将上一帧左图中的feature 在当前帧中的左图中 跟踪到。并且给当前帧中feature对应的地图点 赋值 为 上一帧同一个feature对应的地图点
int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image   使用光流法 估计右图中的点
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_) {//遍历上一帧中左图中的 feature
        if (kp->map_point_.lock()) {//拿到该feature对应的地图点
            // use project point
            auto mp = kp->map_point_.lock();//拿到该feature对应的地图点
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());//并根据当前帧的pose初值，估计出地图点在当前帧中的像素坐标
            kps_last.push_back(kp->position_.pt);//将上一帧中的feature的2D位置（像素坐标），存在kps_last中
            kps_current.push_back(cv::Point2f(px[0], px[1]));//将上一帧中feature在当前帧中的估计位置 存起来
        } else {//如果该feature没有对应的地图点
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);//直接将该点在上一帧中的位置，放在下一帧的估计位置吧
        }
    }

    std::vector<uchar> status;// 0   1  记录跟踪的状态。跟踪上了为1,否则为0
    Mat error;
    //使用上一帧的左图和当前帧的左图进行 光流跟踪
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;//记录跟踪上的点的个数

    for (size_t i = 0; i < status.size(); ++i) {//遍历所有点的跟踪情况
        if (status[i]) {//如果该点跟踪上了
            cv::KeyPoint kp(kps_current[i], 7);//将该点包装成KeyPoint
            Feature::Ptr feature(new Feature(current_frame_, kp));//将该KeyPoint包装成一个Feature，并将该Feature的地址赋值给这个指针
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;//因为该点是在上一帧左侧图像中跟踪到的，所以观察到的是同一个地图点，这里直接将上一帧的这个feature对应的地图点 赋值给 当前帧feature的的地图点
///            feature->is_on_left_image_ = true;//设置该点是在左侧图像中生成的。因为is_on_left_image_默认值就是true，所以此处不用写这句代码
            current_frame_->features_left_.push_back(feature);//将该点 存入当前帧左侧图像的feature中
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}
//双目初始化，第一帧的时候才回来到这里
bool Frontend::StereoInit() {
    int num_features_left = DetectFeatures();//提取特征点
    int num_coor_features = FindFeaturesInRight();//使用光流法，在右侧图像中跟踪左侧图像提取到的特征点
    if (num_coor_features < num_features_init_) {//判断右侧图像跟踪上的点的个数是否大于100
        return false;
    }

    //初始化地图（生成地图点）
    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;//地图初始化成功，则更新前端跟踪的状态

#warning 此处的可视化还没看
        if (viewer_) {//viewer_ 在visual_odometry的init中就已经创建好了。之后赋值给了这里
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}
//提取特征点
int Frontend::DetectFeatures() {
    ///这个mask怎么处理的还没看
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }
    //提取左图像的特征点
    std::vector<cv::KeyPoint> keypoints;
    //对左侧图像  提取 gftt 特征点
    gftt_->detect(current_frame_->left_img_, keypoints, mask);//仅仅是检测出在二维图像中的位置，并没有提取描述子
    int cnt_detected = 0;//记录特征点个数
    //遍历特征点
    for (auto &kp : keypoints) {
        /*
         * 先把特征点包装成一个feature，然后返回内存地址。再赋值给这个Feature类型的智能指针，再push到当前帧的features_left_容器中
         * */
        current_frame_->features_left_.push_back(Feature::Ptr(new Feature(current_frame_, kp)));

        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}
//使用LK流估计右图像中的点
int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_) {//将左侧图像中提取到的，要进行跟踪的点放到 容器中
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();//weak_ptr.lock()返回一个shared_ptr,如果返回的shared_ptr被使用,引用计数增加
        if (mp) {
            // use projected points as initial guess 使用投影点作为初始猜测
            auto px =
                camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left image 使用与左图中相同的点
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;// 0   1  记录跟踪的状态。跟踪上了为1,否则为0
    Mat error;

    //调用opencv的函数进行光流跟踪.使用迭代Lucas-Kanade方法计算稀疏特征集的光流金字塔。

    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_,//第一个8位输入图像或金字塔建立光流金字塔。
        current_frame_->right_img_, //第二个输入图像或金字塔的大小和类型与第一个相同。
        kps_left,//vector of 2D points for which the flow needs to be found;
        kps_right,//output vector of 2D points containing the calculated new positions of input features in the second image;
        status,//output status vector (of unsigned chars); each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
        error,//output vector of errors; each element of the vector is set to an error for the  corresponding feature, type of the error measure can be set in flags parameter; if the flow wasn't  found then the error is not defined (use the status parameter to find such cases).
        cv::Size(11, 11),//size of the search window at each pyramid level.
        3,//构造金字塔的层。0-最大金字塔等级数；如果设置为0，则不使用金字塔（单）级别），如果设置为1，则使用两个级别，依此类推；如果将金字塔传递给输入，则 算法将使用与金字塔相同的级别，但不超过maxLevel。
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW
        );

    int num_good_pts = 0;//记录跟踪上的点的个数
    for (size_t i = 0; i < status.size(); ++i) {//遍历所有点的跟踪情况
        if (status[i]) {//如果该点跟踪上了
            cv::KeyPoint kp(kps_right[i], 7);//将该点包装成KeyPoint
            Feature::Ptr feat(new Feature(current_frame_, kp));//将该KeyPoint包装成一个Feature，并将该Feature的地址赋值给这个指针
            feat->is_on_left_image_ = false;//设置该点是在右侧图像中生成的
            current_frame_->features_right_.push_back(feat);//将该点 存入当前帧右侧图像的feature中
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);//如果没有跟踪上，将该存储该feature的位置的指针设置为空指针
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}
//初始化地图
//1.生成地图点，往地图中添加地图点
//2.设置关键帧，将关键帧添加到地图中
//3.触发后端优化线程进行优化
bool Frontend::BuildInitMap() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};//拿到2个相机的pose
    size_t cnt_init_landmarks = 0;//记录生成 地图点的数量

//1.处理这一帧中的点

    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {//遍历左侧图像中提取到的特征点
        if (current_frame_->features_right_[i] == nullptr) continue;//判断左侧的点在右侧图像中是否也检测到了，如果不是则continue
        // create map point from triangulation 三角化地图点
        std::vector<Vec3> points{
            //将当前循环中，左侧图像中提取到的二维点变为三维点，深度值统一先设置为1   （归一化平面的3D 点）
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),

            //将当前循环中，右侧图像中提取到的二维点变为三维点，深度值统一先设置为1 （归一化平面的3D 点）
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};

        Vec3 pworld = Vec3::Zero();//定义一个世界坐标系下的三维点
        //调用函数，三角化出世界坐标系下的三维点
        if (triangulation(poses, points, pworld) && pworld[2] > 0) {//三角化成功，并且三角化出来的点的深度为正
            auto new_map_point = MapPoint::CreateNewMappoint();//这里生成了一个空的地图点（只有id有值），下面就给这个空的地图点的各个属性赋值
            new_map_point->SetPos(pworld);//设置该地图点的世界坐标 （因为这是第一帧三角化，所以三角化出来的点虽然是在第一帧的相机坐标系下，但是我们以第一帧的相机坐标下为世界坐标系，所以此处直接设置即可，如果不是在第一帧中，那这里就要将相机坐标系下的点转到世界坐标系下再设置）
            //下面给地图点添加 是由哪个feature观测到的
            new_map_point->AddObservation(current_frame_->features_left_[i]);//设置该点的观测
            new_map_point->AddObservation(current_frame_->features_right_[i]);//因为这个地方，这一帧的左右图像都观测到了，都添加
            //下面添加 该帧中 特征点 观测到的 地图点
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;

            cnt_init_landmarks++;//地图点的数量加一
            // 将该地图点添加进地图中
            map_->InsertMapPoint(new_map_point);//在visual_odometry的init中，就给map_这个指针赋值了的。
        }
    }
//2.这一帧的点，在上面处理完了，再来处理这一帧

    current_frame_->SetKeyFrame();//将当前帧设置为关键帧
    map_->InsertKeyFrame(current_frame_);//往地图中添加一个关键帧
//3.后端优化
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

bool Frontend::Reset() {
    //还没实现
    LOG(INFO) << "Reset is not implemented. ";
    return true;
}

}  // namespace myslam