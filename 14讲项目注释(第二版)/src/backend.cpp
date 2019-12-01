//
// Created by gaoxiang on 19-5-2.
//

#include "myslam/backend.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"

namespace myslam {

    //后端初始化
Backend::Backend() {
    backend_running_.store(true);//设置后端运行状态 为true           store() 储存
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this)); //开一个线程，并该该线程的入口函数设置为BackendLoop（）
}

void Backend::UpdateMap() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop() {
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}
//后端优化线程的  入口函数
void Backend::BackendLoop() {
    while (backend_running_.load()) {        //load()读取  backend_running_状态
        std::unique_lock<std::mutex> lock(data_mutex_);

        //https://blog.csdn.net/liyazhen2011/article/details/88603161
        map_update_.wait(lock);//wait() 阻塞程序，等待条件的达成。notify_one() 把被阻塞在wait()的线程唤醒。
                                //函数只有一个参数，程序执行到wait()语句时阻塞（相当于第二参数表示的函数始终返回false），等待其它线程调用nofify_one()将其唤醒后，再继续执行（相当于第二参数表示的函数始终返回true）。

        /// 后端仅优化激活的Frames和Landmarks
        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();//从地图中取出激活的关键帧
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();//从地图中取出激活的地图点
        Optimize(active_kfs, active_landmarks);
    }
}
//将激活的关键帧和路标点传入，进行优化
void Backend::Optimize(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks) {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

///添加位姿顶点
    // pose 顶点，使用Keyframe id
    std::map<unsigned long, VertexPose *> vertices;//此处存储添加进优化器的位姿顶点是哪些个。为了下面好给边设置连接点
    unsigned long max_kf_id = 0;//此处记录添加位姿顶点时最大的 id，  在下面给路标顶点设置id的时候使用，防止 添加位姿顶点和路标顶点的id冲突
    for (auto &keyframe : keyframes) {//遍历激活的关键帧
        auto kf = keyframe.second;//取出该帧
        //VertexPose:自定义的顶点类型
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose   定义一个相机位姿顶点
        vertex_pose->setId(kf->keyframe_id_);//设置顶点的id，此处的setid是根据该帧的keyframe_id设定的
        vertex_pose->setEstimate(kf->Pose());//设置位姿顶点的测量值
        optimizer.addVertex(vertex_pose);//将改定点添加进优化器中

        if (kf->keyframe_id_ > max_kf_id) {
            max_kf_id = kf->keyframe_id_;//记录添加位姿顶点时最大的 id
        }

        vertices.insert({kf->keyframe_id_, vertex_pose});//将该顶点和对应的keyframe_id ，保存到vertices中
    }

    // 路标顶点，使用路标id索引
    std::map<unsigned long, VertexXYZ *> vertices_landmarks;//用来存储加入到优化器中的路标顶点。方便下面判断是否重复加入优化器

    // K 和左右外参  构造边的时候用到了
    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // edges
    int index = 1;//记录边的id
    double chi2_th = 5.991;  // robust kernel 阈值
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

    for (auto &landmark : landmarks) {//遍历激活的路标点
        if (landmark.second->is_outlier_) continue;//如果该点是外点，则跳过
        unsigned long landmark_id = landmark.second->id_;//取出该点的 id
        auto observations = landmark.second->GetObs();//得到该点的 观测（哪些feature可以看到该路标点）
        for (auto &obs : observations) {//遍历这些观测的  feature
            if (obs.lock() == nullptr) continue;//如果该观测 为空，则跳过
            auto feat = obs.lock(); //因为这个观测的feature是个std::weak_ptr<Feature>，它不具有普通指针的行为，没有重载operator*和->,。所以使用lock()取出被观测的shared_ptr
            if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

            auto frame = feat->frame_.lock();//通过该观测feature，取出feature对应的帧。（从而可以将该帧的pose 和 该地图点 建立边的关系）
            //EdgeProjection： 自定义的边类型
            EdgeProjection *edge = nullptr;
            if (feat->is_on_left_image_) {//检测该feature是在左还是右图的，分别初始化不同的 edge
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            // 如果landmark还没有被加入优化，则新加一个顶点
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end()) {//如果没有查找到，则等于末尾的迭代器
                ///添加路边点 顶点  进优化器
                //VertexXYZ ： 自定义的顶点类型
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());//将该点的世界坐标 设置到 该顶点的 观测
                v->setId(landmark_id + max_kf_id + 1); //设置该路标顶点的id，在 位姿顶点的id的基础上加
                v->setMarginalized(true);//是否采用边缘化

                vertices_landmarks.insert({landmark_id, v});//记录该路标点 已经添加到 优化器中了

                optimizer.addVertex(v);//将该路标点 添加到优化器
            }
            //此处，已经找到了
            edge->setId(index);//设置边的id
            //vertices.at(frame->keyframe_id_)，在vertices中根据frame的keyframe_id这个索引，找到该 位姿顶点。 因为位姿顶点setid和添加进vertices的时候，都是使用他的keyframe_id作为键
            edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose  将该边与 这个位姿顶点相连
            edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark  将该边与 这个路标点相连
            edge->setMeasurement(toVec2(feat->position_.pt));
            edge->setInformation(Mat22::Identity());

            auto rk = new g2o::RobustKernelHuber();//定义一个核函数
            rk->setDelta(chi2_th);//设置核函数的阈值
            edge->setRobustKernel(rk);//为该条 边 设置核函数
            edges_and_features.insert({edge, feat});

            optimizer.addEdge(edge);//将该边添加进优化器

            index++;//边的id
        }
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    ///下面关于外点的处理还没看
    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while (iteration < 5) {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshold  确定是否要调整异常值阈值
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_th) {
            ef.second->is_outlier_ = true;
            // remove the observation
            ef.second->map_point_.lock()->RemoveObservation(ef.second);
        } else {
            ef.second->is_outlier_ = false;
        }
    }

    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
              << cnt_inlier;

    // Set pose and lanrmark position

    //更新优化后的值
    for (auto &v : vertices) {
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
}

}  // namespace myslam