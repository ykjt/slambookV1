#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"

namespace myslam
{

VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{//map_ ( new Map ):  因为VisualOdometry在系统中只有一个，所以这个构造函数也只会调用一次，所以这个VisualOdometry的map_也就只有一个
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

VisualOdometry::~VisualOdometry()
{

}

bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:
    {
        state_ = OK; //进入了这里说明  第一帧图片来了  设置状态为ok
        curr_ = ref_ = frame; //当前帧和参考关键帧都设置为第一帧
        // extract features from first frame and add them into map
        extractKeyPoints();//提取关键点
        computeDescriptors();//计算描述子
        addKeyFrame();      // the first frame is a key-frame 第一帧添加为  关键帧
        break;
    }
    case OK:
    {
        curr_ = frame;//将刚读进来的这一张图片，赋值给当前帧
        //将参考关键帧的位姿赋值给当前帧
        //注意此处，所以暂时来看当前帧的位姿是参考关键帧的，并非是实际的，因为都还没计算当前的位姿哪里来的实际位姿）
        //目的： 在下面，利用这个位姿来判断地图中的地图点在不在参考关键帧中，如果在的话，就可以用来与当前帧的特征点做特征点匹配
        curr_->T_c_w_ = ref_->T_c_w_;
        extractKeyPoints();//提取关键点
        computeDescriptors();//计算描述子
        featureMatching();//特征点匹配
        poseEstimationPnP();//计算Tcw,并优化
        if ( checkEstimatedPose() == true ) // a good estimation 判断是否是 好的位姿
        {
            curr_->T_c_w_ = T_c_w_estimated_;//这里更新当前帧的位姿，为优化后的位姿
            optimizeMap();  //优化地图
            num_lost_ = 0;//当前帧跟踪上了，设置num_lost_为0,这个参数是记录 连续跟踪丢失 帧的次数
            if ( checkKeyFrame() == true ) // 调用checkKeyFrame判断是否算是关键帧，如果是就插入
            {
                addKeyFrame();
            }
        }
        else // bad estimation due to various reasons 估计的位姿不好  太多原因了。。。
        {
            num_lost_++;//记录跟踪丢失次数
            cout<<"跟丢"<<num_lost_<<"次!"<<endl;
            if ( num_lost_ > max_num_lost_ )  //判断当前丢失次数 是否超过 最大连续丢失次数（配置文件中设定的）
            {
                //超过了，就跟踪丢失
                state_ = LOST;
                cout<<"状态异常！"<<"当前丢失次数为"<<num_lost_<<endl;
            }
            //，没有超过就接着搞下一帧
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}
//提取关键点
void VisualOdometry::extractKeyPoints()
{
    boost::timer timer;
    orb_->detect ( curr_->color_, keypoints_curr_ );//keypoints_curr_此处只是检测出特帧点在图像中的二维坐标位置，不是带有深度的点
    cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}
//计算描述子
void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
}

void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    // select the candidates in map 
    //建立一个目标图，承接匹配需要地图点的描述子，因为匹配是需要的参数是描述子
    Mat desp_map;
    //建立一个候选地图点数组，承接匹配需要的地图点
    vector<MapPoint::Ptr> candidate;

    //检查地图点是否为匹配需要的，逻辑就是遍历维护的局部地图中所有地图点，
    //然后利用isInFrame()函数检查有哪些地图点在当前观察帧中，
    //如果在则把地图点push进candidate中，描述子push进desp_map中
    for ( auto& allpoints: map_->map_points_ )//遍历地图中所有的地图点
    {
        //总之功能上就是把地图点取出，赋值给p
        MapPoint::Ptr& p = allpoints.second;//取出当前遍历的这个地图点
        // check if p in curr frame image
        //我的理解：因为curr_此时的T_c_w_值是参考关键帧的T_c_w_，所以我认为说判断p点是否在参考关键帧更加准确
        //如果遍历地图点中的点，该点在参考关键帧中，那么就可以作为候选的匹配点 和当前帧中的特征点做匹配
        if ( curr_->isInFrame(p->pos_) ) //传入地图点在世界坐标系点的位置
        {
            // add to candidate 
            p->visible_times_++;//该点被观测到的次数加1
            candidate.push_back( p );//该点添加到候选点 容器中
            desp_map.push_back( p->descriptor_ ); //该点的描述子取出来，添加到候选匹配时用到的描述子的容器中
        }
    }
    //描述子匹配。匹配的结果存储在matches中
    //这里是用的是  地图中 3D 点的描述子 与 当前帧中2d点的描述子做 匹配
    matcher_flann_.match ( desp_map, descriptors_curr_, matches );
    // select the best matches  找出匹配对中，最小的距离
    float min_dis = std::min_element (
                        matches.begin(),
                        matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 ){
                             return m1.distance < m2.distance;
                        }
                    )->distance;

    //根据最小距离，对matches数组进行刷选，只有小于最小距离一定倍率或者小于30的才能push_back进数组。
    //最终得到筛选过的，距离控制在一定范围内的可靠匹配
    match_3dpts_.clear();//先清空容器，再记录当前匹配中 ，匹配上的3d点
    match_2dkp_index_.clear();//先清空容器，再记录当前匹配中 ，匹配上的2d点
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            //这里变化是不像之前直接将好的m  push进feature_matches_就完了。 
            //这里感觉像做一个记录，candidate中存的是观察到的地图点 
            // 进一步，在candidate中选出好的匹配的点，push进match_3dpts_， 
            //这个match_3dpts_代表当前这一帧计算T时所利用到的所有好的地图点，放入其中。 
            //由此可见，candidate只是个中间存储，新的一帧过来会被刷新。 
            //同样match_2dkp_index_也是同样的道理，只不过是记录的当前帧detect出来的keypoint数组中的点的索引
            match_3dpts_.push_back( candidate[m.queryIdx] );//从candidate中根据 索引取出3d点存储进来
            match_2dkp_index_.push_back( m.trainIdx ); //match_2dkp_index_ 存储的匹配上的2d的索引
        }
    }
    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time: "<<timer.elapsed() <<endl;
}

void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    //从这里就可以看出，参考帧用的是3D(在参考帧视野范围内的地图点)，当前帧用的2D
    for ( int index:match_2dkp_index_ )
    {
        pts2d.push_back ( keypoints_curr_[index].pt );
    }
    for ( MapPoint::Ptr pt:match_3dpts_ )
    {
        pts3d.push_back( pt->getPositionCV() );//返回3d点位置，由Vector3d类型变为Point3f类型
    }
    //构造相机内参矩阵
    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
              ref_->camera_->fx_, 0, ref_->camera_->cx_,
              0, ref_->camera_->fy_, ref_->camera_->cy_,
              0,0,1
            );
    Mat rvec, tvec, inliers;
    /*
     * pts3d:  这里传入的是世界坐标下的地图点 3D点
     * pts2d:
     * K:相机内参
     * Mat()：相机畸变参=参数矩阵
     * rvec：输出旋转向量 （从模型坐标系到相机坐标系）
     * tvec：输出平移向量
     * false：
     * 100：RANSAC 迭代次数
     * 4.0：观察到和计算点投影之间的最大允许距离来考虑它
     * 0.99：置信度算法产生有用结果的概率。
     * inliers： inliners输出向量，包含objectPoints和imagePoints中的inliner索引
     * */
    cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;//内点个数
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_w_estimated_ = SE3 (
                           SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
                           Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
                       );

    // using bundle adjustment to optimize the pose
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block; // 每个误差项优化变量维度为6（位姿），误差值维度为2 （重投影误差 (u,v)）
    // 第1步：创建一个线性求解器LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
    Block* solver_ptr = new Block ( linearSolver );
    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    // 第4步：创建终极大boss 稀疏优化器（SparseOptimizer）
    g2o::SparseOptimizer optimizer;// 图模型
    optimizer.setAlgorithm ( solver );  // 设置求解器
//    optimizer.setVerbose( true );       // 打开调试输出

    // 第5步：定义图的顶点和边。并添加到SparseOptimizer中
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();//创建一个位姿节点
    pose->setId ( 0 );//设置id
    pose->setEstimate ( g2o::SE3Quat (
        T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
    ));//设置估计值
    optimizer.addVertex ( pose );//添加该 位姿节点

    // edges 添加边
    for ( int i=0; i<inliers.rows; i++ ) //内点才添加进去
    {
        //真正用于匹配的特征点的索引会存放在inliers中，可以通过inliers.at<int> ( i,0 )得到
        int index = inliers.at<int> ( i,0 );
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();//创建一个EdgeProjectXYZ2UVPoseOnly类型的边
        edge->setId ( i );//设置这个边的id
        edge->setVertex ( 0, pose );//设置这条边连接的顶点
        edge->camera_ = curr_->camera_.get();//把这个类传进去，可以获得内参等
        edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );//把观测值3d点 赋值进去
        edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );//设置测量值  函数来定义观测值?
        edge->setInformation ( Eigen::Matrix2d::Identity() ); //来定义协方差矩阵的逆
        optimizer.addEdge ( edge );//添加边到优化器
        // set the inlier map points 
        //记录匹配次数，作用是如果该点的次数太少的话，之后就剔除掉了
        match_3dpts_[index]->matched_times_++;
    }
    // 第6步：设置优化参数，开始执行优化
    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );//设置迭代次数，然后就开始执行图优化了。

    T_c_w_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );
    
    cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
}
//判断估计的位姿好不好
bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )//判断求解出来的位姿内的  内点数是否小于 配置文件中设定的最少内点个数
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    //如果运动过大，也很有可能是错误的位姿
    //计算当前帧 到 参考关键帧的 T    T_r_c
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();//将SE3 转换为 6维向量
    if ( d.norm() > 5.0 )//这里的是向量中 每个元素的平方和再开平方根
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}
//添加关键帧，第一帧设置为关键帧
void VisualOdometry::addKeyFrame()
{
    //判断是否为第一帧
    if ( map_->keyframes_.empty() )
    {
        // first key-frame, add all 3d points into map
        //如果是第一帧，插入所有的3d点
        cout<<"#############################################################3######"<<endl;
        cout<<"ref_->id_"<<ref_->id_<<"cur_->id_"<<curr_->id_<<endl;
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            double d = curr_->findDepth ( keypoints_curr_[i] );//调用Frame的方法，传入像素坐标，得到深度值
            if ( d < 0 ) 
                continue;
            //将该点由像素坐标 转换为 世界坐标系下的 三维点
            Vector3d p_world = ref_->camera_->pixel2world (//因为是第一帧，在这之前已经ref_ = curr_;
                Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
            );
            //该点在世界坐标系下的三维点  减去 其参考关键帧的原点 ，得到一个由参考关键帧看到该点的 方向向量  （为了记录观测方向）
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();//将其归一化 得到 观测方向向量

            //上方求出构造地图点所需所有参数：该点的 3D点、观测方向、描述子、哪一帧，然后构造一个地图点
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            );//这个 n 是参考关键帧看到该 点 的方向向量，并且归一化了。在MapPoint类里面这个n赋值给了该点的norm_成员变量
            //将该点添加到地图
            map_->insertMapPoint( map_point );
        }
    }
    
    map_->insertKeyFrame ( curr_ );  //不是第一帧的话 直接插入帧
    ref_ = curr_;//当前帧设置为参考关键帧
}

//新增函数，增加地图中的点。局部地图类似于slidewindow一样，随时的增删地图中的点，来跟随运动
void VisualOdometry::addMapPoints()
{
    //创建一个bool型的数组matched，大小为当前keypoints数组大小，值全为false
    vector<bool> matched(keypoints_curr_.size(), false);

    //首先这个match_2dkp_index_是新来的当前帧跟地图匹配时，好的匹配到的关键点在keypoints数组中的索引
    //在这里将已经匹配的keypoint索引置为true
    for ( int index:match_2dkp_index_ ) //match_2dkp_index_存储的匹配上的2d的索引
        matched[index] = true;

    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
        //如果为true，说明在地图中找到了匹配，也就意味着地图中已经有这个点了。直接continue
        if ( matched[i] == true )
            continue;
#warning 此处是否有bug
        //如果没有continue的话，说明这个点在地图中没有找到匹配，认定为新的点，
        //下一步就是找到depth数据，构造3D点，然后构造地图点，添加进地图即可
//        cout<<"--------------------此处打印的两个id值不同---------------------------------------"<<endl;
//        cout<<"ref_->id_"<<ref_->id_<<"cur_->id_"<<curr_->id_<<endl;
///        此处添加地图点：keypoints_curr_是保存当前帧特征点提取的特征点在图像中的像素坐标，
///        这个地方却是在参考关键帧的深度图中根据当前帧特征点的像素坐标查询深度值，
///        那岂不是对应不上？我觉得这个地方应该是d = curr_->findDepth ( keypoints_curr_[i] );
///        虽然不管是写的curr_还是ref_程序都能运行完毕
        double d = ref_->findDepth ( keypoints_curr_[i] );
//        double d2 = curr_->findDepth ( keypoints_curr_[i] );
//        cout<<"-========================下面输出的d1和d2深度值也当然不同==================================="<<endl;
//        cout<<"d = "<<d<<"-----d2 = "<<d2<<endl;
        if ( d<0 )  
            continue;
        //将该点由像素坐标 转换为 世界坐标系下的 三维点
        Vector3d p_world = ref_->camera_->pixel2world (
            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
            curr_->T_c_w_, d
        );
        //该点在世界坐标系下的三维点  减去 其参考关键帧的原点 ，得到一个由参考关键帧看到该点的 方向向量  （为了记录观测方向）
        Vector3d n = p_world - ref_->getCamCenter();
        n.normalize(); //存储对应的向量
#warning
        //上方求出构造地图点所需所有参数：该点的 3D点、观测方向、描述子、哪一帧，然后构造一个地图点
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
        );//这个 n 是参考关键帧看到该 点 的方向向量，并且归一化了。在MapPoint类里面这个n赋值给了该点的norm_成员变量
        //将该点添加到地图
        map_->insertMapPoint( map_point );
    }
}
//优化地图
void VisualOdometry::optimizeMap()
{
    // remove the hardly seen and no visible points
    //遍历地图中的所有地图点
    /*
     * 1.不在当前帧没得地图点，删掉
     * 2.即使在视野中，匹配率过低也删掉
     * 3.如果当前帧和参考关键帧 看到该点的夹角过大，则删除（说明相机跑远了）
     * */
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        //iter->second->pos_  拿到当前迭代地图点的3D点
        if ( !curr_->isInFrame(iter->second->pos_) ) //(*it).first会得到key，(*it).second会得到value。
        {//如果这个地图点不在该帧中
            iter = map_->map_points_.erase(iter);//从地图中剔除该地图点（看来这个map只是一个局部的地图，里面存存储的地图点都是要在当前帧看得到的点）
            continue;
        }
        //定义匹配率，用匹配次数/可见次数，匹配率过低说明经常见但是没有几次匹配。应该是一些比较难识别的点，也就是出来的描述子比价奇葩。所以删掉
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        //如果当前帧和参考关键帧 看到该点的夹角过大，则删除（说明相机跑远了）
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        //继续，可以根据一些其他条件自己添加要删除点的情况
        if ( iter->second->good_ == false )  
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }
    //首先当前帧去地图中匹配时，点少于100个了，适当从当前的2D中 增加地图点点。这样的话，接下来的跟踪时，匹配的3D点数目相对多一点，就更容易与接下来的2d点匹配上
    // 一般情况是运动幅度过大了，跟之前的帧没多少交集了，所以增加一下。
    if ( match_2dkp_index_.size()<100 )   //当前帧和地图的3D点匹配时， 匹配上的2d点的个数是否小于100
        addMapPoints();  
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        //如果点过多了，多于1000个，适当增加释放率，让地图处于释放点的趋势
        //(匹配率=用匹配次数/可见次数 如果匹配率小于这个map_point_erase_ratio_则从地图中删除该点)
        map_point_erase_ratio_ += 0.05;
    }
    else 
    //如果没有多于1000个，保持释放率在0.1，维持地图的体量为平衡态
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}

double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    //构造方法是空间点坐标减去相机中心坐标。得到从该帧相机中心指向指向空间点的向量
    Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    //物理意义就是 参考关键帧下看空间点和从该帧下看空间点，视角差的角度。因为下面的point_norm_存储的是参考关键帧相机中心指向该空间点的向量
    //向量*乘为：a*b=|a||b|cos<a,b> 
    //所以单位向量的*乘得到的是两个向量的余弦值，再用acos()即得到角度，返回
    return acos( n.transpose()*point->norm_ );
}


}