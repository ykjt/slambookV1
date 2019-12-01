#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include <chrono>   // for time stamp
int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {//传入执行文件的名称 和 配置文件   bin/run_vo config/default.yaml
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    //把轨迹文件清空
    string filePath = "./trace.txt";
    fstream file(filePath, ios::out);

    myslam::Config::setParameterFile ( argv[1] ); //将配置文件读取进来

    //整个系统就一个vo对象
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );//创建了一个VisualOdometry对象，创建了一个指向VisualOdometry类型的智能指针并且指向刚才创建的那个对象

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );//<string>:是传入的模板参数. dataset_dir要从配置文件读取参数的名称
    cout<<"dataset: "<<dataset_dir<<endl;//输出数据集路径
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        cout<<dataset_dir;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );  //atof 把字符串转换成浮点数
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }
    cout<<"total "<<rgb_files.size()<<" picture!"<<endl;
    myslam::Camera::Ptr camera ( new myslam::Camera ); //创建一个指向Camera类型的智能指针，并指向一个 new出来的camera对象  是相机的参数对象
    
    // visualization，创造一个可视化窗口，构造参数为窗口名称
    cv::viz::Viz3d vis("Visual Odometry");
    //创建坐标系部件，构造参数是坐标系长度，
    cv::viz::WCoordinateSystem world_coor(0.5), camera_coor(0.5);

    //           相机位置坐标、              相机焦点坐标、              相机y轴朝向
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    // 由这三个参数，用makeCameraPose()函数构造Affine3d类型的相机位姿，这里其实是视角位姿，也就是程序开始时你处在什么视角看
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    //用setViewerPose()设置观看视角
    vis.setViewerPose( cam_pose );
    
    //利用setRenderingProperty()函数设置渲染属性，
    // 第一个参数是个枚举，对应要渲染的属性这里是线宽，后面是属性值
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);

    
    //用showWidget()函数将部件添加到窗口内
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );

    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;

    // 记录系统时间
    auto start = chrono::system_clock::now();

    for ( int i=0; i<=rgb_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            {
                cout<<"图像已经读完！"<<i<<endl;
                break;
            }

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);

        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();//创建一个新的帧，即当前for循环读取到图像这一帧，即当前帧
        pFrame->camera_ = camera; //pFrame的camera_成员变量也是一个智能指针，现在将指针指向camera对象。  此处系统中的每一帧都是指向的同一个camera对象
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed()<<endl;
        
        if ( vo->state_ == myslam::VisualOdometry::LOST )
        {
            cout<<"跟丢了！"<<"当前第"<<i<<"帧！"<<endl;
            break;
        }
            
        ///这里可能命名错了，大概是想Twc=Tcw.inverse（）
        SE3 Tcw = pFrame->T_c_w_.inverse();

        //将轨迹存储在文件中
        Eigen::Quaterniond q(Tcw.rotation_matrix());
        Eigen::Vector3d t(Tcw.translation());
        string filePath = "./trace.txt";
        fstream fileOut(filePath, ofstream::app);
        fileOut <<double(timestamp.count())/1000.0<<" "<< t[0]<<" "<<t[1]<<" "<<t[2]<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
        fileOut.close();
        cout << endl << "trajectory saved!" << endl;

        // show the map and the camera pose 
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
        );
         Mat img_show = color.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }
        cv::imshow("image", img_show );
        cv::waitKey(1);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);
        cout<<"now is show "<<i<<"th picutre!"<<endl;
    }

    return 0;
}
