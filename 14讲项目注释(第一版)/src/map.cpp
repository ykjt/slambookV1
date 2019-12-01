#include "myslam/map.h"

namespace myslam
{
//插入关键帧  要会map这种容器的操作
void Map::insertKeyFrame ( Frame::Ptr frame )
{
    cout<<"Key frame size = "<<keyframes_.size()<<endl;
    //关键帧中没有这一帧，就插入（id号应该是按大小排序的，id号比当前关键帧中的帧号都大，find到的就应该是结尾
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );  //没这个索引，插入帧，并且把id和帧配对起来，
    }
    else
    {
        keyframes_[ frame->id_ ] = frame;  //find到的不是结尾，说明map数据结构中有这个索引，中间补插入一张，直接插入
    }
}
//插入地图点
void Map::insertMapPoint ( MapPoint::Ptr map_point )  //和上面的同理
{
    if ( map_points_.find(map_point->id_) == map_points_.end() )
    {
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else 
    {
        map_points_[map_point->id_] = map_point;
    }
}


}
/*
 *  map是STL里重要容器之一。

它的特性总结来讲就是：所有元素都会根据元素的键值key自动排序（也可根据自定义的仿函数进行自定义排序），其中的每个元素都是<key, value>的键值对，map中不允许有键值相同的元素，

因此map中元素的键值key不能修改，但是可以通过key修改与其对应的value。如果一定要修改与value对应的键值key，可将已存在的key删除掉，然后重新插入。


 *
 * */