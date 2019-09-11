/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#include <costmap_2d/observation_buffer.h>

#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace tf;

namespace costmap_2d
{
ObservationBuffer::ObservationBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, TransformListener& tf, string global_frame,
                                     string sensor_frame, double tf_tolerance) :
    tf_(tf), observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
    last_updated_(ros::Time::now()), global_frame_(global_frame), sensor_frame_(sensor_frame), topic_name_(topic_name),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance)
{
}

ObservationBuffer::~ObservationBuffer()
{
}

//扎心了，这个函数竟然没有被使用过。就是把传感器数据，全部转换到给定坐标系下
bool ObservationBuffer::setGlobalFrame(const std::string new_global_frame)
{
  ros::Time transform_time = ros::Time::now();
  std::string tf_error;

  if (!tf_.waitForTransform(new_global_frame, global_frame_, transform_time, ros::Duration(tf_tolerance_),
                            ros::Duration(0.01), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
    return false;
  }

  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    try
    {
      Observation& obs = *obs_it;

      geometry_msgs::PointStamped origin;
      origin.header.frame_id = global_frame_;
      origin.header.stamp = transform_time;
      origin.point = obs.origin_;

      // we need to transform the origin of the observation to the new global frame
      tf_.transformPoint(new_global_frame, origin, origin);
      obs.origin_ = origin.point;

      // we also need to transform the cloud of the observation to the new global frame
      pcl_ros::transformPointCloud(new_global_frame, *obs.cloud_, *obs.cloud_, tf_);
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", global_frame_.c_str(),
                new_global_frame.c_str(), ex.what());
      return false;
    }
  }

  // now we need to update our global_frame member
  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::bufferCloud(const sensor_msgs::PointCloud2& cloud)
{
  try
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2); //这是一个ros包，可能真的只能转换PointCloud2类型的数据
    // Actually convert the PointCloud2 message into a type we can reason about
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    //把pcl::PointClound2再转换为pcl::PointCloud
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    bufferCloud(pcl_cloud);
  }
  catch (pcl::PCLException& ex)
  {
    ROS_ERROR("Failed to convert a message to a pcl type, dropping observation: %s", ex.what());
    return;
  }
}

//了解了。Pointcloud是个模板类，必须继承其他的类。pcl类也有header这个头？真的有，感觉rosmsg就是抄的pcl
void ObservationBuffer::bufferCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  //这是一个tf数据结构
  Stamped < tf::Vector3 > global_origin;

  // create a new observation on the list to be populated
  //std::list<Observation>是一个list容器, 每一次激光扫描会插入一个observation对象
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try
  {
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    //local_origin的坐标原点默认为0，下面将此坐标转换到map坐标系下
    Stamped < tf::Vector3 > local_origin(tf::Vector3(0, 0, 0),
                            pcl_conversions::fromPCL(cloud.header).stamp, origin_frame);
    tf_.waitForTransform(global_frame_, local_origin.frame_id_, local_origin.stamp_, ros::Duration(0.5));
    //得到激光雷达在全局坐标系下的点
    tf_.transformPoint(global_frame_, local_origin, global_origin);
    //所以观测点的坐标原点，是传感器在map坐标系下的值
    observation_list_.front().origin_.x = global_origin.getX();
    observation_list_.front().origin_.y = global_origin.getY();
    observation_list_.front().origin_.z = global_origin.getZ();

    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;

    pcl::PointCloud < pcl::PointXYZ > global_frame_cloud;

    // transform the point cloud
    //把点云数据转换到map坐标系下
    pcl_ros::transformPointCloud(global_frame_, cloud, global_frame_cloud, tf_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // now we need to remove observations from the cloud that are below or above our height thresholds
    //这里用到了c++的引用
    pcl::PointCloud < pcl::PointXYZ > &observation_cloud = *(observation_list_.front().cloud_);
    unsigned int cloud_size = global_frame_cloud.points.size();
    observation_cloud.points.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    //去除超出高度范围的点, 然后保存在点云数据中.因为2d激光雷达的高度低于此最小高度, 导致一直检测不到障碍物
    for (unsigned int i = 0; i < cloud_size; ++i)
    {
      if (global_frame_cloud.points[i].z <= max_obstacle_height_
          && global_frame_cloud.points[i].z >= min_obstacle_height_)
      {
        observation_cloud.points[point_count++] = global_frame_cloud.points[i];
      }
    }

    // resize the cloud for the number of legal points
    observation_cloud.points.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  }
  catch (TransformException& ex)
  {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  //晚于最新的scan的时间
  last_updated_ = ros::Time::now();

  // we'll also remove any stale observations from the list
  //清除旧的障碍物
  purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(vector<Observation>& observations)
{
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    observations.push_back(*obs_it);
  }
}



// 清除旧的障碍物点
void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty())
  {
    list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    // 设置为0，即不保留之前观测的数据，observation_list只保留当场观测数据
    // 直接返回了
    if (observation_keep_time_ == ros::Duration(0.0))
    {
      //把第二个元素到最后一个元素中间的都清除掉，移除[first,last）的元素
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    //如果保留之前的数据，那么根据保留的时间，将超过这个时间的数据删除掉
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      Observation& obs = *obs_it;
      // check if the observation is out of date... and if it is, remove it and those that follow from the list
      //后面的stamp是传感器的接收时间，last_updated肯定要大于此时间的，那不得把所有元素都给删除了
      ros::Duration time_diff = last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
      if (time_diff > observation_keep_time_)
      {
        //移除[first,last）的元素,
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

//检查该topic是否按其设定的频率在更新
bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == ros::Duration(0.0))
    return true;

  bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
  if (!current)
  {
    ROS_WARN(
        "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
  }
  return current;
}

void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = ros::Time::now();
}
}  // namespace costmap_2d
