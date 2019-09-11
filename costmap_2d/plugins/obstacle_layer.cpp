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
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstacleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void ObstacleLayer::onInitialize()
{
  //这里这么做，就是为了将命名空间改成指定的name_空间。name_在layered_costmap的initialize函数中指定
  //下面的这些参数，都是在这个命名空间下的。在launch文件中，local_costmap和global_costmap的命名空间，都会加载common_costmap_params.yaml文件
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  //当global planning穿过unknown_space时，不允许其生成路径
  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  //根据设置，改变默认值的类型。默认为true。
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  //更新了地图的大小.如果是local_costmap，那大小应该在哪儿初始化呢？  在构造local_costmap时，根据参数更新了costmap_的大小
  // 参数回调函数中，调用了layered_costmap的resizeMap函数
  ObstacleLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  //waitforTf的block时间，超过这个时间监听不到tf变换则报错
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  //需要在launch的yaml文件中指定观测来源,一般为base scan
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  // get our tf prefix，当前命名空间可以为：/move_base/local_costmap/obstacle_layer/
  ros::NodeHandle prefix_nh;
  const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

  // now we need to split the topics based on whitespace which we can use a stringstream for
  //所以，如果要加其他传感器的数据，在这里加就好了，用空格隔开。且每个传感器，都得指定相关参数
  std::stringstream ss(topics_string);

  std::string source;
  //读取参数,scan参数设置再costmap_common_params.yaml文件中
  while (ss >> source)
  {
    //重映射节点.如果source是相对路径,则新节点为nh/source;如果source是绝对路径,那么新节点为source
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    //如果不给定，默认值就是对应的传感器名字
    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));   //可不设置, 因为scan msg中会有包含的
    //0表示只保留当次的数据
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    //如果设置为true，则inf值会被设置为最大距离
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);  //是否标记障碍物

    if (!sensor_frame.empty())
    {
      sensor_frame = tf::resolve(tf_prefix, sensor_frame);
    }

    //只有点云类型的数据才能够被使用
    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
    {
      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
    }

    std::string raytrace_range_param_name, obstacle_range_param_name;

    // get the obstacle range for the sensor
    double obstacle_range = 2.5;
    //如果能找到参数,就获取该参数值
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // get the raytrace range for the sensor
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    // create an observation buffer
    //有个专门的observationBuffer类,用来保存激光雷达/点云数据
    //由于目前只有设置了scan类型，所以这个数组当前只有一个元素，保存的就是激光的观测值。
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer> (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance)));

    // check if we'll add this buffer to our marking observation buffers
    //true，默认要标记障碍物。把激光雷达扫到的点，都保存到marking_buffers这个列表中去
    if (marking)
      marking_buffers_.push_back(observation_buffers_.back());

    // check if we'll also add this buffer to our clearing observation buffers
    //true
    if (clearing)
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic
    //目前只有laserscan，后续是不是可以添加红外，超声波？这些都是sensor_msgs/Range类型的，这里显然没有。没关系，有专门的range_layer
    if (data_type == "LaserScan")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
          > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));

      if (inf_is_valid)
      {
        filter->registerCallback(
            boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
      }
      else
      {
        filter->registerCallback(
            boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
    }
    else if (data_type == "PointCloud")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud>
          > filter(new tf::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50));
      filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }
    else//datatype是PointCloud2类型的
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
          > filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));
      filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "")
    {
      std::vector < std::string > target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}















void ObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  //指定命名空间nh
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
      &ObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

ObstacleLayer::~ObstacleLayer()
{
    if (dsrv_)
        delete dsrv_;
}
void ObstacleLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  combination_method_ = config.combination_method;
}










void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  //激光雷达msg是距离信息,实际用的是点云信息
  //最终都要转换成PointCloud2的类型
  try
  {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
    projector_.projectLaser(*message, cloud);
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void ObstacleLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[ i ];
    if (!std::isfinite(range) && range > 0)
    {
      message.ranges[ i ] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
             global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}


void ObstacleLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                               const boost::shared_ptr<ObservationBuffer>& buffer)
{
  sensor_msgs::PointCloud2 cloud2;
  //最终都要转换成PointCloud2的类型
  if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
  {
    ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud2);
  buffer->unlock();
}

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}





//更新地图的边界
//根据测量数据完成clear操作之后, 就开始mark操作, 对每个测量的点,标记为obstacle.
//在layered_costmap这个包工头的update函数中执行
//再往前追溯，那就是costmap_2d_ros中的update 线程函数里，定时去更新地图
// 传感器观测数据保存在observations中，
void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  
  //如果执行了clear_costmap，那么这里就变成了实际地图大小了。如果没有，则不会做任何操作
  //这个范围是从layered_costmap中传进来的，可能会非常大1e30
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  // 就是拷贝的observation_list的内容，注意，这都是已经在global_map坐标系下了.也就是世界坐标系下的观测值
  // observations是障碍物观测点。clearing_observations也包含观测点
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  //marking点,是指的occupy的网格。这是激光雷达扫射到的障碍物点。保存的是所有观测值
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  //clearing是指的freespace的网格。保存的是所有观测值
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  // raytrace freespace
  // 清空障碍物点
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  //根据点云数据，更新costmap_value值，以及边界范围值
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

    //2.5 * 2.5
    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    //将扫掉的点云数据，修改其cost
    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_)
      {
        ROS_DEBUG("The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      //只检测设定障碍物距离内的点，这是一个圆形区域
      if (sq_dist >= sq_obstacle_range)
      {
        ROS_DEBUG("The point is too far away");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my))
      {
        ROS_DEBUG("Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      //把激光扫到的点的cost更新一下
      costmap_[index] = LETHAL_OBSTACLE;
      //用点云数据来更新边界范围，这里其实也就是缩小这几个变量的范围，趋于正常值
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  //根据机器人的外形尺寸，更新最小边界
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}







void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  //默认为true, 将机器人经过的地方设置为free_space
  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  switch (combination_method_)
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

//如下这两个函数，适用于动态障碍物添加和清除的函数。obstacle_layer有这个函数，但是暂时没有用到
//就是用来添加虚拟墙的吧
void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    marking_buffers_[i]->lock();
    //这个函数是清除旧的障碍物, 然后把observation_list_的障碍物保存到marking_observations
    //observation_list_保存的对象，是单次扫描的数据，而不是单个点
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  //把增加障碍物也添加到marking列表中，实际上这个列表是空的
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool ObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}





void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{
  //单次观测的坐标原点，即sensor坐标点在global_map下的坐标
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  pcl::PointCloud < pcl::PointXYZ > cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  //计算sensor在地图坐标系上的坐标
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0))
  {
    ROS_WARN_THROTTLE(
        1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy);
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  //在内循环外先预计算地图的端点，这里origin_x_未赋值，初始化为0
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  // 这么一修改，最大最小值都等于ox，oy了
  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  //对于每一个障碍物点，会清除原点到障碍物点之间的所有值
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    //点云到laser坐标点的距离
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    //如果点超出范围了，那就只更新的边界那里
    if (wx < origin_x)
    {
      //
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y)
    {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x)
    {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    //把障碍物点坐标转到地图坐标
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    // 这一部分实在看不懂是什么意思啊
    // 将raytrace距离，改为网格距离
    unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    //清除这条线上的值，即设置为free
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}

void ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

}  // namespace costmap_2d
