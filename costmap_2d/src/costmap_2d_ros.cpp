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
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>


using namespace std;

namespace costmap_2d
{

//检测原句柄是否包含此参数，如果包含，则将此参数复制给新的节点句柄，否则啥都不做
void move_parameter(ros::NodeHandle& old_h, ros::NodeHandle& new_h, std::string name, bool should_delete = true)
{
  if (!old_h.hasParam(name))
    return;

  XmlRpc::XmlRpcValue value;
  old_h.getParam(name, value);
  new_h.setParam(name, value);
  if (should_delete) old_h.deleteParam(name);
}

Costmap2DROS::Costmap2DROS(std::string name, tf::TransformListener& tf) :
    layered_costmap_(NULL),
    name_(name),
    tf_(tf),
    transform_tolerance_(0.3),
    map_update_thread_shutdown_(false),
    stop_updates_(false),
    initialized_(true),
    stopped_(false),
    robot_stopped_(false),
    map_update_thread_(NULL),
    last_publish_(0),
    plugin_loader_("costmap_2d", "costmap_2d::Layer"),
    publisher_(NULL),
    dsrv_(NULL),
    footprint_padding_(0.0)
{
  // Initialize old pose with something
  old_pose_.setIdentity();
  old_pose_.setOrigin(tf::Vector3(1e30, 1e30, 1e30));

  //move_base/name, 这里使用了私有命名空间。在move_base的launch文件中，给其加了global_costmap或local_costmap的前缀(对应这里的name)
  //这个命名空间的参数，定义在costmap_common_params.yaml中
  ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle g_nh;

  // get our tf prefix
  //这个节点又没初始化, 得到的前缀是啥?应该是空的
  //根据getPrefixParam函数, 需要找"tf_prefix"的参数,如果没有找到,则返回空字符,一般没有定义,所以为空
  ros::NodeHandle prefix_nh;
  //就是获取当前节点句柄的命名空间，这里应该是/move_base/name/,name是实例化的时候给定的
  std::string tf_prefix = tf::getPrefixParam(prefix_nh);

  // get two frames
  private_nh.param("global_frame", global_frame_, std::string("/map"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

  // make sure that we set the frames appropriately based on the tf_prefix
  //resolve函数,当有前缀时,加上前缀. 没有前缀, 所以frame还是不变。这里是有前缀的
  global_frame_ = tf::resolve(tf_prefix, global_frame_);
  robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);

  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  // we need to make sure that the transform between the robot base frame and the global frame is available
  //kinetic版本才有这个函数,melodic都没有这个函数了
  //如果没有找到tf变换关系,就会阻塞在这里
  while (ros::ok()
      && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01),
                               &tf_error))
  {
    ros::spinOnce();
    if (last_error + ros::Duration(5.0) < ros::Time::now())
    {
      ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
               robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = ros::Time::now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }

  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;
  private_nh.param("rolling_window", rolling_window, false);
  private_nh.param("track_unknown_space", track_unknown_space, false);
  private_nh.param("always_send_full_costmap", always_send_full_costmap, false);

  //track_unkown_space设置为true, 全局规划器就不会在未知区域规划路径
  //这个成员(包工头)非常有用, 用来加载地图
  //rolling_window为true的话, 地图会随着机器人位姿滚动(local_costmap会以机器人为中心)
  layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);

  //通过插件加载地图.plugins参数,必须得通过xml文件设置.参考clear_costmap_recovery/test/params.yaml
  //如果没有指定plugins，则会创建相关的plugin。显然是有指定的
  if (!private_nh.hasParam("plugins"))
  {
    //这个函数，有点像是更新插件的意思，转换成XmlRpc的格式，方便下面加载插件
    resetOldParameters(private_nh);
  }

  //加载地图
  //global_costmap加载了:static_layer(type:costmap_2d::StaticLayer), obstacle_layer(type:costmap_2d::VoxelLayer), inflation_layer(type:costmap_2d::InflationLayer)
  //local_costmap 加载了: obstacle_layer, inflation_layer
  //obstacle_layer是VoxeLayer,因为后者继承自前者
  //inflation_layer与costmapLayer继承自layer,而obstacle_layer和static_layer继承自costmapLayer
  if (private_nh.hasParam("plugins"))
  {
    XmlRpc::XmlRpcValue my_list;
    private_nh.getParam("plugins", my_list);
    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      std::string pname = static_cast<std::string>(my_list[i]["name"]);//比如obstacle_layer
      std::string type = static_cast<std::string>(my_list[i]["type"]);//比如costmap_2d::VoxelLayer
      ROS_INFO("Using plugin \"%s\"", pname.c_str());

      //返回插件对象的指针, plugin是根据type不同而得到的插件
      boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(type);
      layered_costmap_->addPlugin(plugin);
      //调用各类型地图的onInitialize函数.此函数定义在layer中，被costmap_layer继承，最后被其他应用layer继承
      plugin->initialize(layered_costmap_, name + "/" + pname, &tf_);
    }
  }

  // subscribe to the footprint topic
  //没有设置footprint这个参数, 所以下面的topic默认为footprint
  std::string topic_param, topic;
  if (!private_nh.searchParam("footprint_topic", topic_param))
  {
    topic_param = "footprint_topic";
  }

  //对footprint进行膨胀.问题是,为啥只膨胀了1cm?
  //订阅的footprint.是updatemap后pub出来新的footprint
  private_nh.param(topic_param, topic, std::string("footprint"));
  footprint_sub_ = private_nh.subscribe(topic, 1, &Costmap2DROS::setUnpaddedRobotFootprintPolygon, this);

  if (!private_nh.searchParam("published_footprint_topic", topic_param))
  {
    topic_param = "published_footprint";
  }

  //updatemap后,更新footprint
  private_nh.param(topic_param, topic, std::string("oriented_footprint"));
  footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);

  //makeFootprintFromParams根据footprint或radius得到描述小车大小的点
  //setUnpaddedRobotFootprint根据找到的点,膨胀一定的比例
  setUnpaddedRobotFootprint(makeFootprintFromParams(private_nh));

  //Costmap2DPublisher类主要两个函数, 一个是updatebounds,一个是publishCostmap
  publisher_ = new Costmap2DPublisher(&private_nh, layered_costmap_->getCostmap(), global_frame_, "costmap",
                                      always_send_full_costmap);

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a time r to check if the robot is moving
  //创建一个定时器去检查小车是否有移动
  robot_stopped_ = false;
  timer_ = private_nh.createTimer(ros::Duration(.1), &Costmap2DROS::movementCB, this);

  //加载动态参数，这里面包含了地图大小，坐标原点，机器人尺寸等参数。local_costmap的大小就是通过这里设定的
  //但是这是在地图加载后才调用的。而且是通过load yaml修改的参数服务器里的参数，这里加载的是动态参数。
  //应该参数服务器保存了所有命名空间的参数，同事通过dynamic reconfigure的方式，可以去修改参数服务器里面的参数
  dsrv_ = new dynamic_reconfigure::Server<Costmap2DConfig>(ros::NodeHandle("~/" + name));
  //构造函数初始化结束的时候, 开启了一个线程来updateMap.
  //这里的参数，都跟name有关，即如果name是global_planner，那么调用的都是global_planner下的参数
  dynamic_reconfigure::Server<Costmap2DConfig>::CallbackType cb = boost::bind(&Costmap2DROS::reconfigureCB, this, _1,
                                                                              _2);
  dsrv_->setCallback(cb);
}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL)
    delete publisher_;

  delete layered_costmap_;
  delete dsrv_;
}


void Costmap2DROS::resetOldParameters(ros::NodeHandle& nh)
{
  ROS_INFO("Loading from pre-hydro parameter style");
  bool flag;
  std::string s;
  std::vector < XmlRpc::XmlRpcValue > plugins;

  XmlRpc::XmlRpcValue::ValueStruct map;
  SuperValue super_map;
  //这是默认构造的plugins
  SuperValue super_array;

  //如果存在static_map参数，就构造一个static_layer，下同
  if (nh.getParam("static_map", flag) && flag)
  {
    //构造一个static_layer的plugin
    map["name"] = XmlRpc::XmlRpcValue("static_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::StaticLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    ros::NodeHandle map_layer(nh, "static_layer");
    move_parameter(nh, map_layer, "map_topic");
    move_parameter(nh, map_layer, "unknown_cost_value");
    move_parameter(nh, map_layer, "lethal_cost_threshold");
    move_parameter(nh, map_layer, "track_unknown_space", false);
  }

  ros::NodeHandle obstacles(nh, "obstacle_layer");
  if (nh.getParam("map_type", s) && s == "voxel")
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::VoxelLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    move_parameter(nh, obstacles, "origin_z");
    move_parameter(nh, obstacles, "z_resolution");
    move_parameter(nh, obstacles, "z_voxels");
    move_parameter(nh, obstacles, "mark_threshold");
    move_parameter(nh, obstacles, "unknown_threshold");
    move_parameter(nh, obstacles, "publish_voxel_map");
  }
  else
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::ObstacleLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);
  }

  move_parameter(nh, obstacles, "max_obstacle_height");
  move_parameter(nh, obstacles, "raytrace_range");
  move_parameter(nh, obstacles, "obstacle_range");
  move_parameter(nh, obstacles, "track_unknown_space", true);
  nh.param("observation_sources", s, std::string(""));
  std::stringstream ss(s);
  std::string source;
  while (ss >> source)
  {
    move_parameter(nh, obstacles, source);
  }
  move_parameter(nh, obstacles, "observation_sources");

  ros::NodeHandle inflation(nh, "inflation_layer");
  move_parameter(nh, inflation, "cost_scaling_factor");
  move_parameter(nh, inflation, "inflation_radius");
  map["name"] = XmlRpc::XmlRpcValue("inflation_layer");
  map["type"] = XmlRpc::XmlRpcValue("costmap_2d::InflationLayer");
  super_map.setStruct(&map);
  plugins.push_back(super_map);

  super_array.setArray(&plugins);
  nh.setParam("plugins", super_array);
}

void Costmap2DROS::reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level)
{

  //首次加载的时候，config中的参数已经更新为参数服务器定义的参数了。所以local_costmap_param.yaml文件中定义width，height参数，可以被用来resizemap
  //local_costmap中的inflation_layer也跟obstacle_layer一样的大小。它们公用layered_costmap_指针
  
  //而global_costmap没有指定width，height参数，它的地图大小在static_layer中调用了resizeMap函数来更新地图了
  //global_costmap中obstacle_layer的大小呢？所有layer公用一个layered_costmap_,这个参数保存了一张costmap_,所以会跟static_layer一样大
  transform_tolerance_ = config.transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = config.update_frequency;

  double map_publish_frequency = config.publish_frequency;
  if (map_publish_frequency > 0)
    publish_cycle = ros::Duration(1 / map_publish_frequency);
  else
    publish_cycle = ros::Duration(-1);

  // find size parameters
  //local_costmap就是通过修改地图的宽度和高度，来调整costmap的大小的
  double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution, origin_x =
             config.origin_x,
         origin_y = config.origin_y;

  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  if (footprint_padding_ != config.footprint_padding)
  {
    footprint_padding_ = config.footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  readFootprintFromConfig(config, old_config_);

  old_config_ = config;

  //开启了一个更新map的线程
  map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
}

void Costmap2DROS::readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                                           const costmap_2d::Costmap2DConfig &old_config)
{
  // Only change the footprint if footprint or robot_radius has
  // changed.  Otherwise we might overwrite a footprint sent on a
  // topic by a dynamic_reconfigure call which was setting some other
  // variable.
  if (new_config.footprint == old_config.footprint &&
      new_config.robot_radius == old_config.robot_radius)
  {
    return;
  }

  //如果footprint是方型的，则逐一提取；如果是圆形底盘，则离散16个点
  if (new_config.footprint != "" && new_config.footprint != "[]")
  {
    std::vector<geometry_msgs::Point> new_footprint;
    if (makeFootprintFromString(new_config.footprint, new_footprint))
    {
        setUnpaddedRobotFootprint(new_footprint);
    }
    else
    {
        ROS_ERROR("Invalid footprint string from dynamic reconfigure");
    }
  }
  else
  {
    // robot_radius may be 0, but that must be intended at this point.
    //如果是圆形底盘，则离散成16个点的footprint
    setUnpaddedRobotFootprint(makeFootprintFromRadius(new_config.robot_radius));
  }
}

void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  //footprint_padding是dynamic参数, 可以动态修改, 默认值为1cm,进行了膨胀
  padFootprint(padded_footprint_, footprint_padding_);

  //只有inflation_layer才需要这个
  layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DROS::movementCB(const ros::TimerEvent &event)
{
  // don't allow configuration to happen while this check occurs
  // boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

  tf::Stamped < tf::Pose > new_pose;

  if (!getRobotPose(new_pose))
  {
    ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling reconfiguration");
    robot_stopped_ = false;
  }
  // make sure that the robot is not moving
  else if (fabs((old_pose_.getOrigin() - new_pose.getOrigin()).length()) < 1e-3
      && fabs(old_pose_.getRotation().angle(new_pose.getRotation())) < 1e-3)
  {
    old_pose_ = new_pose;
    robot_stopped_ = true;
  }
  else
  {
    old_pose_ = new_pose;
    robot_stopped_ = false;
  }
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle
  if (frequency == 0.0)
    return;

  ros::NodeHandle nh;
  ros::Rate r(frequency);   //按一定的频率刷新地图
  //local_costmap频率默认为5hz, global_costmap默认为1hz
  while (nh.ok() && !map_update_thread_shutdown_)
  {
    struct timeval start, end;
    double start_t, end_t, t_diff;
    //这个计时的函数挺好用，很简单
    gettimeofday(&start, NULL);

    //核心在这个函数里的,layered_costmap_->updateMap(x,y,yaw)
    //同时也会更新边界值
    updateMap();

    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_DEBUG("Map update time: %.9f", t_diff);
    if (publish_cycle.toSec() > 0 && layered_costmap_->isInitialized())
    {
      unsigned int x0, y0, xn, yn;
      //获取上次更新的边界值
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      //更新边界。publiser初始化的时候调用了costmap_，将自己的成员变量xn_,yn_设置成为了地图大小
      //所以这里把边界也该成了地图大小了
      publisher_->updateBounds(x0, xn, y0, yn);

      ros::Time now = ros::Time::now();
      if (last_publish_ + publish_cycle < now)
      {
        //发布更新后的地图,发布命名空间/costmpa话题,就是发布地图.可以被rviz等订阅
        publisher_->publishCostmap();
        last_publish_ = now;
      }
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency,
               r.cycleTime().toSec());
  }
}


void Costmap2DROS::updateMap()
{
  if (!stop_updates_)
  {
    // get global pose
    tf::Stamped < tf::Pose > pose;
    if (getRobotPose (pose))
    {
      double x = pose.getOrigin().x(),
             y = pose.getOrigin().y(),
             yaw = tf::getYaw(pose.getRotation());
             
      /**分两个阶段,第一阶段是updateBounds.第二个阶段是updateCosts
      *这个更新,是layered_costmap调用的,会更新所有插件地图
      */
      layered_costmap_->updateMap(x, y, yaw);

      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = ros::Time::now();
      //根据机器人的位姿,以及footprint,得到最后一个参数:oriented_footprint
      transformFootprint(x, y, yaw, padded_footprint_, footprint);
      //这里发布的是一个新的footprint
      footprint_pub_.publish(footprint);

      initialized_ = true;
    }
  }
}

void Costmap2DROS::start()
{
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused
  if (stopped_)
  {
    // if we're stopped we need to re-subscribe to topics
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    stopped_ = false;
  }
  //更新此标志位后, 就会updatemap了,在里面执行完后, 会更新initialized_标志位
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  //阻塞直到costmap初始化
  ros::Rate r(100.0);
  while (ros::ok() && !initialized_)
    r.sleep();
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
}


void Costmap2DROS::resetLayers()
{
  Costmap2D* top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

//这个函数的robot_pose是temp变量,并没有提供位姿,怎么更新得到global_pose?
//难道是因为tf变换关系,有一直在更新吗?  是的, 里程计会一直在更新
bool Costmap2DROS::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
{
  //先清零
  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    //tf求坐标变换，只需要知道其中一个对象的时间和frame_id即可，而不需要知道其位姿
    tf_.transformPose(global_frame_, robot_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const
{
  tf::Stamped<tf::Pose> global_pose;
  if (!getRobotPose(global_pose))
    return;

  double yaw = tf::getYaw(global_pose.getRotation());
  transformFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw,
                     padded_footprint_, oriented_footprint);
}

}  // namespace costmap_2d
