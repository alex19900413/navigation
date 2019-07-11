/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/* Author: Wim Meeussen */

#include <robot_pose_ekf/odom_estimation_node.h>


using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;


static const double EPS = 1e-5;


//#define __EKF_DEBUG_FILE__

namespace estimation
{
  // constructor
  OdomEstimationNode::OdomEstimationNode()
    : odom_active_(false),
      imu_active_(false),
      vo_active_(false),
      gps_active_(false),
      uwb_active_(false),
      odom_initializing_(false),
      imu_initializing_(false),
      vo_initializing_(false),
      gps_initializing_(false),
      uwb_initializing_(false),
      odom_covariance_(6),
      imu_covariance_(3),
      vo_covariance_(6),
      gps_covariance_(3),
      uwb_covariance_(3),
      odom_callback_counter_(0),
      imu_callback_counter_(0),
      vo_callback_counter_(0),
      gps_callback_counter_(0),
      uwb_callback_counter_(0),
      ekf_sent_counter_(0)
  {
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    // paramters
    nh_private.param("output_frame", output_frame_, std::string("odom_combined"));
    nh_private.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint"));
    nh_private.param("sensor_timeout", timeout_, 1.0);
    nh_private.param("odom_used", odom_used_, true);
    nh_private.param("imu_used",  imu_used_, true);
    nh_private.param("vo_used",   vo_used_, true);
    nh_private.param("gps_used",   gps_used_, false);
    nh_private.param("uwb_used",   uwb_used_, false);
    nh_private.param("debug",   debug_, false);
    nh_private.param("self_diagnose",  self_diagnose_, false);
    double freq;
    nh_private.param("freq", freq, 30.0);

    //这波操作难道是从launch文件读取参数的必备步骤？
    //tf_prefix是tf老版本的概念，用/作为框架名称的前缀。是不是如果launch文件中给定/odom和odom一样的。/odom则会清除掉前缀，odom则不操作
    tf_prefix_ = tf::getPrefixParam(nh_private);
    output_frame_ = tf::resolve(tf_prefix_, output_frame_);
    base_footprint_frame_ = tf::resolve(tf_prefix_, base_footprint_frame_);

    ROS_INFO_STREAM("output frame: " << output_frame_);
    ROS_INFO_STREAM("base frame: " << base_footprint_frame_);

    // set output frame and base frame names in OdomEstimation filter
    // so that user-defined tf frames are respected
    //在launch文件中修改对应的tf frame
    my_filter_.setOutputFrame(output_frame_);
    my_filter_.setBaseFootprintFrame(base_footprint_frame_);

    //开启一个定时器，执行循环滤波操作
    //虽然定时器函数在topic订阅函数之前，但是不影响。因为spin中的时间大于topic初始时间时，才会激活传感器，才可以进行update
    //所以这里先进入spin，但是时间小于topic订阅时间，所以也不会update
    timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);

    // advertise our estimation
    pose_pub_ = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_combined", 10);

    // initialize
    filter_stamp_ = Time::now();

    //订阅一系列传感器数据
    // subscribe to odom messages
    if (odom_used_){
      ROS_DEBUG("Odom sensor can be used");
      odom_sub_ = nh.subscribe("odom", 10, &OdomEstimationNode::odomCallback, this);
    }
    else ROS_DEBUG("Odom sensor will NOT be used");

    // subscribe to imu messages
    //modify imu topic type
    if (imu_used_){
      ROS_DEBUG("Imu sensor can be used");
      imu_sub_ = nh.subscribe("imu", 10,  &OdomEstimationNode::imuCallback, this);
    }
    else ROS_DEBUG("Imu sensor will NOT be used");

    // subscribe to vo messages
    if (vo_used_){
      ROS_DEBUG("VO sensor can be used");
      vo_sub_ = nh.subscribe("vo", 10, &OdomEstimationNode::voCallback, this);
    }
    else ROS_DEBUG("VO sensor will NOT be used");

    if (gps_used_){
      ROS_DEBUG("GPS sensor can be used");
      gps_sub_ = nh.subscribe("gps", 10, &OdomEstimationNode::gpsCallback, this);
    }
    else ROS_DEBUG("GPS sensor will NOT be used");

    if (uwb_used_){
      ROS_DEBUG("UWB sensor can be used");
      gps_sub_ = nh.subscribe("uwb", 10, &OdomEstimationNode::uwbCallback, this);
    }
    else ROS_DEBUG("GPS sensor will NOT be used");


    // publish state service
    state_srv_ = nh_private.advertiseService("get_status", &OdomEstimationNode::getStatus, this);

    if (debug_){
      // open files for debugging
      odom_file_.open("/tmp/odom_file.txt");
      imu_file_.open("/tmp/imu_file.txt");
      vo_file_.open("/tmp/vo_file.txt");
      gps_file_.open("/tmp/gps_file.txt");
      uwb_file_.open("/tmp/gps_file.txt");
      corr_file_.open("/tmp/corr_file.txt");

  
    }
  };














  // destructor
  OdomEstimationNode::~OdomEstimationNode(){

    if (debug_){
      // close files for debugging
      odom_file_.close();
      imu_file_.close();
      gps_file_.close();
      uwb_file_.close();
      vo_file_.close();
      corr_file_.close();
    }
  };





  // callback function for odom data
  void OdomEstimationNode::odomCallback(const OdomConstPtr& odom)
  {
    odom_callback_counter_++;

    ROS_DEBUG("Odom callback at time %f ", ros::Time::now().toSec());
    assert(odom_used_);

    // receive data 
    odom_stamp_ = odom->header.stamp;
    odom_time_  = Time::now();
    Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    //构造一个transform，odom to base_link
    odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];

    //将tf添加到transformer_ buffer中，用于记录和查询tf
    //odomtopic是odom to base_link，这里取逆了，所以是base_link to odom。正如网上说的，用的是其相对坐标的数据
    //modify wheelodom
    my_filter_.addMeasurement(StampedTransform(odom_meas_.inverse(), odom_stamp_, base_footprint_frame_, "odom"), odom_covariance_);
    
    // activate odom
    //初始化的时候是false，订阅到第一条数据就被激活了
    if (!odom_active_) {
      if (!odom_initializing_){
        odom_initializing_ = true;
        odom_init_stamp_ = odom_stamp_;
        ROS_INFO("Initializing Odom sensor");      
      }
      //filter_stamp在订阅topic之前，所以不可能大于的啊。在filter loop中，会更新filter_stamp
      //滤波器的时间戳必须大于odom的时间戳，才会将odom激活
      if ( filter_stamp_ >= odom_init_stamp_){
        odom_active_ = true;
        odom_initializing_ = false;
        ROS_INFO("Odom sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate Odom, because Odom measurements are still %f sec in the future.", 
		    (odom_init_stamp_ - filter_stamp_).toSec());
    }
    
    //这个可以，还可以写log文件的，默认为false
    if (debug_){
      // write to file
      double tmp, yaw;
      odom_meas_.getBasis().getEulerYPR(yaw, tmp, tmp);
      odom_file_<< fixed <<setprecision(5) << ros::Time::now().toSec() << " " << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
    }
  };




  // callback function for imu data
  void OdomEstimationNode::imuCallback(const ImuConstPtr& imu)
  {
    imu_callback_counter_++;

    assert(imu_used_);

    // receive data 
    imu_stamp_ = imu->header.stamp;
    tf::Quaternion orientation;
    quaternionMsgToTF(imu->orientation, orientation);
    //imu的transform的位姿，这里就设置为0了，后面会更新
    imu_meas_ = tf::Transform(orientation, tf::Vector3(0,0,0));
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];

    // Transforms imu data to base_footprint frame
    if (!robot_state_.waitForTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, ros::Duration(0.5))){
      // warn when imu was already activated, not when imu is not active yet
      if (imu_active_)
        ROS_ERROR("Could not transform imu message from %s to %s", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else if (my_filter_.isInitialized())
        ROS_WARN("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else 
        ROS_DEBUG("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      return;
    }
    StampedTransform base_imu_offset;
    robot_state_.lookupTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, base_imu_offset);
    //这里更新了imu_meas_
    imu_meas_ = imu_meas_ * base_imu_offset;

    imu_time_  = Time::now();

    // manually set covariance untile imu sends covariance
    //如果imu没有设定协方差，手动设置一个值
    if (imu_covariance_(1,1) == 0.0){
      SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
      imu_covariance_ = measNoiseImu_Cov;
    }

    my_filter_.addMeasurement(StampedTransform(imu_meas_.inverse(), imu_stamp_, base_footprint_frame_, "imu_link"), imu_covariance_);
    
    // activate imu
    if (!imu_active_) {
      if (!imu_initializing_){
	imu_initializing_ = true;
	imu_init_stamp_ = imu_stamp_;
	ROS_INFO("Initializing Imu sensor");      
      }
      if ( filter_stamp_ >= imu_init_stamp_){
	imu_active_ = true;
	imu_initializing_ = false;
	ROS_INFO("Imu sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate IMU, because IMU measurements are still %f sec in the future.", 
		    (imu_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      imu_meas_.getBasis().getEulerYPR(yaw, tmp, tmp); 
      imu_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< yaw << endl;
    }
  };




  // callback function for VO data
  void OdomEstimationNode::voCallback(const VoConstPtr& vo)
  {
    vo_callback_counter_++;

    assert(vo_used_);

    // get data
    vo_stamp_ = vo->header.stamp;
    vo_time_  = Time::now();
    poseMsgToTF(vo->pose.pose, vo_meas_);
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        vo_covariance_(i+1, j+1) = vo->pose.covariance[6*i+j];
    my_filter_.addMeasurement(StampedTransform(vo_meas_.inverse(), vo_stamp_, base_footprint_frame_, "vo"), vo_covariance_);
    
    // activate vo
    if (!vo_active_) {
      if (!vo_initializing_){
	vo_initializing_ = true;
	vo_init_stamp_ = vo_stamp_;
	ROS_INFO("Initializing Vo sensor");      
      }
      if (filter_stamp_ >= vo_init_stamp_){
	vo_active_ = true;
	vo_initializing_ = false;
	ROS_INFO("Vo sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate VO, because VO measurements are still %f sec in the future.", 
		    (vo_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double Rx, Ry, Rz;
      vo_meas_.getBasis().getEulerYPR(Rz, Ry, Rx);
      vo_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< vo_meas_.getOrigin().x() << " " << vo_meas_.getOrigin().y() << " " << vo_meas_.getOrigin().z() << " "
               << Rx << " " << Ry << " " << Rz << endl;
    }
  };


  void OdomEstimationNode::gpsCallback(const GpsConstPtr& gps)
  {
    gps_callback_counter_++;

    assert(gps_used_);

    // get data
    gps_stamp_ = gps->header.stamp;
    gps_time_  = Time::now();
    geometry_msgs::PoseWithCovariance gps_pose = gps->pose;
    if (isnan(gps_pose.pose.position.z)){
      // if we have no linear z component in the GPS message, set it to 0 so that we can still get a transform via `tf
      // (which does not like "NaN" values)
      gps_pose.pose.position.z = 0;
      // set the covariance for the linear z component very high so we just ignore it
      gps_pose.covariance[6*2 + 2] = std::numeric_limits<double>::max();
    }
    poseMsgToTF(gps_pose.pose, gps_meas_);
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        gps_covariance_(i+1, j+1) = gps_pose.covariance[6*i+j];
    my_filter_.addMeasurement(StampedTransform(gps_meas_.inverse(), gps_stamp_, base_footprint_frame_, "gps"), gps_covariance_);
    
    // activate gps
    if (!gps_active_) {
      if (!gps_initializing_){
	    gps_initializing_ = true;
	    gps_init_stamp_ = gps_stamp_;
	    ROS_INFO("Initializing GPS sensor");      
      }
      if (filter_stamp_ >= gps_init_stamp_){
	    gps_active_ = true;
	    gps_initializing_ = false;
	    ROS_INFO("GPS sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate GPS, because GPS measurements are still %f sec in the future.", 
		    (gps_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      gps_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< gps_meas_.getOrigin().x() << " " << gps_meas_.getOrigin().y() << " " << gps_meas_.getOrigin().z() <<endl;
    }
  };


  void OdomEstimationNode::uwbCallback(const UwbConstPtr& uwb){
    uwb_callback_counter_++;

    assert(uwb_used_);

    // get data
    uwb_stamp_ = uwb->header.stamp;
    uwb_time_  = Time::now();
    //uwb我用的geometry_msgs::point,没有定义协方差哦
    geometry_msgs::PoseWithCovariance uwb_pose;
    uwb_pose.pose.position = uwb->position;
    // if (isnan(uwb_pose.pose.position.z)){
    //   // if we have no linear z component in the GPS message, set it to 0 so that we can still get a transform via `tf
    //   // (which does not like "NaN" values)
    //   uwb_pose.pose.position.z = 0;
    //   // set the covariance for the linear z component very high so we just ignore it
    //   uwb_pose.covariance[6*2 + 2] = std::numeric_limits<double>::max();
    // }
    poseMsgToTF(uwb_pose.pose, uwb_meas_);
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        uwb_covariance_(i+1, j+1) = uwb_pose.covariance[3*i+j];
    //如果uwb没有设定协方差，手动设置一个值
    if (uwb_covariance_(1,1) == 0.0){
      SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  
      uwb_covariance_ = measNoiseImu_Cov;
    }
    my_filter_.addMeasurement(StampedTransform(uwb_meas_.inverse(), uwb_stamp_, base_footprint_frame_, "uwb_label"), uwb_covariance_);
    
    // activate gps
    if (!uwb_active_) {
      if (!uwb_initializing_){
	    uwb_initializing_ = true;
	    uwb_init_stamp_ = gps_stamp_;
	    ROS_INFO("Initializing uwb sensor");      
      }
      if (filter_stamp_ >= uwb_init_stamp_){
	    uwb_active_ = true;
	    uwb_initializing_ = false;
	    ROS_INFO("uwb sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate uwb, because uwb measurements are still %f sec in the future.", 
		    (uwb_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      uwb_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< uwb_meas_.getOrigin().x() << " " << uwb_meas_.getOrigin().y() << " " << uwb_meas_.getOrigin().z() <<endl;
    }
  };









  // filter loop
  void OdomEstimationNode::spin(const ros::TimerEvent& e)
  {
    ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

    // check for timing problems
    //检查imu和odom的时间间隔是否超过1s，没法融合
    if ( (odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_) ){
      double diff = fabs( Duration(odom_stamp_ - imu_stamp_).toSec() );
      if (diff > 1.0) ROS_ERROR("Timestamps of odometry and imu are %f seconds apart.", diff);
    }
    
    // initial value for filter stamp; keep this stamp when no sensors are active
    filter_stamp_ = Time::now();
    
    // check which sensors are still active
    //如果传感器数据的时间已经超时了1s，则丢弃。并把传感器状态置false
    if ((odom_active_ || odom_initializing_) && 
        (Time::now() - odom_time_).toSec() > timeout_){
      odom_active_ = false; odom_initializing_ = false;
      ROS_INFO("Odom sensor not active any more");
    }
    if ((imu_active_ || imu_initializing_) && 
        (Time::now() - imu_time_).toSec() > timeout_){
      imu_active_ = false;  imu_initializing_ = false;
      ROS_INFO("Imu sensor not active any more");
    }
    if ((vo_active_ || vo_initializing_) && 
        (Time::now() - vo_time_).toSec() > timeout_){
      vo_active_ = false;  vo_initializing_ = false;
      ROS_INFO("VO sensor not active any more");
    }

    if ((gps_active_ || gps_initializing_) && 
        (Time::now() - gps_time_).toSec() > timeout_){
      gps_active_ = false;  gps_initializing_ = false;
      ROS_INFO("GPS sensor not active any more");
    }

    if ((uwb_active_ || uwb_initializing_) && 
        (Time::now() - uwb_time_).toSec() > timeout_){
      uwb_active_ = false;  uwb_initializing_ = false;
      ROS_INFO("uwb sensor not active any more");
    }

    
    // only update filter when one of the sensors is active
    //只要有一个传感器被激活了，就可以去update
    if (odom_active_ || imu_active_ || vo_active_ || gps_active_ || uwb_active_){
      
      // update filter at time where all sensor measurements are available
      //将filter_stamp设置为最早的传感器的时间。本来filter_stamp是要大于所有传感器时间的
      if (odom_active_)  filter_stamp_ = min(filter_stamp_, odom_stamp_);
      if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
      if (vo_active_)    filter_stamp_ = min(filter_stamp_, vo_stamp_);
      if (gps_active_)  filter_stamp_ = min(filter_stamp_, gps_stamp_);
      if (uwb_active_)  filter_stamp_ = min(filter_stamp_, uwb_stamp_);

      
      // update filter
      //默认是没有初始化的，在后面进行了初始化，用传感器的数据的第一帧数据作为先验
      //建立好ekf之后，就是置true。在第二帧odom数据收到的时候开始初始化
      if ( my_filter_.isInitialized() )  {
        bool diagnostics = true;
        //关键是这个函数，实现了数据的融合
        if (my_filter_.update(odom_active_, imu_active_,gps_active_, vo_active_, uwb_active_,  filter_stamp_, diagnostics)){
          
          // output most recent estimate and relative covariance
          //发布最近的状态估计及协方差矩阵
          my_filter_.getEstimate(output_);
          //发布融合后的位姿，只是用来debug的吧
          pose_pub_.publish(output_);
          ekf_sent_counter_++;
          
          // broadcast most recent estimate to TransformArray
          //发布融合后的tf
          StampedTransform tmp;
          //获取融合后的位姿，base_link to odom,其实who to who没关系，只要能建立变换就可以
          my_filter_.getEstimate(ros::Time(), tmp);
          if(!vo_active_ && !gps_active_)
            tmp.getOrigin().setZ(0.0);
          odom_broadcaster_.sendTransform(StampedTransform(tmp, tmp.stamp_, output_frame_, base_footprint_frame_));
          
          if (debug_){
            // write to file
            ColumnVector estimate; 
            my_filter_.getEstimate(estimate);
            corr_file_ << fixed << setprecision(5)<<ros::Time::now().toSec()<<" ";
            
            for (unsigned int i=1; i<=6; i++)
            corr_file_ << estimate(i) << " ";
            corr_file_ << endl;
          }
        }
        if (self_diagnose_ && !diagnostics)
          ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
      }


      // initialize filer with odometry frame
      //如果有gps数据，则可以用imu来初始化，否则得用odom来初始化。odom可以是轮速计得到的，也可以是vo计算出来的
      //如果用uwb的话，是不是可以用uwb+imu来初始化呢。不用，还是用odom来初始化
      if (imu_active_ && gps_active_ && !my_filter_.isInitialized()) {
	      Quaternion q = imu_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and imu measurement");
      }	
      else if ( odom_active_ && gps_active_ && !my_filter_.isInitialized()) {
	      Quaternion q = odom_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and odometry measurement");
      }
      else if ( vo_active_ && gps_active_ && !my_filter_.isInitialized()) {
	      Quaternion q = vo_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and visual odometry measurement");
      }
      //一般情况是imu+odom的融合，用odom作为预测。那我可以在这里改一下，用uwb去做预测怎么样？
      else if ( odom_active_  && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        ROS_INFO("Kalman filter initialized with odom measurement");
      }
      else if ( vo_active_ && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(vo_meas_, vo_stamp_);
        ROS_INFO("Kalman filter initialized with vo measurement");
      }
    }
  };







bool OdomEstimationNode::getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp)
{
  stringstream ss;
  ss << "Input:" << endl;
  ss << " * Odometry sensor" << endl;
  ss << "   - is "; if (!odom_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!odom_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << odom_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << odom_sub_.getTopic() << endl;
  ss << " * IMU sensor" << endl;
  ss << "   - is "; if (!imu_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!imu_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << imu_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << imu_sub_.getTopic() << endl;
  ss << " * Visual Odometry sensor" << endl;
  ss << "   - is "; if (!vo_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!vo_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << vo_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << vo_sub_.getTopic() << endl;
  ss << " * GPS sensor" << endl;
  ss << "   - is "; if (!gps_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!gps_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << gps_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << gps_sub_.getTopic() << endl;
  ss << "Output:" << endl;
  ss << " * Robot pose ekf filter" << endl;
  ss << "   - is "; if (!my_filter_.isInitialized()) ss << "NOT "; ss << "active" << endl;
  ss << "   - sent " << ekf_sent_counter_ << " messages" << endl;
  ss << "   - pulishes on topics " << pose_pub_.getTopic() << " and /tf" << endl;
  resp.status = ss.str();
  return true;
}

}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot_pose_ekf");

  // create filter class
  OdomEstimationNode my_filter_node;

  ros::spin();
  
  return 0;
}
