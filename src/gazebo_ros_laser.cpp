/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 */

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include "std_msgs/Bool.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <gazebo_plugins/gazebo_ros_laser.h>
#include "brass_gazebo_plugins/SetLidarMode.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosLaser::GazeboRosLaser()
{
  ROS_INFO_STREAM("BRASS Hokuyo Plugin loaded");
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosLaser::~GazeboRosLaser()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  this->parentSensor_ = _parent;
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosLaser controller requires a Ray Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");


  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("mode"))
  {
    this->sensor_mode_ = true;
  }
  else {
    this->sensor_mode_ = sdf->GetElement("mode")->Get<bool>();
  }

  this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_STREAM("Starting Laser Plugin (ns = " << this->robot_namespace_.c_str() << "), mode=" << this->sensor_mode_?"true":"false");
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosLaser::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosLaser::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  if(this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
  }
  ROS_INFO_NAMED("laser", "Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_, 1,
      boost::bind(&GazeboRosLaser::LaserConnect, this),
      boost::bind(&GazeboRosLaser::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::LaserScan>();
    ROS_INFO_NAMED("laser", "Lase Plugin leaving LoadThread");
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);

  this->sensor_mode_srv_ = this->rosnode_->advertiseService("/mobile_base/lidar/mode", &GazeboRosLaser::SetSensorMode, this);

  ros::AdvertiseOptions lidar_status_ao =
    ros::AdvertiseOptions::create<std_msgs::Bool>("/mobile_base/lidar/status", 1,
      boost::bind (&GazeboRosLaser::StatusConnect, this),
      boost::bind (&GazeboRosLaser::StatusDisconnect, this),
      ros::VoidPtr(), NULL);

  this->status_pub_ = this->rosnode_->advertise(lidar_status_ao);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosLaser::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1) {
    ROS_INFO_NAMED("laser", "Connecting to %s", this->parent_ray_sensor_->Topic().c_str());
    this->laser_scan_sub_ =
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                    &GazeboRosLaser::OnScan, this);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosLaser::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

void GazeboRosLaser::StatusConnect() 
{
  this->status_connect_count_++;
  this->parentSensor_->SetActive(true);
}

void GazeboRosLaser::StatusDisconnect()
{
  this->status_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboRosLaser::OnScan(ConstLaserScanStampedPtr &_msg)
{
  
  common::Time sensor_update_time_ = this->parentSensor_->LastMeasurementTime();
  if (last_status_update_ + 1 < sensor_update_time_.Float() ) {
    this->last_status_update_ = sensor_update_time_;
    std_msgs::Bool v_msg;
    v_msg.data = this->sensor_mode_;
    this->status_pub_.publish(v_msg);
  }
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  //ROS_INFO_NAMED("laser", "Got a new scan");
  if (!this->sensor_mode_) return;
  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  laser_msg.header.frame_id = this->frame_name_;
  laser_msg.angle_min = _msg->scan().angle_min();
  laser_msg.angle_max = _msg->scan().angle_max();
  laser_msg.angle_increment = _msg->scan().angle_step();
  laser_msg.time_increment = 0;  // instantaneous simulator scan
  laser_msg.scan_time = 0;  // not sure whether this is correct
  laser_msg.range_min = _msg->scan().range_min();
  laser_msg.range_max = _msg->scan().range_max();
  laser_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            laser_msg.ranges.begin());
  laser_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(),
            _msg->scan().intensities().end(),
            laser_msg.intensities.begin());
  this->pub_queue_->push(laser_msg, this->pub_);
  //ROS_INFO_NAMED("laser", "Published new scan");
}


bool GazeboRosLaser::SetSensorMode(brass_gazebo_plugins::SetLidarMode::Request& req, brass_gazebo_plugins::SetLidarMode::Response& res) {
  ROS_INFO_STREAM("Setting lidar mode to " << std::to_string(req.mode));
  this->sensor_mode_ = req.mode;
  return true;
}

}
