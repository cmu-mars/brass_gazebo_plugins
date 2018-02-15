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
 @mainpage
   Desc: GazeboRosCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
*/

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include "std_msgs/Bool.h"
#include "brass_gazebo_plugins/SetCameraMode.h"


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCamera::GazeboRosCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCamera::~GazeboRosCamera()
{
  ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
}

void GazeboRosCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_INFO_STREAM("BRASS Camera Plugin Loading");

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  ROS_INFO_STREAM("BRASS Camera Plugin loaded with mode: " << std::to_string(this->sensor_mode_));
  load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboRosCamera::Advertise, this));

   GazeboRosCameraUtils::Load(_parent, _sdf);
}



void GazeboRosCamera::Advertise() 
{
  this->sensor_mode_srv_ = this->rosnode_->advertiseService("/mobile_base/camera/mode", &GazeboRosCamera::SetCameraMode, this);

  ros::AdvertiseOptions camera_status_ao =
    ros::AdvertiseOptions::create<std_msgs::Bool>("/mobile_base/camera/status", 1,
      boost::bind (&GazeboRosCamera::StatusConnect, this),
      boost::bind (&GazeboRosCamera::StatusDisconnect, this),
      ros::VoidPtr(), NULL);

  this->status_pub_ = this->rosnode_->advertise(camera_status_ao);
}

void GazeboRosCamera::StatusConnect() 
{
  this->status_connect_count_++;
  this->parentSensor_->SetActive(true);
}

void GazeboRosCamera::StatusDisconnect()
{
  this->status_connect_count_--;
}
////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();
if (last_status_update_ + 1 < sensor_update_time_.Float() ) {
    this->last_status_update_ = sensor_update_time_;
    std_msgs::Bool v_msg;
    v_msg.data = this->sensor_mode_;
    this->status_pub_.publish(v_msg);
  }
    if (!this->sensor_mode_) return;

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      // OnNewFrame is triggered at the gazebo sensor <update_rate>
      // while there is also a plugin <updateRate> that can throttle the
      // rate down further (but then why not reduce the sensor rate?
      // what is the use case?).
      // Setting the <updateRate> to zero will make this plugin
      // update at the gazebo sensor <update_rate>, update_period_ will be
      // zero and the conditional always will be true.
      if (sensor_update_time - this->last_update_time_ >= this->update_period_)
      {
        this->PutCameraData(_image, sensor_update_time);
        this->PublishCameraInfo(sensor_update_time);
        this->last_update_time_ = sensor_update_time;
      }
    }
  }
}

bool GazeboRosCamera::SetCameraMode(brass_gazebo_plugins::SetCameraMode::Request& req, brass_gazebo_plugins::SetCameraMode::Response& res) {
  ROS_INFO_STREAM("Setting camera mode to " << std::to_string(req.mode));
  this->sensor_mode_ = req.mode;
  return true;
}
}
