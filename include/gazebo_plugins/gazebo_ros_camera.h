/*
 * Copyright 2012 Open Source Robotics Foundation
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
 * Desc: A dynamic controller plugin that publishes ROS image_raw
 *    camera_info topic for generic camera sensor.
*/

#ifndef GAZEBO_ROS_CAMERA_HH
#define GAZEBO_ROS_CAMERA_HH

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include "brass_gazebo_plugins/SetCameraMode.h"

namespace gazebo
{
  class GazeboRosCamera : public CameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosCamera();

    /// \brief Destructor
    public: ~GazeboRosCamera();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

        private: event::ConnectionPtr load_connection_;

     protected: void Advertise();

    protected: bool sensor_mode_=true;
    private: ros::ServiceServer sensor_mode_srv_;
    protected: bool SetCameraMode(brass_gazebo_plugins::SetCameraMode::Request& req, brass_gazebo_plugins::SetCameraMode::Response& res);

    protected: ros::Publisher status_pub_;
    protected: void StatusConnect();
    protected: void StatusDisconnect();
    private: int status_connect_count_ = 0;
    private: common::Time last_status_update_;
  };
}
#endif

