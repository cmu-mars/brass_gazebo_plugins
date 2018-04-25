#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Bool.h"
#include <ignition/math/Pose3.hh>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/ModelState.h"
#include "brass_gazebo_plugins/ToggleHeadlamp.h"

namespace gazebo
{
    class ControlLight : public ModelPlugin {
        private:    
            // Pointer to the model
            physics::ModelPtr mModel;

            // Pointer to the update event connection
            event::ConnectionPtr mUpdateConnection;

            // A node use for ROS transport
            std::unique_ptr<ros::NodeHandle> mRosNode;

            // A ROS subscriber
            ros::Subscriber mRosSub;

            ros::ServiceServer mRosSrv;

            // A ROS callbackqueue that helps process messages
            ros::CallbackQueue mRosQueue;

            // A thread the keeps running the mRosQueue
            std::thread mRosQueueThread;

            // Whether to show the light or not
            bool mShowLight = false;

            // Pointer to the light in the world
            physics::LightPtr mLightPtr_f;
            physics::LightPtr mLightPtr_b;

            // Name of the light
            std::string mLightName = "Turtlebot_headlamp";

            // Which model to attach the light
            std::string mAttchedModel = "mobile_base";

            // Height of the light
            double mHeightOffset = 1;

            // ROS topic for this plugin
            std::string mRosTopic = "/toggle_headlamp";

            // Debug message flag for deeper debug
            bool mDebugMore = false;

            ros::Publisher mStatusPub;

            double mLastUpdateTime;
            physics::WorldPtr world;

            physics::LinkPtr link;
            math::Pose* pose_front;
            math::Pose* pose_back;

        public:
            ControlLight(): ModelPlugin() {
                ROS_INFO_STREAM("BRASS Headlamp loaded");
                pose_front = new math::Pose(.05, 0, .6,
                            1.54,
                            0,
                            -1.54);
                pose_back = new math::Pose(-.05, 0, .6, 1.54, 0, 1.54);
            }

            ~ControlLight() {
                this->mRosNode->shutdown();
                this->mRosNode.reset();
            }

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                // Store the pointer to the model
                this->mModel = _parent; 
                this->world = _parent->GetWorld();
                ROS_INFO_STREAM("BRASS Headlamp attached to model " << this->mModel/*->GetName().c_str()*/);
                
                link = mModel->GetLink("base_link");
                if (!link) {
                    ROS_INFO_STREAM("Link: camera360_link not found!");
                }
                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ControlLight::OnUpdate, this, _1));

                this->mRosNode.reset (new ros::NodeHandle(this->mModel->GetName() + "/headlamp"));
                ROS_INFO_STREAM("Topic: " << this->mModel->GetName() << mRosTopic);
                // Initialize ros, if it has not already bee initialized.


                // Create a named topic, and subscribe to it.
                // Command line to send the message
                // rostopic pub /ground_plane_0/toggle_light std_msgs/Empty
                // ros::SubscribeOptions so =
                //     ros::SubscribeOptions::create<std_msgs::Bool>(
                //     "/" + this->mModel->GetName() + mRosTopic, 1,
                //     boost::bind(&ControlLight::OnRosMsg, this, _1),
                //     ros::VoidPtr(), &this->mRosQueue);
       
                // this->mRosSub = this->mRosNode->subscribe(so);
                // ros::ServiceServer headlamp = 
                //     this->mRosNode->advertiseService<brass_gazebo_plugins::ToggleHeadlamp::Request,brass_gazebo_plugins::ToggleHeadlamp::Response>
                //                             ("/gazebo/toggle_headlamp", 
                //                             boost::bind(&ControlLight::ToggleHeadlamp, this, 
                //                             (brass_gazebo_plugins::ToggleHeadlamp::Request& )_1,
                //                             (brass_gazebo_plugins::ToggleHeadlamp::Response& )_2));
                ros::ServiceServer headlamp =
                    this->mRosNode->advertiseService("/mobile_base/headlamp",
                                    &ControlLight::ToggleHeadlamp, this);

                this->mRosSrv = headlamp;

                this->mStatusPub = this->mRosNode->advertise<std_msgs::Bool>("/mobile_base/headlamp/status",1);

                // Spin up the queue helper thread.
                this->mRosQueueThread =
                    std::thread(std::bind(&ControlLight::QueueThread, this));
                mLastUpdateTime =  this->world->GetSimTime().Double();
            }

            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & /*_info*/) {
	            if (mShowLight && mLightPtr_f != NULL && mLightPtr_b != NULL) {
                    mLightPtr_f->SetWorldPose(*pose_front + mModel->GetWorldPose());
                    mLightPtr_b->SetWorldPose(*pose_back + mModel->GetWorldPose());
                    // math::Pose poses [2];
                    // GetRobotLocation(poses);
                    // mLightPtr_f->SetWorldPose(poses[0].Ign());
                    // mLightPtr_b->SetWorldPose(poses[1].Ign());
                    
	            }
                double time = this->world->GetSimTime().Double();
                if (mLastUpdateTime+1 < time) {
                    mLastUpdateTime = time;
                    std_msgs::Bool v_msg;
                    v_msg.data = mShowLight;
                    this->mStatusPub.publish(v_msg);
                }
    
            }

            bool ToggleHeadlamp(brass_gazebo_plugins::ToggleHeadlamp::Request& req,
                                brass_gazebo_plugins::ToggleHeadlamp::Response& res) {
                mShowLight = req.enablement;
                res.result = this->doToggle(mShowLight);
                return res.result;
            }

            bool doToggle(bool mShowLight) {
                ROS_INFO_STREAM("Toggling the Headlamp(" << mShowLight << ")");
                physics::WorldPtr worldPtr = this->mModel->GetWorld();
            
                if (mShowLight) {
                    gzdbg << "Adding Light\n";

                    sdf::SDF point;

                    // TODO make values custmizable e.g., use mLightName
                    point.SetFromString("<?xml version='1.0' ?>\
                        <sdf version='1.6'>\
                        <!-- Light Source -->\
                        <light name='Turtlebot_headlamp_front' type='spot'>\
                                <pose>1 1 1 0 0 0</pose>\
                                <diffuse>1 1 1 1</diffuse>\
                                <specular>1 1 1 1</specular>\
                                <attenuation>\
                                    <range>5</range>\
                                    <constant>0.25</constant>\
                                    <linear>0.0</linear>\
                                    <quadratic>0.0</quadratic>\
                                </attenuation>\
                                <cast_shadows>0</cast_shadows>\
                                <spot>\
                                    <inner_angle>1.54</inner_angle>\
                                    <outer_angle>1.54</outer_angle>\
                                    <falloff>1</falloff>\
                                </spot>\
                        </light>\
                        </sdf>");

                    sdf::ElementPtr light = point.Root()->GetElement("light");
                    msgs::Light msg = gazebo::msgs::LightFromSDF(light);
        
                    transport::NodePtr node(new transport::Node());
                    node->Init(worldPtr->GetName());

                    transport::PublisherPtr lightPub 
                        = node->Advertise<msgs::Light>("~/factory/light");
                    lightPub->Publish(msg);

                    sdf::SDF point2;
                    point2.SetFromString("<?xml version='1.0' ?>\
                        <sdf version='1.6'>\
                        <!-- Light Source -->\
                        <light name='Turtlebot_headlamp_back' type='spot'>\
                                <pose>1 1 1 0 0 0</pose>\
                                <diffuse>1 1 1 1</diffuse>\
                                <specular>1 1 1 1</specular>\
                                <attenuation>\
                                    <range>5</range>\
                                    <constant>0.25</constant>\
                                    <linear>0.0</linear>\
                                    <quadratic>0.0</quadratic>\
                                </attenuation>\
                                <cast_shadows>0</cast_shadows>\
                                <spot>\
                                    <inner_angle>1.54</inner_angle>\
                                    <outer_angle>1.54</outer_angle>\
                                    <falloff>1</falloff>\
                                </spot>\
                        </light>\
                        </sdf>");
                    light = point2.Root()->GetElement("light");
                    msg = gazebo::msgs::LightFromSDF(light);        
                    lightPub->Publish(msg);
                    while ((mLightPtr_f = worldPtr->Light(mLightName+"_front")) == NULL) {
                        sleep(0.5);
                    }

                    assert(mLightPtr_f != NULL);
                    mLightPtr_f->PlaceOnEntity(mAttchedModel);
                    while ((mLightPtr_b = worldPtr->Light(mLightName+"_back")) == NULL) {
                        sleep(0.5);
                    }
                    assert(mLightPtr_b != NULL);
                    mLightPtr_b->PlaceOnEntity(mAttchedModel);

                } else if (mLightPtr_f != NULL) {
                    gzdbg << "Deleting Light\n";

                    // TODO Explore these APIs rather than deleting the model
                    // mLightPtr->ProcessMsg(msg);
                    // mLightPtr->UpdateParameters(light);
                    // TODO ideally we should not delete the model
                    // Instead tweak the diffusion parameter
                    worldPtr->RemoveModel(mLightName + "_front");
                    worldPtr->RemoveModel(mLightName + "_back");
                    mLightPtr_f = NULL;
                    mLightPtr_b = NULL;
                } else {
                    return false;
                }

                gzdbg << "Light count = " << worldPtr->Lights().size();
                return true;
            }

            // Handle an incoming message from ROS
            // TODO BUG consitency issues between GUI 
            // and command line control of light
            void OnRosMsg(const std_msgs::BoolConstPtr &_msg) {
                 ROS_INFO_STREAM("Entered ControlLight::OnRosMessage");
	
                 this->doToggle(_msg->data);
            }
 

        private:
 
 
            
            /// ROS helper function that processes messages
            void QueueThread() {
                static const double timeout = 0.01;
                while (this->mRosNode->ok()) {
                    this->mRosQueue.callAvailable(ros::WallDuration(timeout));
                }
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ControlLight)
}

