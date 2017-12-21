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
            physics::LightPtr mLightPtr;

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

        public:
            ControlLight(): ModelPlugin() {
                ROS_INFO_STREAM("BRASS Headlamp loaded");
            }

            ~ControlLight() {
                this->mRosNode->shutdown();
                this->mRosNode.reset();
            }

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                // Store the pointer to the model
                this->mModel = _parent; 
                ROS_INFO_STREAM("BRASS Headlamp attached to model " << this->mModel/*->GetName().c_str()*/);
            
      
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

                // Spin up the queue helper thread.
                this->mRosQueueThread =
                    std::thread(std::bind(&ControlLight::QueueThread, this));
            }

            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & /*_info*/) {
	            if (mShowLight && mLightPtr != NULL) {
                    mLightPtr->SetWorldPose((GetRobotLocation()).Ign());
                    // TODO explore this APIs.
                    //mLightPtr->PlaceOnEntity(mAttchedModel);
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
                        <light name='Turtlebot_headlamp' type='spot'>\
                                <pose>1 1 1 0 0 0</pose>\
                                <diffuse>0.5 0.5 0.5 1</diffuse>\
                                <specular>0.1 0.1 0.1 1</specular>\
                                <attenuation>\
                                    <range>5</range>\
                                    <constant>0.5</constant>\
                                    <linear>0.1</linear>\
                                    <quadratic>0.03</quadratic>\
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
                
                    while ((mLightPtr = worldPtr->Light(mLightName)) == NULL) {
                        sleep(0.5);
                    }

                    assert(mLightPtr != NULL);
                    mLightPtr->PlaceOnEntity(mAttchedModel);
                } else if (mLightPtr != NULL) {
                    gzdbg << "Deleting Light\n";

                    // TODO Explore these APIs rather than deleting the model
                    // mLightPtr->ProcessMsg(msg);
                    // mLightPtr->UpdateParameters(light);
                    // TODO ideally we should not delete the model
                    // Instead tweak the diffusion parameter
                    worldPtr->RemoveModel(mLightName);
                    mLightPtr = NULL;
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
            /* // TODO Customize sdf string
            private std::string GetSDFForLight() {
                math::Pose pose = GetRobotLocation();
                std::string pose_x = to_string(pose.position.x);
                std::string pose_y = to_string(pose.position.y);
                std::string pose_z = to_string(pose.position.z + 1);
                std::string orient_y = to_string(pose.orientation.y);
                std::string orient_z = to_string(pose.orientation.z);
                std::string orient_w = to_string(pose.orientation.w);

                std::string pose_str = pose_x + " " + pose_y
                    + pose_z + " " + orient_y
                    + orient_z + " " + orient_w;

                std::string light_sdf = "<?xml version='1.0' ?>\
			            <sdf version='1.6'>\
  			            <!-- Light Source -->\
  			            <light name='Turtlebot_light' type='point'>\
      				            <pose>" + pose_str + "</pose>\
      				            <diffuse>0.5 0.5 0.5 1</diffuse>\
      				            <specular>0.1 0.1 0.1 1</specular>\
     				            <attenuation>\
        				            <range>20</range>\
        				            <constant>0.5</constant>\
        				            <linear>0.01</linear>\
        				            <quadratic>0.001</quadratic>\
      				            </attenuation>\
      				            <cast_shadows>0</cast_shadows>\
  			            </light>\
			            </sdf>";
                printf("pose_str = %s\n", pose_str.c_str());
            }*/

        private:

            void QuatToEuler(const double x, const double y, const double z, const double w, double *rotx,  double *roty, double *rotz)
            {
                double sqw;
                double sqx;
                double sqy;
                double sqz;
                
                double rotxrad;
                double rotyrad;
                double rotzrad;
                
                sqw = w * w;
                sqx = x * x;
                sqy = y * y;
                sqz = z * z;
                
                rotxrad = (double)atan2l(2.0 * ( y * z + x * w ) , ( -sqx - sqy + sqz + sqw ));
                rotyrad = (double)asinl(-2.0 * ( x * z - y * w ));
                rotzrad = (double)atan2l(2.0 * ( x * y + z * w ) , (  sqx - sqy - sqz + sqw ));
                
                *rotx = rotxrad;
                *roty = rotyrad;
                *rotz = rotzrad;

                
                return;
            }
 
            // Returns the robot location.
            math::Pose GetRobotLocation(void) {
                    geometry_msgs::Pose pose;
                    ros::ServiceClient gms_c 
                            = this->mRosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
            
                    gazebo_msgs::GetModelState getmodelstate;
                    getmodelstate.request.model_name = mAttchedModel;
                    gms_c.call(getmodelstate);
	            
                    double light_height = getmodelstate.response.pose.position.z + mHeightOffset;
 
                    if (mDebugMore) { 
                        printf("p.x = %f\n", getmodelstate.response.pose.position.x);
	                    printf("p.y = %f\n", getmodelstate.response.pose.position.y);
	                    printf("p.z = %f\n", getmodelstate.response.pose.position.z);
	                    printf("o.x = %f\n", getmodelstate.response.pose.orientation.x);
	                    printf("o.y = %f\n", getmodelstate.response.pose.orientation.y);
	                    printf("o.z = %f\n", getmodelstate.response.pose.orientation.z);
	                    printf("o.w = %f\n", getmodelstate.response.pose.orientation.w);
                    }

                    double p, y = 0.0;
                    double r = 0;
                    QuatToEuler(getmodelstate.response.pose.orientation.x, 
                        getmodelstate.response.pose.orientation.y,
                        getmodelstate.response.pose.orientation.z,
                        getmodelstate.response.pose.orientation.w,
                        &r, &p, &y);


                    math::Pose light_pose(getmodelstate.response.pose.position.x, 
                            getmodelstate.response.pose.position.y,
                            light_height,
                            1.54,
                            0,
                            y - 1.54);

                    return light_pose;
            }
            
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

