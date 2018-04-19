#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <boost/thread/mutex.hpp>

#include <cmath>
#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <kobuki_msgs/MotorPower.h>
#include "brass_gazebo_plugins/SetCharge.h"
#include "brass_gazebo_plugins/SetCharging.h"

#include "v_data.cc"

#define ENERGY_MONITOR_DEBUG
#define ENERGY_LEVEL_DBG_INTERVAL 5.0

namespace gazebo {
  class EnergyMonitorPlugin : public ModelPlugin {
    public: 
		EnergyMonitorPlugin() : ModelPlugin() {
#ifdef ENERGY_MONITOR_DEBUG
					gzdbg << "Constructed energy_monitor." << "\n";
#endif
            }

		~EnergyMonitorPlugin() {
			this->rosNode->shutdown();
		}
	
	private:
		const double SEC_PER_HR = 3600.0;

		// A node use for ROS transport
		std::unique_ptr<ros::NodeHandle> rosNode;

		// ROS subscribers
		ros::ServiceServer set_charging_srv;
		ros::ServiceServer set_charge_srv;
		ros::Subscriber get_model_state_sub;
		ros::Subscriber kinect_onoff_sub;
		ros::Subscriber lidar_onoff_sub;
		ros::Subscriber headlamp_onoff_sub;

		ros::Subscriber nuc_utilization_sub;

		// A ROS publisher
		ros::Publisher charge_state_pub;
		ros::Publisher charge_v_pub;
		ros::Publisher motor_power_pub;

		// A lock to prevent ticks from happening at the same time as messages.
		boost::mutex lock;

		// A ROS callbackqueue that helps process messages
		ros::CallbackQueue rosQueue;

		// A thread the keeps the running rosQueue
		std::thread rosQueueThread;

		// Charge level
		bool charging;
		double charge_rate = 30055.0 / SEC_PER_HR /* mwh / sec */;
		double multiplier = 1.0; // make things go faster

		double battery_capacity /* mwh */ = 32560.0;
		
		double delta_base_FULLSPEED /* mwh / sec */ = 14004.0 /* mwh / hr */ / SEC_PER_HR; 
		double delta_base_HALFSPEED /* mwh / sec */= 6026.0 / SEC_PER_HR;
		double delta_base_FULLSPEEDR /* mwh / sec */= 17640 / SEC_PER_HR;
		double delta_base_HALFSPEEDR /* mwh / sec */=(delta_base_FULLSPEEDR * delta_base_HALFSPEED/delta_base_FULLSPEED) / SEC_PER_HR;
		double delta_base_SAFESPEED /* mwh / sec */=4000.0 / SEC_PER_HR;
		double delta_base_STOPPED /* mwh / sec */ = 0.0 / SEC_PER_HR;
		enum Speed { FULLSPEED, FULLSPEEDR, HALFSPEED, HALFSPEEDR, SAFESPEED, STOPPED };
		Speed speed = FULLSPEED;
		double delta_base_of(Speed speed) {
			switch(speed) {
				case FULLSPEED: return delta_base_FULLSPEED; break;
				case HALFSPEED: return delta_base_HALFSPEED; break;
				case FULLSPEEDR: return delta_base_FULLSPEEDR; break;
				case HALFSPEEDR: return delta_base_HALFSPEEDR; break;
				case SAFESPEED: return delta_base_SAFESPEED; break;
				case STOPPED:   return delta_base_STOPPED; break;
			}
		}

		double v_FULL_thresh = 0.4;
		double v_HALF_thresh = 0.01;
		double z_FULL_thresh = 0.3;
		double z_HALF_thresh = 0.01;
		double v_SAFE_thresh = 0.07;

		Speed speed_of(double v, double twist_z) {
			double abs_twist_z = abs(twist_z);
			if (abs_twist_z > z_FULL_thresh) {
				return FULLSPEEDR;
			} else if (v > v_FULL_thresh) {
				return FULLSPEED;
			} else if (abs_twist_z > z_HALF_thresh) {
				return HALFSPEEDR;
			} else if (v > v_HALF_thresh) {
				return HALFSPEED;
			}
			else if (v > v_SAFE_thresh) {
				return SAFESPEED;
			}
			else {
				return STOPPED;
			}
/*
			if (v > v_FULL_thresh || abs_twist_z > z_FULL_thresh) {
				return FULLSPEED;
			} else if (v > v_HALF_thresh || abs_twist_z > z_HALF_thresh) {
				return HALFSPEED;
			} else {
				return STOPPED;
			}
*/
		}

		double v_of(double x, double y) {
			return sqrt(pow(x, 2) + pow(y, 2));
		}

		double delta_kinect_USED /* mwh / sec */ = 5132.0 / SEC_PER_HR;
		double delta_kinect_UNUSED /* mwh / sec */ = 250.0 / SEC_PER_HR;
		double delta_kinect_CAMERA_ONLY /*mwh / sec */= 2000.0 / SEC_PER_HR;

		enum KinectState { KINECT_USED, KINECT_UNUSED, CAMERA_ONLY };
		KinectState kinectState = KINECT_USED;
		double delta_kinect_of(KinectState kinectState) {
			switch(kinectState) {
				case KINECT_USED: return delta_kinect_USED; break;
				case KINECT_UNUSED: return delta_kinect_UNUSED; break;
				case CAMERA_ONLY: return delta_kinect_CAMERA_ONLY; break;
			}
		}

		double delta_lidar_USED /* mwh / sec */ = 7132.0 / SEC_PER_HR;
		double delta_lidar_UNUSED /* mwh / sec */ = 250.0 / SEC_PER_HR;

		enum LidarState { LIDAR_USED, LIDAR_UNUSED};
		LidarState lidarState = LIDAR_UNUSED;
		double delta_lidar_of(LidarState lidarState) {
			switch (lidarState) {
				case LIDAR_USED: return delta_lidar_USED; break;
				case LIDAR_UNUSED: return delta_lidar_UNUSED; break;
			}
		}

		double delta_headlamp_USED /* mwh / sec */ = 10000.0 / SEC_PER_HR;
		double delta_headlamp_UNUSED /* mwh / sec */ = 0.0 / SEC_PER_HR;

		enum HeadlampState { LAMP_USED, LAMP_UNUSED};
		HeadlampState headlampState = LAMP_UNUSED;
		double delta_headlamp_of(HeadlampState headlampState) {
			switch (headlampState) {
				case LAMP_USED: return delta_headlamp_USED; break;
				case LAMP_UNUSED: return delta_headlamp_UNUSED; break;
			}
		}

		double nuc_utilization = 0.0; /* ranges from 0.0 to 100.0 */
		double coeff_cpu_utilization = 115.28;
		double const_cpu_utilization = 6894.0;
		double delta_nuc_of(double nuc_utilization) {
			return (coeff_cpu_utilization*nuc_utilization + const_cpu_utilization) / SEC_PER_HR;
		}

		double cur_charge /* mwh */;

		
		// Time management
		double last_time /* s */;
		double last_print_time /* s */ = -1.0;

		// gazebo stuff
		physics::WorldPtr world;
		event::ConnectionPtr updateConnection;

    public: 
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
			this->world = _model->GetWorld();

			last_time = this->world->GetSimTime().Double(); 

#ifdef ENERGY_MONITOR_DEBUG
			gzdbg << "Initial time: " << last_time << "\n";
#endif


			if (_sdf->HasElement("battery_capacity")) {
				battery_capacity = _sdf->GetElement("battery_capacity")->Get<double>();
			}
			if (_sdf->HasElement("coeff_fullspeed")) {
				delta_base_FULLSPEED = _sdf->GetElement("coeff_fullspeed")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_halfspeed")) {
				delta_base_HALFSPEED = _sdf->GetElement("coeff_halfspeed")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_fullspeed_rot")) {
				delta_base_FULLSPEEDR = _sdf->GetElement("coeff_fullspeed_rot")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_halfspeed_rot")) {
				delta_base_HALFSPEEDR = _sdf->GetElement("coeff_halfspeed_rot")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_safespeed")) {
				delta_base_SAFESPEED = _sdf->GetElement("coeff_safespeed")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("fullspeed_threshold")) {
				v_FULL_thresh = _sdf->GetElement("fullspeed_threshold")->Get<double>();
			}
			if (_sdf->HasElement("halfspeed_threshold")) {
				v_HALF_thresh = _sdf->GetElement("halfspeed_threshold")->Get<double>();
			}
			if (_sdf->HasElement("safespeed_threshold")) {
				v_SAFE_thresh = _sdf->GetElement("safespeed_threshold")->Get<double>();
			}
			if (_sdf->HasElement("fullspeed_rot_threshold")) {
				z_FULL_thresh = _sdf->GetElement("fullspeed_rot_threshold")->Get<double>();
			}
			if (_sdf->HasElement("halfspeed_rot_threshold")) {
				z_HALF_thresh = _sdf->GetElement("halfspeed_rot_threshold")->Get<double>();
			}
			if (_sdf->HasElement("coeff_kinect_on")) {
				delta_kinect_USED = _sdf->GetElement("coeff_kinect_on")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_kinect_off")) {
				delta_kinect_UNUSED = _sdf->GetElement("coeff_kinect_off")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_lidar_on")) {
				delta_lidar_USED = _sdf->GetElement("coeff_lidar_on")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_kinect_camera_only")) {
				delta_kinect_CAMERA_ONLY = _sdf->GetElement("coeff_kinect_camera_only")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_lidar_off")) {
				delta_lidar_UNUSED = _sdf->GetElement("coeff_lidar_off")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_headlamp_on")) {
				delta_headlamp_USED = _sdf->GetElement("coeff_headlamp_on")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_headlamp_off")) {
				delta_headlamp_UNUSED = _sdf->GetElement("coeff_headlamp_off")->Get<double>() / SEC_PER_HR;
			}
			if (_sdf->HasElement("coeff_cpu_utilization")) {
				coeff_cpu_utilization = _sdf->GetElement("coeff_cpu_utilization")->Get<double>();
			}
			if (_sdf->HasElement("const_cpu_utilization")) {
				const_cpu_utilization = _sdf->GetElement("const_cpu_utilization")->Get<double>();
			}


			cur_charge = battery_capacity;
			charging = true;

			// Register callback for every simulation tick
			this->updateConnection = event::Events::ConnectWorldUpdateBegin( 
					boost::bind(&EnergyMonitorPlugin::UpdateChild, this)); 

			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "energy_monitor_client",
				  ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("energy_monitor_client"));

			this->set_charging_srv =
				this->rosNode->advertiseService(_model->GetName() + "/set_charging",
					&EnergyMonitorPlugin::SetCharging, this);
			this->set_charge_srv =
				this->rosNode->advertiseService(_model->GetName() + "/set_charge",
					&EnergyMonitorPlugin::SetCharge, this);

			// // Create a named topic, and subscribe to it.
			// ros::SubscribeOptions so =
			//   ros::SubscribeOptions::create<std_msgs::Bool>(
			// 	  "/energy_monitor/set_charging",
			// 	  1,
			// 	  boost::bind(&EnergyMonitorPlugin::OnSetChargingMsg, this, _1),
			// 	  ros::VoidPtr(), &this->rosQueue);
			// this->set_charging_sub = this->rosNode->subscribe(so);

			// ros::SubscribeOptions set_voltage_so =
			//   ros::SubscribeOptions::create<std_msgs::Int32>(
			// 	  "/energy_monitor/set_voltage",
			// 	  1,
			// 	  boost::bind(&EnergyMonitorPlugin::OnSetVoltageMsg, this, _1),
			// 	  ros::VoidPtr(), &this->rosQueue);
			// this->set_voltage_sub = this->rosNode->subscribe(set_voltage_so);

			ros::SubscribeOptions get_model_state_so = 
				ros::SubscribeOptions::create<geometry_msgs::Twist>(
						"/gazebo/get_model_state",
						1,
						boost::bind(&EnergyMonitorPlugin::OnGetModelState, this, _1),
						ros::VoidPtr(), &this->rosQueue);
			this->get_model_state_sub = this->rosNode->subscribe(get_model_state_so);

			ros::SubscribeOptions kinect_onoff_so = 
				ros::SubscribeOptions::create<std_msgs::Int8>(
						"/mobile_base/kinect/status",
						1,
						boost::bind(&EnergyMonitorPlugin::OnKinectOnOffMsg, this, _1),
						ros::VoidPtr(), &this->rosQueue);
			this->kinect_onoff_sub = this->rosNode->subscribe(kinect_onoff_so);

			ros::SubscribeOptions nuc_utilization_so = 
				ros::SubscribeOptions::create<std_msgs::Float64>(
						"/energy_monitor/set_nuc_utilization",
						1,
						boost::bind(&EnergyMonitorPlugin::OnNucUtilizationMsg, this, _1),
						ros::VoidPtr(), &this->rosQueue);
			this->nuc_utilization_sub = this->rosNode->subscribe(nuc_utilization_so);

			ros::SubscribeOptions lidar_onoff_so = 
				ros::SubscribeOptions::create<std_msgs::Bool>(
					"/mobile_base/lidar/status",
					1,
					boost::bind(&EnergyMonitorPlugin::OnLidarOnOffMsg, this, _1),
					ros::VoidPtr(), &this->rosQueue);
			this->lidar_onoff_sub = this->rosNode->subscribe(lidar_onoff_so);

			ros::SubscribeOptions headlamp_onoff_so = 
					ros::SubscribeOptions::create<std_msgs::Bool>(
					"/mobile_base/headlamp/status",
					1,
					boost::bind(&EnergyMonitorPlugin::OnHeadlampOnOffMsg, this, _1),
					ros::VoidPtr(), &this->rosQueue);
			this->headlamp_onoff_sub = this->rosNode->subscribe(headlamp_onoff_so);

			// Publish a topic
			this->charge_state_pub = this->rosNode->advertise<std_msgs::Float64>(
					"/energy_monitor/energy_level",
					1);


			this->charge_v_pub = this->rosNode->advertise<std_msgs::Float64>(
					"/energy_monitor/charge",
					1);

			this->motor_power_pub = this->rosNode->advertise<kobuki_msgs::MotorPower>(
					"/mobile_base/commands/motor_power",
					1);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&EnergyMonitorPlugin::QueueThread, this));
			ROS_INFO_STREAM("BRASS Energy Monitor loaded, battery_capacity=" << battery_capacity);

#ifdef ENERGY_MONITOR_DEBUG
			gzdbg << "Loaded energy_monitor." << "\n";
#endif
		}

		void UpdateChild() {
			// Update time
			double curr_time = this->world->GetSimTime().Double(); // measured in seconds
			double dt = curr_time - last_time; 
			if (dt < 1.0) return;
			last_time = curr_time;
		    

			if (charging) {
				cur_charge += charge_rate * multiplier * dt;
			} else {
				double delta_base = delta_base_of(speed);
				double delta_kinect = delta_kinect_of(kinectState);
				double delta_nuc = delta_nuc_of(nuc_utilization);
				double delta_headlamp = delta_headlamp_of(headlampState);
				double delta_lidar = delta_lidar_of(lidarState);
				double delta_discharging_energy = 
					- (delta_base + delta_kinect + delta_nuc + delta_headlamp + delta_lidar);
				cur_charge += multiplier * delta_discharging_energy * dt;
			}

			if (cur_charge <= 0.0) {
				cur_charge = 0.0;
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "out of power\n";
#endif
				// turn off motor
				kobuki_msgs::MotorPower power_msg;
				power_msg.state = 0;
				lock.lock();
				this->motor_power_pub.publish(power_msg);
				lock.unlock();
			} else if (cur_charge >= battery_capacity) {
				cur_charge = battery_capacity;
			}
			
			if ((curr_time - last_print_time) >= ENERGY_LEVEL_DBG_INTERVAL) {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "current charge (" << (charging?"c":"d") << "): " << cur_charge << "\n";
#endif
				last_print_time = curr_time;
			}

			// publish charge state
			std_msgs::Float64 msg;
			msg.data = cur_charge;
			lock.lock();
			this->charge_state_pub.publish(msg);
			lock.unlock();
		}

		// Handle an incoming message from ROS
		void OnSetChargingMsg(const std_msgs::BoolConstPtr &_msg)
		{
			lock.lock();
			charging = _msg->data;
/*#ifdef ENERGY_MONITOR_DEBUG
			gzdbg << "received message" << charging << "\n";
#endif*/
			lock.unlock();
		}

		void OnSetChargeMsg(const std_msgs::Float64 &msg) {
// 			lock.lock();
// 			auto charge = msg->data; 
// 			cur_charge = charge;
// #ifdef ENERGY_MONITOR_DEBUG
// 			gzdbg << "received voltage " << charge << "\n";
// #endif
// 			lock.unlock();
		}

		bool SetCharge(brass_gazebo_plugins::SetCharge::Request& req,
						brass_gazebo_plugins::SetCharge::Response& res) {
			lock.lock();
			auto charge = req.charge;
			cur_charge = charge;
			res.result = true;
#ifdef ENERGY_MONITOR_DEBUG
			gzdbg << "received charge " << charge << "\n";
#endif
			lock.unlock();
			return true;
		}

		bool SetCharging(brass_gazebo_plugins::SetCharging::Request& req,
						 brass_gazebo_plugins::SetCharging::Response& res) {
			lock.lock();
			charging = req.charging;
			lock.unlock();
			res.result = true;
			return true;
		}

		void OnGetModelState(const geometry_msgs::TwistConstPtr &twist) {
			lock.lock();
			double x = twist->linear.x;
			double y = twist->linear.y;
			double z = twist->angular.z;
			double v = v_of(x, y);
			speed = speed_of(v, z);
#ifdef ENERGY_MONITOR_DEBUG
			gzdbg << "x, y, z: " << x << ", " << y << ", " << z << "\n";
#endif
			lock.unlock();
		}

		void OnKinectOnOffMsg(const std_msgs::Int8ConstPtr &msg) {
			lock.lock();
			auto s = msg->data;
			if (s == 1) {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "kinect on" << "\n";
#endif
				kinectState = KINECT_USED;
			} 
			else if (s == 2) {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "kinect camera only" << "\n";
#endif
				kinectState = CAMERA_ONLY;
			}
			else {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "kinect off" << "\n";
#endif
				kinectState = KINECT_UNUSED;
			}
			lock.unlock();
		}

		void OnLidarOnOffMsg(const std_msgs::BoolConstPtr &msg) {
			lock.lock();
			auto on = msg->data;
			if (on) {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "lidar on " << "\n";
#endif
				lidarState = LIDAR_USED;
			}
			else {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "lidar off " << "\n";
#endif
				lidarState = LIDAR_UNUSED;
			}
			lock.unlock();
		}

		void OnHeadlampOnOffMsg(const std_msgs::BoolConstPtr &msg) {
			lock.lock();
			auto on = msg->data;
			if (on) {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "Headlamp on" << "\n";
#endif
				headlampState = LAMP_USED;
			}
			else {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "headlamp off " << "\n";
#endif
				headlampState = LAMP_UNUSED;
			}
			lock.unlock();
		}

		void OnNucUtilizationMsg(const std_msgs::Float64ConstPtr &msg) {
			lock.lock();
			auto s = msg->data;
			nuc_utilization = s;
/*#ifdef ENERGY_MONITOR_DEBUG
			gzdbg << "nuc utilization " << s << "\n";
#endif*/
			lock.unlock();
		}

		/// ROS helper function that processes messages
		private: void QueueThread()
		{
		  static const double timeout = 0.01;
		  while (this->rosNode->ok())
		  {
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		  }
		}
  };
  GZ_REGISTER_MODEL_PLUGIN(EnergyMonitorPlugin)
}

