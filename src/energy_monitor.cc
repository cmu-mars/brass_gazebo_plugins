#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <boost/thread/mutex.hpp>

#include <cmath>
#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <kobuki_msgs/MotorPower.h>

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
		ROS_INFO_STREAM("BRASS Energy Monitor loaded");
            }

		~EnergyMonitorPlugin() {
			this->rosNode->shutdown();
		}
	
	private:
		const double SEC_PER_HR = 3600.0;

		// A node use for ROS transport
		std::unique_ptr<ros::NodeHandle> rosNode;

		// ROS subscribers
		ros::Subscriber set_charging_sub;
		ros::Subscriber set_voltage_sub;
		ros::Subscriber get_model_state_sub;
		ros::Subscriber kinect_onoff_sub;
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
		const double charge_rate = 30055.0 / SEC_PER_HR /* mwh / sec */;
		const double multiplier = 5.0; // make things go faster

		const double battery_capacity /* mwh */ = 32560.0;
		
		const double delta_base_FULLSPEED /* mwh / sec */ = 14004.0 /* mwh / hr */ / SEC_PER_HR; 
		const double delta_base_HALFSPEED /* mwh / sec */= 6026.0 / SEC_PER_HR;
		const double delta_base_FULLSPEEDR /* mwh / sec */= 17640 / SEC_PER_HR;
		const double delta_base_HALFSPEEDR /* mwh / sec */=(delta_base_FULLSPEEDR * delta_base_HALFSPEED/delta_base_FULLSPEED) / SEC_PER_HR;
		const double delta_base_STOPPED /* mwh / sec */ = 0.0 / SEC_PER_HR;
		enum Speed { FULLSPEED, FULLSPEEDR, HALFSPEED, HALFSPEEDR, STOPPED };
		Speed speed = FULLSPEED;
		double delta_base_of(Speed speed) {
			switch(speed) {
				case FULLSPEED: return delta_base_FULLSPEED; break;
				case HALFSPEED: return delta_base_HALFSPEED; break;
				case FULLSPEEDR: return delta_base_FULLSPEEDR; break;
				case HALFSPEEDR: return delta_base_HALFSPEEDR; break;
				case STOPPED:   return delta_base_STOPPED; break;
			}
		}

		const double v_FULL_thresh = 0.4;
		const double v_HALF_thresh = 0.01;
		const double z_FULL_thresh = 0.3;
		const double z_HALF_thresh = 0.01;
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

		const double delta_kinect_USED /* mwh / sec */ = 5132.0 / SEC_PER_HR;
		const double delta_kinect_UNUSED /* mwh / sec */ = 250.0 / SEC_PER_HR;
		enum KinectState { USED, UNUSED };
		KinectState kinectState = USED;
		double delta_kinect_of(KinectState kinectState) {
			switch(kinectState) {
				case USED: return delta_kinect_USED; break;
				case UNUSED: return delta_kinect_UNUSED; break;
			}
		}

		double nuc_utilization = 0.0; /* ranges from 0.0 to 100.0 */
		double delta_nuc_of(double nuc_utilization) {
			return (115.28*nuc_utilization + 6894.0) / SEC_PER_HR;
		}

		double cur_charge /* mwh */;

		// used to send voltage sensor values to the robot (a main use case)
		int voltage_of_charge(double charge) {
			double pct = charge / battery_capacity;
			double idx_dbl = pct * (NUM_V_DATA - 1); // -1 is important here (mapping from 0 to num-1), otherwise the index goes out of range for pct = 0 
			int idx_int = round(idx_dbl);
			// The highest voltage is idx 0 in v_data, so need 
			// reverse this when calculating
			int idx_int_r = NUM_V_DATA -1 - idx_int; 
			if (idx_int_r < 0 || idx_int_r > NUM_V_DATA-1)
				gzdbg << "Error: voltage index " << idx_int_r << " out of bounds [0, " << NUM_V_DATA - 1 << "]\n"; 

			return gazebo::v_data[idx_int_r];
		}

		// used to interpret the messed-up MIT-LL input (a crutch)
		// from voltage (what it shouldn't be)
		// to charge (what it should be)
		double charge_of_voltage(int voltage) {
			double pct = percent_of_v[voltage - MIN_VOLTAGE];
			return pct * battery_capacity;
		}

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

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
			  ros::SubscribeOptions::create<std_msgs::Bool>(
				  "/energy_monitor/set_charging",
				  1,
				  boost::bind(&EnergyMonitorPlugin::OnSetChargingMsg, this, _1),
				  ros::VoidPtr(), &this->rosQueue);
			this->set_charging_sub = this->rosNode->subscribe(so);

			ros::SubscribeOptions set_voltage_so =
			  ros::SubscribeOptions::create<std_msgs::Int32>(
				  "/energy_monitor/set_voltage",
				  1,
				  boost::bind(&EnergyMonitorPlugin::OnSetVoltageMsg, this, _1),
				  ros::VoidPtr(), &this->rosQueue);
			this->set_voltage_sub = this->rosNode->subscribe(set_voltage_so);

			ros::SubscribeOptions get_model_state_so = 
				ros::SubscribeOptions::create<geometry_msgs::Twist>(
						"/gazebo/get_model_state",
						1,
						boost::bind(&EnergyMonitorPlugin::OnGetModelState, this, _1),
						ros::VoidPtr(), &this->rosQueue);
			this->get_model_state_sub = this->rosNode->subscribe(get_model_state_so);

			ros::SubscribeOptions kinect_onoff_so = 
				ros::SubscribeOptions::create<std_msgs::String>(
						"/sensor/kinect/onoff",
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

			// Publish a topic
			this->charge_state_pub = this->rosNode->advertise<std_msgs::Float64>(
					"/energy_monitor/energy_level",
					1);


			this->charge_v_pub = this->rosNode->advertise<std_msgs::Int32>(
					"/energy_monitor/voltage",
					1);

			this->motor_power_pub = this->rosNode->advertise<kobuki_msgs::MotorPower>(
					"/mobile_base/commands/motor_power",
					1);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&EnergyMonitorPlugin::QueueThread, this));
#ifdef ENERGY_MONITOR_DEBUG
			gzdbg << "Loaded energy_monitor." << "\n";
#endif
		}

		void UpdateChild() {
			// Update time
			double curr_time = this->world->GetSimTime().Double(); // measured in seconds
			double dt = curr_time - last_time; 
			last_time = curr_time;
		    
			if (charging) {
				cur_charge += charge_rate * multiplier * dt;
			} else {
				double delta_base = delta_base_of(speed);
				double delta_kinect = delta_kinect_of(kinectState);
				double delta_nuc = delta_nuc_of(nuc_utilization);
				double delta_discharging_energy = 
					- (delta_base + delta_kinect + delta_nuc);
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

			// publish voltage
			std_msgs::Int32 v_msg;
			v_msg.data = voltage_of_charge(cur_charge);
			lock.lock();
			this->charge_v_pub.publish(v_msg);
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

		void OnSetVoltageMsg(const std_msgs::Int32ConstPtr &msg) {
			lock.lock();
			auto voltage = msg->data; 
			cur_charge = charge_of_voltage(voltage);
#ifdef ENERGY_MONITOR_DEBUG
			gzdbg << "received voltage " << voltage << "\n";
#endif
			lock.unlock();
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

		void OnKinectOnOffMsg(const std_msgs::StringConstPtr &msg) {
			lock.lock();
			auto s = msg->data.c_str();
			if (strcmp(s, "on") == 0) {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "kinect on" << "\n";
#endif
				kinectState = USED;
			} else if (strcmp(s, "off") == 0) {
#ifdef ENERGY_MONITOR_DEBUG
				gzdbg << "kinect off" << "\n";
#endif
				kinectState = UNUSED;
			} else {
				gzerr << "invalid kinect on/off string: " << s << "\n";
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

