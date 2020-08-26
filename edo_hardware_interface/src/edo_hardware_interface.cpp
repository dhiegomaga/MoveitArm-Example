#include <sstream>
#include <sensor_msgs/JointState.h>
#include <edo_hardware_interface/edo_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace edo_hardware_interface
{
    edoHardwareInterface::edoHardwareInterface(ros::NodeHandle& nh, unsigned num_driver_joints) : nh_(nh)
    {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/edo/hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &edoHardwareInterface::update, this);
        this->joints_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 0);
        this->driver = new edo(nh_, num_driver_joints);
    }

    edoHardwareInterface::~edoHardwareInterface() {
        delete this->driver;
    }

    void edoHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/edo/hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();
        
        if (joint_names_.size() == 0)
		{
		    ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}

        for (int i = 0; i < num_joints_; i++){
            ROS_INFO("joint name: %s", joint_names_.at(i).c_str());
        }
        
        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {
            //edocpp::Joint joint = edo.getJoint(joint_names_[i]);

             // Create joint state interface
            JointStateHandle jointStateHandle(joint_names_.at(i), &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // Create position joint interface
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            JointLimits limits;
            SoftJointLimits softLimits;

            getJointLimits(joint_names_.at(i), nh_, limits);

            PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
            position_joint_interface_.registerHandle(jointPositionHandle);

            // Create effort joint interface
            JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(jointEffortHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&effort_joint_interface_);
        registerInterface(&positionJointSoftLimitsInterface);
    }

    void edoHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);

        // Read current joint states from robot
        read();

        // Publish joint angles on /joint_states
        sensor_msgs::JointState current_joint_state; 
        current_joint_state.position = std::vector<double>(num_joints_, 0.0); 
        current_joint_state.velocity = std::vector<double>(num_joints_, 0.0); 
        current_joint_state.effort = std::vector<double>(num_joints_, 0.0);   
        current_joint_state.name = std::vector<std::string>(num_joints_); 
        for (int i = 0; i < num_joints_; i++) {
            current_joint_state.position[i] = this->joint_position_[i];
            current_joint_state.name[i] = this->joint_names_[i];
        }
        this->joints_pub.publish(current_joint_state);

        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void edoHardwareInterface::read() {
        auto driver_joints = this->driver->getJoints();        

        for (int i = 0; i < num_joints_; i++) {
            this->joint_position_[i] = driver_joints[i];
        }
    }

    void edoHardwareInterface::write(ros::Duration elapsed_time) {
        positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

        // Convert from std::vector<double> to std::vector<float>
        std::vector<float> joint_position_command__float(this->joint_position_command_.begin(), this->joint_position_command_.end());
        this->driver->setJoints(joint_position_command__float);
    }
}