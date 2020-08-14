#ifndef EDO_DRIVER_H
#define EDO_DRIVER_H

#include <edo_core_msgs/MovementCommand.h>
#include <edo_core_msgs/JointsPositions.h>
#include <edo_core_msgs/JointStateArray.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <string>

namespace edo_driver
{
    class edo
    {
        public:
            edo(ros::NodeHandle& nh, unsigned num_joints);
            std::vector<float> getJoints();
            void setJoints( std::vector<float> joint_positions);
            
        protected:
            ros::NodeHandle nh_;
            ros::Publisher move_pub;
            edo_core_msgs::MovementCommand move_command;
            std::vector<float> current_joint_positions; 
            unsigned num_joints;
            ros::Subscriber sub;

            void jointPositionsCallback(const edo_core_msgs::JointStateArray::ConstPtr&);
    };
}

#endif