#ifndef EDO_DRIVER_H
#define EDO_DRIVER_H

//#include <vector>
#include <edo_core_msgs/MovementCommand.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

namespace edo_driver
{
    class edo
    {
        public:
            edo(ros::NodeHandle& nh, unsigned num_joints);
            std::vector<double> getJoints();
            void setJoints( std::vector<double> joint_positions);

        protected:
            ros::NodeHandle nh_;
            ros::Publisher move_pub;
            edo_core_msgs::MovementCommand move_command;
            std::vector<double> current_joint_positions; 
    };
}

#endif