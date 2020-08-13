
#include <driver/edo_driver.h>

namespace edo_driver
{
    edo::edo(ros::NodeHandle& nh, unsigned num_joints)
    {
        nh_ = nh;
        current_joint_positions.resize(num_joints, 0.0);
        move_pub = nh_.advertise<edo_core_msgs::MovementCommand>("/machine_move", 1);
    }
    std::vector<double> edo::getJoints()
    {
        
    }
    void edo::setJoints( std::vector<double> joint_positions)
    {   
        
        move_pub.publish(move_command);
    }
}