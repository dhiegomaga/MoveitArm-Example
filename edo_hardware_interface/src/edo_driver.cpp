
#include <driver/edo_driver.h>

namespace edo_driver
{
    edo::edo(ros::NodeHandle& nh, unsigned num_joints)
    {
        nh_ = nh;
        this->num_joints = num_joints;
        this->current_joint_positions.resize(num_joints, 0.0);
        this->move_pub = nh_.advertise<edo_core_msgs::MovementCommand>("/machine_move", 1);
        this->sub = nh.subscribe("/joint_positions", 1, &edo::jointPositionsCallback, this );
    }

    std::vector<float> edo::getJoints()
    {
        return current_joint_positions;
    }
    void edo::setJoints( std::vector<float> joint_positions)
    {   
        if(joint_positions.size() != this->num_joints){
            ROS_ERROR("setJoints: Incompatible number of joints called: %d , %d ", joint_positions.size(), this->num_joints);
            return;
        }
        // for debug purposes 
        current_joint_positions = joint_positions;

        move_pub.publish(move_command);
    }

    void edo::jointPositionsCallback(const edo_core_msgs::JointsPositions::ConstPtr& msg){
        if(msg->positions.size() == this->num_joints)
        {
            this->current_joint_positions = msg->positions;
            std::cout<< "CURRENT SIZE:" << std::to_string( this->current_joint_positions.size() ) << std::endl;
            std::cout << "position 0 :" << std::to_string( this->current_joint_positions.at(0) ) << std::endl;
            std::string positions; 
            for(int i = 0; i< this->current_joint_positions.size(); i++){
                positions= positions + std::to_string( this->current_joint_positions.at(i) ) + ", ";
                
            }
            std::cout << "VECTOR: " << positions << std::endl;
        }
        else{
            ROS_ERROR("jointPositionsCallback: Received wrong number of angles");
        }
        
        
    }
}