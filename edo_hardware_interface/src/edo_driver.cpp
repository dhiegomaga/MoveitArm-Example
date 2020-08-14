
#include <driver/edo_driver.h>

namespace edo_driver
{
    edo::edo(ros::NodeHandle& nh, unsigned num_joints)
    {
        nh_ = nh;
        this->num_joints = num_joints;
        this->current_joint_positions.resize(num_joints, 0.0);
        this->move_pub = nh_.advertise<edo_core_msgs::MovementCommand>("/machine_move", 1);
        this->sub = nh.subscribe("/machine_algo_jnt_state", 1, &edo::jointPositionsCallback, this );
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
        //current_joint_positions = joint_positions;
        
        move_command.move_command = 77;
        move_command.move_type = 74;
        move_command.ovr = 70;
        move_command.delay = 0;
        move_command.remote_tool = 0;
        move_command.cartesian_linear_speed = 0;
        move_command.target.data_type = 74;
        move_command.target.cartesian_data.x = 0;
        move_command.target.cartesian_data.y = 0;
        move_command.target.cartesian_data.z = 0;
        move_command.target.cartesian_data.a = 0;
        move_command.target.cartesian_data.e = 0;
        move_command.target.cartesian_data.r = 0;
        move_command.target.joints_mask = 127;
        move_command.target.joints_data = joint_positions;

        move_command.tool.x = 0.0;
        move_command.tool.y = 0.0;
        move_command.tool.z = 0.0;
        move_command.tool.a = 0.0;
        move_command.tool.e = 0.0;
        move_command.tool.r = 0.0;

        move_command.frame.x = 0.0;
        move_command.frame.y = 0.0;
        move_command.frame.z = 0.0;
        move_command.frame.a = 0.0;
        move_command.frame.e = 0.0;
        move_command.frame.r = 0.0;

        move_pub.publish(move_command);
    }

    void edo::jointPositionsCallback(const edo_core_msgs::JointStateArray::ConstPtr& msg){
        auto positions = msg->joints;
        if(positions.size() == this->num_joints)
        {
            //std::string angles_str; 

            // Get each position
            for(int i = 0; i< positions.size(); i++){
                float joint_angle = positions[i].position;
                this->current_joint_positions[i] = joint_angle; // Assign angle

                //angles_str= angles_str + std::to_string( joint_angle ) + ", "; // Debugging
            }
            
            //std::cout << "Current angles: " << angles_str << std::endl;
        }
        else{
            ROS_ERROR("jointPositionsCallback: Received wrong number of angles");
        }
        
        
    }
}