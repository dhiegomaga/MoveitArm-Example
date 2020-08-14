#include <edo_hardware_interface/edo_hardware_interface.h>
#include <driver/edo_driver.h>

#include <random>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edo_hardware_interface");
    ros::NodeHandle nh;
    unsigned num_joints = 7;

    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(-65, 65); // define the range
    
    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    
    edo_driver::edo driver(nh, num_joints);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //edo_hardware_interface::edoHardwareInterface edo_hi(nh);

    ros::Rate loop_rate(0.11);
    std::vector<float> target_angles(7);
    while(ros::ok())
    {
        // Generate some random positions
        target_angles[1] = (float) distr(gen);
        target_angles[2] = (float) distr(gen);

        driver.setJoints(target_angles);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    //ros::spin();
    return 0;
}