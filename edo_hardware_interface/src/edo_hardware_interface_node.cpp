#include <edo_hardware_interface/edo_hardware_interface.h>
#include <driver/edo_driver.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edo_hardware_interface");
    ros::NodeHandle nh;
    
    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    
    edo_driver::edo driver(nh, 6);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //edo_hardware_interface::edoHardwareInterface edo_hi(nh);

    ros::Rate loop_rate(1);
    std::vector<float> v(6);
    while(ros::ok())
    {
        driver.setJoints(v);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    //ros::spin();
    return 0;
}