#include <franka_msgs/FrankaState.h>
#include <ros/ros.h>


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"demo_pub_frankastate");
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    auto state = ros::topic::waitForMessage<franka_msgs::FrankaState>(
        "/franka_state_controller/franka_states", nh,  ros::Duration(10));

    double z_position = state->O_T_EE[14]; 
    double x_position = state->O_T_EE[12];
    double y_position = state->O_T_EE[13];
    
    std::cout << "state:" << x_position << ", " << y_position << "," << z_position << std::endl;
    // ros::topic::waitForMessage<franka_msgs::FrankaState>("/franka_state_controller/franka_states", nh, ros::Duration(2));
}