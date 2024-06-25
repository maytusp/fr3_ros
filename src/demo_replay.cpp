#include <ros/ros.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka_msgs/FrankaState.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
// Function to move the robot to a specified pose
void moveRobot(franka::Robot& robot, const franka_msgs::FrankaState::ConstPtr& state_msg) {
    std::array<double, 16> pose;
    for (int i = 0; i < 16; i++) {
    pose[i] = state_msg->O_T_EE[i];
    }
    robot.control([new_pose](const franka::RobotState&, franka::Duration) -> franka::CartesianPose {
    return franka::MotionFinished(pose);
    });
}
// Function to control the gripper
void controlGripper(franka::Gripper& gripper, const sensor_msgs::JointState::ConstPtr& joint_state) {
if (!joint_state->position.empty()) {
double width = joint_state->position[0] * 2; // Assuming symmetric gripper
gripper.move(width, 0.1);
  }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_replay");
    ros::NodeHandle nh;
    franka::Robot robot("172.16.0.2");
    franka::Gripper gripper("172.16.0.2");
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{0, 47 * M_PI/180, 0, -150 * M_PI/180, 0, 103 * M_PI/180, 45 * M_PI/180}};
    MotionGenerator motion_generator(0.3, q_goal);
    robot.control(motion_generator);

    ros::Subscriber state_sub = nh.subscribe<franka_msgs::FrankaState>("/franka_state_controller/franka_states", 10, boost::bind(moveRobot, boost::ref(robot), _1));
    ros::Subscriber gripper_sub = nh.subscribe<sensor_msgs::JointState>("/franka_gripper/joint_states", 10, boost::bind(controlGripper, boost::ref(gripper), _1));
    ros::spin();
    return 0;
}