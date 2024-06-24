#include <math.h>
#include <ros/ros.h>
#include <cmath>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// #include "franka_hw/franka_hw.h"

#include <franka_msgs/FrankaState.h>
#include <franka_gripper/franka_gripper.h>
#include <franka/exception.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <actionlib/client/simple_action_client.h>

// #include <geometric_shapes/shapes.h>
// #include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

#include "openvla_real_expr/openvla_action_srv.h"
#include "openvla_real_expr/openvla_instr_srv.h"
#include <openvla_real_expr/openvla_action_srvRequest.h>
#include <openvla_real_expr/openvla_action_srvResponse.h>



#ifdef WIN32
  #define COLOR_NORMAL ""
  #define COLOR_RED ""
  #define COLOR_GREEN ""
#else
  #define COLOR_NORMAL "\033[0m"
  #define COLOR_RED "\033[31m"
  #define COLOR_GREEN "\033[32m"
#endif

using namespace std;
// using namespace pcl;
// using namespace vivspace;

static const string prefix_param("/home/rishabh/Robot/Octo_robot/ros_ws/src/openvla_real_expr/demo_test_action/");
static bool use_pause=true;

char WaitUserConfirm(const string str_message="[Enter]", const string str_done="[OK]")
{
    char ret;    
    if( use_pause )
    {
        std::cout << COLOR_RED << str_message << COLOR_NORMAL;
        //char ret = std::cin.get();
        std::cin >> ret;
        std::cin.ignore(INT_MAX,'\n');
        cout << COLOR_GREEN << str_done << COLOR_NORMAL << endl;
    }
    else
    {
        ros::Duration(1).sleep();
        ret = 0;
    }
    return ret;
}

void Quaterion2Euler(geometry_msgs::Quaternion &quat, double &angle_x, double &angle_y, double &angle_z)
{
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;
    double w = quat.w;
    
    double t0, t1, t2, t3, t4;

    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + y * y);

    t2 =  2.0 * (w * y - z * x);
    if( t2 >  1.0 ) t2 =  1.0;
    if( t2 < -1.0 ) t2 = -1.0;   

    t3 = 2.0 * (w * z + x * y);
    t4 = 1.0 - 2.0 * (y * y + z * z);

    angle_x = atan2(t0, t1);
    angle_y = asin(t2);
    angle_z = atan2(t3, t4);
    cout << "angle_z: ---- yaw : " << angle_z << endl; 
}

bool quaternion_similarity(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
	double SIM_THRESHOLD = 0.001;
	auto q1 = p1.orientation;
	auto q2 = p2.orientation;

	Eigen::Quaternionf eq1;
	eq1.x()=q1.x;
	eq1.y()=q1.y;
	eq1.z()=q1.z;
	eq1.w()=q1.w;
	Eigen::Quaternionf eq2;
	eq2.x()=q2.x;
	eq2.y()=q2.y;
	eq2.z()=q2.z;
	eq2.w()=q2.w;

	eq1.normalize();
	eq2.normalize();

	if( std::abs ( std::abs(eq1.dot(eq2)) -1 ) < SIM_THRESHOLD  )
	{
		ROS_WARN_STREAM("Orientations same.");
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("Orientations differ.");
		return false;
	}
}

void move_gripper(float signal, actionlib::SimpleActionClient<franka_gripper::MoveAction> &franka_action){
    ROS_INFO("Waiting for action server to start.");
    franka_action.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    franka_gripper::MoveGoal goal;

    if(signal>=0.5){
        // open the gripper
        goal.speed = 0.1;
        goal.width = 0.08;
        franka_action.sendGoal(goal);
        // ros::Duration(1).sleep();
    }
    else{
        // close the gripper
        goal.speed = 0.1;
        goal.width = 0.0;
        franka_action.sendGoal(goal);
        // ros::Duration(1).sleep();
    }
}

void pickup_gripper(actionlib::SimpleActionClient<franka_gripper::GraspAction> &franka_action_grasp){
    ROS_INFO("Waiting for action server to start...");
    franka_action_grasp.waitForServer();
    ROS_INFO("Action server started, sending grasp goal.");
    franka_gripper::GraspGoal grasp;
    grasp.width = 0.06;  // Attempt to grasp an object with 0.04m width
    grasp.epsilon.inner = 0.005;  // Tolerance for inner grasping
    grasp.epsilon.outer = 0.005;  // Tolerance for outer grasping
    grasp.speed = 0.1;  // Speed of the grasping operation
    grasp.force = 30;
    franka_action_grasp.sendGoal(grasp);
    // ros::Duration(1).sleep();
}


bool execute_trajectory(moveit::planning_interface::MoveGroupInterface::Plan& myplan){
	bool success = false;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac_("/position_joint_trajectory_controller/follow_joint_trajectory", true);
	control_msgs::FollowJointTrajectoryActionGoal trajectory_goal;
	trajectory_ac_.waitForServer();

	trajectory_goal.goal.trajectory = myplan.trajectory_.joint_trajectory;
	trajectory_ac_.sendGoal(trajectory_goal.goal);
	success = trajectory_ac_.waitForResult();
	if(success){
		ROS_WARN_STREAM("actionlib: success");
	}else{
		ROS_WARN_STREAM("actionlib: failed");
	}
		auto result = trajectory_ac_.getResult();
		ROS_WARN_STREAM("execute_trajectory result:"<<result->error_code);
	
	return success;
}

void move_franka(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& myplan, float roll_delta, float pitch_delta, float yaw_delta, float delta_x, float delta_y, float delta_z){
    geometry_msgs::Pose pose_action = move_group_interface.getCurrentPose().pose;
    double roll, pitch, yaw;
    Quaterion2Euler(pose_action.orientation, roll, pitch, yaw);
    cout << "current roll, pitch, yaw:" << roll << ", " << pitch << ", " << yaw << endl;

    roll += roll_delta;
    pitch += pitch_delta;
    yaw += yaw_delta;

    cout << "changed roll, pitch, yaw:" << roll << ", " << pitch << ", " << yaw << endl;
    // geometry_msgs::Pose current_pose_push_temp = move_group_interface.getCurrentPose().pose;

    Eigen::Quaternionf q_euler = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
                               * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                               * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    q_euler.normalize();

    pose_action.orientation.x = q_euler.x();
    pose_action.orientation.y = q_euler.y();
    pose_action.orientation.z = q_euler.z();
    pose_action.orientation.w = q_euler.w();

    pose_action.position.x = pose_action.position.x + delta_x;
    pose_action.position.y = pose_action.position.y + delta_y;
    pose_action.position.z = pose_action.position.z + delta_z;

    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    move_group_interface.setPoseTarget(pose_action);
    bool success = (move_group_interface.plan(myplan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        move_group_interface.move();
    }
    else ROS_ERROR("Could not move!");

}

double plan_and_execute_via_waypoints(moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::MoveGroupInterface::Plan& myplan,const geometry_msgs::Pose& start_p, const geometry_msgs::Pose& target_p, double num_midpts)
{

	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose mid_p, transformed_start_p;

	transformed_start_p = start_p;
	transformed_start_p.orientation = target_p.orientation;

	bool orientations_same = quaternion_similarity(transformed_start_p, start_p);
	if(!orientations_same)
	{
		waypoints.push_back(transformed_start_p);
	}

	double delx = (start_p.position.x-target_p.position.x);
	double dely = (start_p.position.y-target_p.position.y);
	double delz = (start_p.position.z-target_p.position.z);
	bool positions_same = std::abs(std::sqrt((delx*delx)+(dely*dely)+(delz*delz))) < 0.001;

	if(orientations_same && positions_same)
	{
		ROS_WARN_STREAM("Asked to cartesian control to identical goal point.");
		return 1.0;
	}
	for (int i = 1; i < num_midpts; ++i)
	{
		mid_p = transformed_start_p;
		mid_p.position.x += (target_p.position.x - transformed_start_p.position.x)/num_midpts * i;
		mid_p.position.y += (target_p.position.y - transformed_start_p.position.y)/num_midpts * i;
		mid_p.position.z += (target_p.position.z - transformed_start_p.position.z)/num_midpts * i;
		waypoints.push_back(mid_p);
	}

	waypoints.push_back(target_p);

	moveit_msgs::RobotTrajectory trajectory;

	group.setStartState(*group.getCurrentState());

	double fraction = group.computeCartesianPath(waypoints, 0.01, 0.00, trajectory);

	ROS_ERROR_STREAM("Waypoint fractional completion..."<<fraction);
	// if (fraction > 0)
	if (std::abs(fraction-1.0) <  0.01)
	{
		myplan.trajectory_ = trajectory;
		if(trajectory.joint_trajectory.points.back().time_from_start.toSec() < 5)
		{
			execute_trajectory(myplan); 
		}

	}
	return fraction;

}


double plan_and_execute_via_waypoints_point(moveit::planning_interface::MoveGroupInterface& group, moveit::planning_interface::MoveGroupInterface::Plan& myplan, float yaw_delta, float pitch_delta, float roll_delta, float delta_x, float delta_y, float delta_z, double num_midpts=20){ 
	geometry_msgs::Pose current_pose = group.getCurrentPose().pose;
	geometry_msgs::Pose target_pose = current_pose;
    double roll, pitch, yaw;
    Quaterion2Euler(current_pose.orientation, roll, pitch, yaw);
    cout << "current roll, pitch, yaw:" << roll << ", " << pitch << ", " << yaw << endl;

    roll += roll_delta;
    pitch += pitch_delta;
    yaw += yaw_delta;

    cout << "changed roll, pitch, yaw:" << roll << ", " << pitch << ", " << yaw << endl;
    // geometry_msgs::Pose current_pose_push_temp = move_group_interface.getCurrentPose().pose;

    Eigen::Quaternionf q_euler = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
                               * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                               * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    q_euler.normalize();

    target_pose.orientation.x = q_euler.x();
    target_pose.orientation.y = q_euler.y();
    target_pose.orientation.z = q_euler.z();
    target_pose.orientation.w = q_euler.w();

    cout << "current position:" << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << endl;
    target_pose.position.x = target_pose.position.x + delta_x;
    target_pose.position.y = target_pose.position.y + delta_y;
    target_pose.position.z = target_pose.position.z + delta_z;
    cout << "changed position:" << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << endl;
	return plan_and_execute_via_waypoints(group, myplan, current_pose, target_pose, num_midpts);
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"demo_test_action");
    
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group_interface("fr3_arm");
    moveit::planning_interface::MoveGroupInterface::Plan myplan;

    move_group_interface.setPlanningTime(0.5);
    move_group_interface.allowReplanning(true);
    move_group_interface.setEndEffectorLink("fr3_hand");
    move_group_interface.setPlannerId("RRTstar");
	move_group_interface.setMaxVelocityScalingFactor(0.2);
    move_group_interface.setMaxAccelerationScalingFactor(0.2);  
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.001);


    // For the setting of environment
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "fr3_link0";
    collision_object.id = "table_design";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.8;
    primitive.dimensions[1] = 0.8;
    primitive.dimensions[2] = 0.6;

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.69;
    table_pose.position.y = 0.26;
    table_pose.position.z = -0.31;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;

    ros::Publisher pub_collision_object = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    ros::Duration(2.0).sleep();  // waiting for publisher
    pub_collision_object.publish(collision_object);

    ros::Publisher pub_gripper = nh.advertise<std_msgs::String>("/GripperCommand", 1, true);
    actionlib::SimpleActionClient<franka_gripper::MoveAction> franka_action("/franka_gripper/move", true);
    actionlib::SimpleActionClient<franka_gripper::GraspAction> franka_action_grasp("/franka_gripper/grasp", true);
    
    WaitUserConfirm("[!] start [!]");  
    std::vector<float> action;
    // float action[7];
    int action_step;

    geometry_msgs::Pose current_pose_action = move_group_interface.getCurrentPose().pose;
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    move_group_interface.setPoseTarget(current_pose_action);
    move_group_interface.move();

    cout << "current pose:" << current_pose_action.position.x << " , " << current_pose_action.position.y << " , " << current_pose_action.position.z << endl;

    // float delta_x, delta_y;
    // double action[3][6];
    // action = {{0.015187380684356905, -0.012168297867564626, -0.02424368244643313, -0.00311655245808996, -0.004956759999780139, 0.002097183678518317},
    //           {0.005383068708812468, -0.019849711744224348, -0.0012805449107990592, -0.0020546634945216302, -0.017527215524631407, 0.004768682880728764},
    //           {0.004712045985109648, 0.005403554257224531, -0.004658273500116439, -0.024405461504178987, -0.028245050838764996, 0.20301984594966835}};

    // int time;
    // time = 0 ;

    while(use_pause){

        // plan_and_execute_via_waypoints_point(move_group_interface, myplan, 0.015187380684356905, 0.012168297867564626, 0.02424368244643313, -0.00311655245808996, -0.004956759999780139, 0.002097183678518317);

        move_franka(move_group_interface, myplan, 0.015187380684356905, 0.012168297867564626, 0.02424368244643313, 0.00311655245808996, 0.004956759999780139, 0.002097183678518317);

        // WaitUserConfirm("[!] pickup [!]"); 
        // pickup_gripper(franka_action_grasp);

        // ros::Duration(1).sleep();
        // move_gripper(1.0, franka_action);

        // WaitUserConfirm("[!] continue [!]"); 

        // pickup_gripper(franka_action_grasp);
        // time = time + 1;

        // if (time >= 3) {
        //     break; // Exit the loop if time exceeds the array bounds
        // }

        geometry_msgs::Pose new_pose_action = move_group_interface.getCurrentPose().pose;
        cout << "new_pose_action pose:" << new_pose_action.position.x << " , " << new_pose_action.position.y << " , " << new_pose_action.position.z << endl;

    }
    
    return 0;
}
