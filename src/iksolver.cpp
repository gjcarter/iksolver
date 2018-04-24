#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/attached_body.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Eigenvalues>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <leap_motion/leapros.h>


robot_model::JointModelGroup *model_group = NULL;
robot_state::RobotStatePtr thor_state = NULL;
moveit::planning_interface::MoveGroupInterface *group = NULL;
ros::Publisher pub;
geometry_msgs::Pose pose;
sensor_msgs::JointState goal;
std::vector<double> joint_values;
int count;

void calculateIK(leap_motion::leapros msg){
  // Compute IK
 
  
  pose.orientation.x = 0;
  pose.orientation.y = .7;
  pose.orientation.z = 0;
  pose.orientation.w = 0.72;
  pose.position.x = msg.palmpos.x/250 * .4;
  std::cout << count++ << std::endl;
  if(msg.palmpos.z > 0)
    pose.position.y =  (msg.palmpos.z*.035)/150 + .325;
  else if (msg.palmpos.z > 0)
    pose.position.y = (msg.palmpos.z * .035)/150 + .25;
  pose.position.z = msg.palmpos.y/350 * .5;

  if (((pose.position.y + pose.position.x) <= .5) && (pose.position.y + pose.position.x + pose.position.z) <= .7)  {
    bool found_ik = thor_state->setFromIK(model_group, pose, 10, 0.1);
    if(!found_ik){
      std::cout << "Failed IK Solution" << std::endl;
      //return;
    }
    else
      std::cout << "Found Solution" << std::endl;
    
    goal.name = model_group->getJointModelNames();
    thor_state->copyJointGroupPositions(model_group, joint_values);
    for(int i = 0; i < joint_values.size(); i++)
      std::cout << joint_values[i] << std::endl;
    goal.position = joint_values;
    
    
    // pub.publish(goal);
  }
    
}

int main(int argc, char **argv){
  ros::init(argc, argv, "thor_ik");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::JointState>("joint_states", 1000);
  count = 0;
							      
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr thor_model = robot_model_loader.getModel();
  thor_state.reset(new robot_state::RobotState(thor_model));
  thor_state->setToDefaultValues();
  model_group = thor_model->getJointModelGroup("manipulator");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  group = new moveit::planning_interface::MoveGroupInterface("manipulator");
  
  /*geometry_msgs::PoseStamped pose_target;
  pose_target = group->getCurrentPose();
  std::cout << pose_target.pose.position.x << std::endl;
  std::cout << pose_target.pose.position.y << std::endl;
  std::cout << pose_target.pose.position.z << std::endl;
  std::cout << pose_target.pose.orientation.x << std::endl;
  std::cout << pose_target.pose.orientation.y << std::endl;
  std::cout << pose_target.pose.orientation.z << std::endl;
  std::cout << pose_target.pose.orientation.w << std::endl;
  //std::vector<double> joint_values; 
  //for(int i = 0; i < joint_values.size()-1; i++)
  //std::cout << joint_values[i] << std::endl;
   
  pose_target.pose.position.x = -0.219293;
  pose_target.pose.position.y = .320425;
  pose_target.pose.position.z = .18;
  pose_target.pose.orientation.x = 0.180713;
  pose_target.pose.orientation.y = -0.58803;
  pose_target.pose.orientation.z = -0.531438;*/
 
 
  
  ros::Subscriber sub = node.subscribe("leapmotion/data", 1000, calculateIK);
  
  ros::spin();
  
  ros::shutdown();
  return 0;
}
