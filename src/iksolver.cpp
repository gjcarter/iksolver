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
#include <cmath>

robot_model::JointModelGroup *model_group = NULL;
robot_state::RobotStatePtr thor_state = NULL;
moveit::planning_interface::MoveGroupInterface *group = NULL;
ros::Publisher pub;
geometry_msgs::Pose pose;
sensor_msgs::JointState goal;
std::vector<double> joint_values;

/* COORDINATE INFO 
 * Top coord =~ (.28, .48)
 * Bottom coord =~ (0.28, 0)
 * Straight out coord =~ (0.4, 0.24)
 * Z limit = .28
 * X limit = +-.27
 */

/* TODO:
 * -Still getting some failed calcs most likely due to x coords.
 * -High y values also causing failure
 */

void calculateIK(leap_motion::leapros msg){
  pose.orientation.x = -.71;
  pose.orientation.y = 0;
  pose.orientation.z = .0;
  pose.orientation.w = 0.7;
  pose.position.x = msg.palmpos.x/250 * .4;
  /* Currently limiting z to .28-.4. Points at .28 are limited
   * by a plane rather than a ellipsoid but this could be changed
   * in the future.
   */
  if(msg.palmpos.z <= 0)
    pose.position.y =  (abs(msg.palmpos.z) * .06)/150 + .34;
  else if (msg.palmpos.z > 0)
    pose.position.y = .34 - (msg.palmpos.z * .06)/150;
  pose.position.z = msg.palmpos.y/350 * .48;

  double radiusCheck = ((pose.position.x * pose.position.x / .0729) +
		     ((pose.position.y - .28) * (pose.position.y - .28) / .0144) +
		     ((pose.position.z - .24) * (pose.position.z -.24) / .0576));
  // std::cout << "radius:" << radiusCheck << std::endl;

  /* Using equation for an ellipsoid to check if pose it within 3 given radii
   * Equation: x^2/radius1^2 + y^2/radius2^2 + z^2/radius3^2 = 1
   * radius1 = .27, radius2 = .28, radius3 = .12
   */
  if (((pose.position.x * pose.position.x / .0729) +
       ((pose.position.y - .28) * (pose.position.y - .28) / .0144) +
       ((pose.position.z - .24) * (pose.position.z -.24) / .0576)) <= 1) {
    bool found_ik = thor_state->setFromIK(model_group, pose, 3, 0.1);
    if(!found_ik){
      //std::cout << "Failed IK Solution" << std::endl;
      return;
    }
    // else
      //std::cout << "Found Solution" << std::endl;
   
    goal.name = model_group->getJointModelNames();
    thor_state->copyJointGroupPositions(model_group, joint_values);
    goal.position = joint_values;
    
    /*std::cout << "x:" <<  pose.position.x << std::endl;
    std::cout << "z:" << pose.position.y << std::endl;
    std::cout << "y:" << pose.position.z << std::endl;
    std::cout << "leap z:" << msg.palmpos.z << std::endl;*/
    //for(int i = 0; i < joint_values.size(); i++)
    //std::cout << joint_values[i] << std::endl;
    
    pub.publish(goal);
  }   
}

int main(int argc, char **argv){
  ros::init(argc, argv, "thor_ik");
  ros::NodeHandle node;
  pub = node.advertise<sensor_msgs::JointState>("joint_states", 1000);
							      
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr thor_model = robot_model_loader.getModel();
  thor_state.reset(new robot_state::RobotState(thor_model));
  thor_state->setToDefaultValues();
  model_group = thor_model->getJointModelGroup("manipulator");
  group = new moveit::planning_interface::MoveGroupInterface("manipulator");
  
  /*geometry_msgs::PoseStamped pose_target;
  pose_target = group->getCurrentPose();
  std::cout << pose_target.pose.position.x << std::endl;
  std::cout << pose_target.pose.position.y << std::endl;
  std::cout << pose_target.pose.position.z << std::endl;
  std::cout << pose_target.pose.orientation.x << std::endl;
  std::cout << pose_target.pose.orientation.y << std::endl;
  std::cout << pose_target.pose.orientation.z << std::endl;
  std::cout << pose_target.pose.orientation.w << std::endl;*/

  ros::Subscriber sub = node.subscribe("leapmotion/data", 1000, calculateIK);
  
  ros::spin();
  
  ros::shutdown();
  return 0;
}
