#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/attached_body.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <leap_motion/leapros.h>
#include <cmath>
#include <tf/transform_listener.h>

robot_model::JointModelGroup *model_group = NULL;
robot_state::RobotStatePtr discoverbot_state = NULL;
moveit::planning_interface::MoveGroupInterface *group = NULL;
ros::Publisher pub;
geometry_msgs::Pose pose;
sensor_msgs::JointState goal;
std::vector<double> joint_values;

/* COORDINATE INFO 
 * Top coord (x,y) = (.2, 0.48)
 * Bottom coord (x,y) =  (0.2, 0.06)
 * Extended coord (x,y) = (.37, .2)
 * Pulled back coord (x,y) = (.19, .2)
 * Z limits = +-.3
 */

/* TODO:
 * -Still getting some failed calcs most likely due to x coords.
 * -High y values also causing failure
 */

void calculateIK(leap_motion::leapros msg){

  //Temporary valid orientation so that iksolver doesn't fail
  pose.orientation.x = 0;
  pose.orientation.y = -0.71;
  pose.orientation.z = .0;
  pose.orientation.w = 0.7;

  pose.position.y = msg.palmpos.x/170 * .3;
  /* Currently limiting x to a plane. Could be switched to another 
   * ellipsoid in the future.
   */
  if(msg.palmpos.z <= 0)
    pose.position.x =  ((abs(msg.palmpos.z) * .09)/250 + .27);
  else if (msg.palmpos.z > 0)
    pose.position.x = (.27 - (msg.palmpos.z * .09)/130);
  pose.position.z = (msg.palmpos.y-50)/350 * .42;

  /* Using equation for an ellipsoid to check if pose it within 3 given radii
   * Equation: x^2/radius1^2 + y^2/radius2^2 + z^2/radius3^2 = 1
   * radius1 = .18, radius2 = .2, radius3 = .3
   */
  if ((((pose.position.x - .18) * (pose.position.x - .18) / .0324) +
       (pose.position.y * pose.position.y / .09) +
       ((pose.position.z - .21) * (pose.position.z - .21) / .0441)) <= 1) {
    //X needs to be negative to be a valid pose
    pose.position.x *= -1;
    bool found_ik = discoverbot_state->setFromIK(model_group, pose, 3, 0.1);
    if(!found_ik){
      /*std::cout << "Failed IK Solution" << std::endl;
      std::cout << "x:" <<  pose.position.x << std::endl;
      std::cout << "z:" << pose.position.y << std::endl;
      std::cout << "y:" << pose.position.z << std::endl;*/
      return;
    }
    //else
      //std::cout << "Found Solution" << std::endl;
   
    goal.name = model_group->getJointModelNames();
    discoverbot_state->copyJointGroupPositions(model_group, joint_values);
    goal.position = joint_values;

    /* Only allow negative angles to prevent random angles that come from 
     * Leap sometimes. Might have to edit this a bit later as it does restrict
     * a couple positions.
     */ 
    if(msg.ypr.x > 0)
      msg.ypr.x *= -1;
    goal.position[4] = (msg.ypr.x/1.2 + 45)/57.2958;
    /* Using roll of hand to change angle of joint 3 
     * is preferred but needs physical testing on robot
     * in order to see if it would be valid with some random
     * movements due to Leap's rpy tracking not being as
     * accurate as xyz. For now just moving joint 5 with
     * hand roll.
     */
    goal.position[5] = (msg.ypr.z)/-57.2958;
    goal.position[3] = 0;
    //goal.position[3] = (msg.ypr.z)/-57.2958;
    pub.publish(goal);

    
  }   
}

int main(int argc, char **argv){
  ros::init(argc, argv, "discoverbot_ik");
  ros::NodeHandle node;
  /* Currently publishing to joint_states for Rviz but when Arduino
   * is implemented this will publish to that
   */
  pub = node.advertise<sensor_msgs::JointState>("joint_states", 1000);
							      
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr discoverbot_model = robot_model_loader.getModel();
  discoverbot_state.reset(new robot_state::RobotState(discoverbot_model));
  discoverbot_state->setToDefaultValues();
  model_group = discoverbot_model->getJointModelGroup("manipulator");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  group = new moveit::planning_interface::MoveGroupInterface("manipulator");
  
  ros::Subscriber sub = node.subscribe("leapmotion/data", 1000, calculateIK);
  
  ros::spin();
  
  ros::shutdown();
  return 0;
}
