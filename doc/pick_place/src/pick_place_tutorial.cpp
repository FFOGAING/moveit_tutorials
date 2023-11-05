#include "moveit_wrapper.h"
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{  

  /********************************SETUP*************************************/
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(45.0);
    MoveitWrapper moveit_wrapper(&planning_scene_interface, &group);
    moveit_wrapper.setup();
    moveit_wrapper.setup_object();
  /*******************************************************************************************/
    
  /********************************INSERT YOUR CODE HERE *************************************/
    // int num_object = 1;
    // std::string typ_object = "SPHERE";
    // std::string object_id = "object";
    // moveit_wrapper.add_object(typ_object, num_object,object_id);
    // moveit_wrapper.pick_object();
    // moveit_wrapper.place_object();
 
  /*******************************************************************************************/
  // ARRAY TO GIVE THE ACTUAL POSE AND PLACE POSE , TRY TO GIVE DEFAULT VALUE
  

  
  //ros::waitForShutdown();
  return 0;
}

// BEGIN_TUTORIAL
// CALL_SUB_TUTORIAL table1
// CALL_SUB_TUTORIAL table2
// CALL_SUB_TUTORIAL object
//
// Pick Pipeline
// ^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL pick1
// openGripper function
// """"""""""""""""""""
// CALL_SUB_TUTORIAL open_gripper
// CALL_SUB_TUTORIAL pick2
// closedGripper function
// """"""""""""""""""""""
// CALL_SUB_TUTORIAL closed_gripper
// CALL_SUB_TUTORIAL pick3
//
// Place Pipeline
// ^^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL place
// END_TUTORIAL
