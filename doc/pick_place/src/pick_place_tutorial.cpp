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
    int random_number = moveit_wrapper.random_number_;
  /*******************************************************************************************/
    
  /********************************INSERT YOUR CODE HERE *************************************/
    


  /*******************************************************************************************/
  
  

  
  //ros::waitForShutdown();
  return 0;
}

