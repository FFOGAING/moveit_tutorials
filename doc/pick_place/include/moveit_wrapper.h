/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra, Franck Fogaing Kamgaing*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <ctime>
// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
enum Type_object {BOX = 1, SPHERE = 2, CYLINDER = 3 , CONE = 4};
class MoveitWrapper
{
public:
    MoveitWrapper(moveit::planning_interface::PlanningSceneInterface* planning_scene_interface,
  moveit::planning_interface::MoveGroupInterface* group);
    ~MoveitWrapper(void);
    float gripper_posture_open_[2];
    float gripper_posture_close_[2];
    void pick_object( std::string object_id, std::string table_id="table1");
    void pick_object( std::string object_id, std::string table_id, float grasp_pose[]);
    void place_object( std::string object_id, std::string table_id="table2");
    void place_object(std::string object_id, std::string table_id, float place_pose[]);
    void add_object(std::string object_type, std::string object_id, float position[]);
    void setup();
    void setup_object();
    int random_number_;

private:
    
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);
    void pick(float grasp_pose[], std::string object_id, std::string table_id);
    void place(float place_pose[], std::string object_id, std::string table_id);
    void addCollisionObject(std::string object_type, std::string object_id, float position[]);
    
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface* group_;
    Type_object object_type_ ;
    
};