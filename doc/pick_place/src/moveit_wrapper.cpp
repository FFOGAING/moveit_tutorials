#include "moveit_wrapper.h"

MoveitWrapper::MoveitWrapper(moveit::planning_interface::PlanningSceneInterface* planning_scene_interface,
  moveit::planning_interface::MoveGroupInterface* group)
{
    planning_scene_interface_ = planning_scene_interface;
    group_ = group;
    gripper_posture_open_[0]=0.04;
    gripper_posture_open_[1]=0.04;
    gripper_posture_close_[0]=0.00;
    gripper_posture_close_[1]=0.00;
}
MoveitWrapper::~MoveitWrapper(void)
{
    
}
   
void MoveitWrapper::pick_object(std::string object_id, std::string table_id)
{   
    float grasp_pose[] = {0.415,0,0.5};
    if (object_id == "BOX")
    {
        grasp_pose[0] = 0.415;
    }
    else if (object_id == "CYLINDER")
    {
        grasp_pose[0] = 0.215;
    }
    else if(object_id == "SPHERE")
    {
        grasp_pose[0] = 0.315;
    }
    
    pick(grasp_pose, object_id, table_id);
}

void MoveitWrapper::pick_object( std::string object_id, std::string table_id, float grasp_pose[])
{   
    pick(grasp_pose, object_id, table_id);
}

void MoveitWrapper::place_object( std::string object_id, std::string table_id)
{   
  float place_pose[] = {0,0.5,0.5};
  if(table_id =="table2")
    {
        if (object_id == "BOX")
        {
           place_pose[1] = 0.5;
        }
        else if (object_id == "CYLINDER")
        {
           place_pose[1] = 0.5;
        }
        else if(object_id == "SPHERE")
        {
           place_pose[1] = 0.5;
        }
     
    }
    else if(table_id == "table3")
    {
        if (object_id == "BOX")
        {
             place_pose[1] = -0.5;
        }
        else if (object_id == "CYLINDER")
        {
             place_pose[1] = -0.5;
        }
        else if(object_id == "SPHERE")
        {
             place_pose[1] = -0.5;
        }
    }
    
    
    place( place_pose, object_id, table_id);
}

void MoveitWrapper::place_object(std::string object_id, std::string table_id, float place_pose[])
{
    place( place_pose, object_id, table_id);
}

void MoveitWrapper::add_object(std::string object_type, std::string object_id, float position[], float object_dim[])
{
    addCollisionObject(object_type, object_id,  position, object_dim);
}

void MoveitWrapper::openGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = gripper_posture_open_[0];//0.04;
  posture.points[0].positions[1] = gripper_posture_open_[1];//0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void MoveitWrapper::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = gripper_posture_close_[0];
  posture.points[0].positions[1] = gripper_posture_close_[1];
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
  ros::WallDuration(1.0).sleep();
}

void MoveitWrapper::pick(float grasp_pose[], std::string object_id, std::string table_id)
{
    // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x =  grasp_pose[0];//0.415;
  grasps[0].grasp_pose.pose.position.y = grasp_pose[1];//0;
  grasps[0].grasp_pose.pose.position.z = grasp_pose[2];//0.5;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  group_->setSupportSurfaceName(table_id); //"table1"
  // Call pick to pick up the object using the grasps given
  group_->pick(object_id, grasps);
  // END_SUB_TUTORIAL
  ros::WallDuration(1.0).sleep();
}

void MoveitWrapper::place(float place_pose[], std::string object_id, std::string table_id)
{
    // BEGIN_SUB_TUTORIAL place
  // location in verbose mode." This is a known issue. |br|
  // |br|
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = place_pose[0];//0;
  place_location[0].place_pose.pose.position.y = place_pose[0];//0.5;
  place_location[0].place_pose.pose.position.z = place_pose[0];//0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group_->setSupportSurfaceName(table_id); //"table2"
  // Call place to place the object using the place locations given.
  group_->place(object_id, place_location);
  // END_SUB_TUTORIAL
  ros::WallDuration(1.0).sleep();
}

void MoveitWrapper::setup()
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL table3
  // Add the second table where we will be placing the cube.
  collision_objects[2].id = "table3";
  collision_objects[2].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.4;
  collision_objects[2].primitives[0].dimensions[1] = 0.2;
  collision_objects[2].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0;
  collision_objects[2].primitive_poses[0].position.y = -0.5;
  collision_objects[2].primitive_poses[0].position.z = 0.2;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[2].ADD;

  planning_scene_interface_->applyCollisionObjects(collision_objects);
}

void MoveitWrapper::setup_object()
{     
      std::vector<moveit_msgs::CollisionObject> collision_objects;
      collision_objects.resize(3);

      collision_objects[0].header.frame_id = "panda_link0";
      collision_objects[0].id = "BOX";
      collision_objects[0].primitives.resize(1);
      collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX; 
      
      collision_objects[0].primitives[0].dimensions.resize(3);
      collision_objects[0].primitives[0].dimensions[0] = 0.02;
      collision_objects[0].primitives[0].dimensions[1] = 0.02;
      collision_objects[0].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
      collision_objects[0].primitive_poses.resize(1);
      collision_objects[0].primitive_poses[0].position.x = 0.5;
      collision_objects[0].primitive_poses[0].position.y = 0;
      collision_objects[0].primitive_poses[0].position.z = 0.5;
      collision_objects[0].primitive_poses[0].orientation.w = 1.0;

      collision_objects[1].header.frame_id = "panda_link0";
      collision_objects[1].id = "SPHERE";
      collision_objects[1].primitives.resize(1);
      collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].SPHERE; 
      
      collision_objects[1].primitives[0].dimensions.resize(1);
      collision_objects[1].primitives[0].dimensions[0] = 0.02;

    /* Define the pose of the object. */
      collision_objects[1].primitive_poses.resize(1);
      collision_objects[1].primitive_poses[0].position.x = 0.4;
      collision_objects[1].primitive_poses[0].position.y = 0.1;
      collision_objects[1].primitive_poses[0].position.z = 0.5;
      collision_objects[1].primitive_poses[0].orientation.w = 1.0;

      collision_objects[2].header.frame_id = "panda_link0";
      collision_objects[2].id = "CYLINDER";
      collision_objects[2].primitives.resize(1);
      collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].CYLINDER; 
      
      collision_objects[2].primitives[0].dimensions.resize(1);
      collision_objects[2].primitives[0].dimensions[0] = 0.02;
      collision_objects[2].primitives[0].dimensions.resize(2);
      collision_objects[2].primitives[0].dimensions[0] = 0.2;
      collision_objects[2].primitives[0].dimensions[1] = 0.02;
    
    /* Define the pose of the object. */
      collision_objects[2].primitive_poses.resize(1);
      collision_objects[2].primitive_poses[0].position.x = 0.3;
      collision_objects[2].primitive_poses[0].position.y = 0.2;
      collision_objects[2].primitive_poses[0].position.z = 0.5;
      collision_objects[2].primitive_poses[0].orientation.w = 1.0;


    
      planning_scene_interface_->applyCollisionObjects(collision_objects);
}

void MoveitWrapper::addCollisionObject(std::string object_type, std::string object_id, float position[], float object_dim[])
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  //object type
  if(object_type == "BOX")
  {
    object_type_ = BOX;
  }
  else if(object_type == "SPHERE")
  {
    object_type_ = SPHERE;
  }
  else if(object_type == "CYLINDER")
  {
    object_type_ = CYLINDER;
  }
  
  ROS_INFO("Object type : %s", object_type.c_str());
  ROS_INFO("Object type : %d", object_type_);
 
// Define the object that we will be manipulating 
  collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].id = object_id; 
  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = object_type_; //collision_objects[1].primitives[0].BOX;
  
  switch (object_type_)
  {
  case BOX:
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = object_dim[0];//0.02;
    collision_objects[0].primitives[0].dimensions[1] = object_dim[1];//0.02;
    collision_objects[0].primitives[0].dimensions[2] = object_dim[2];//0.2;

  /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = position[0];//0.5;
    collision_objects[0].primitive_poses[0].position.y = position[1];//0;
    collision_objects[0].primitive_poses[0].position.z = position[2];//0.5;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    break;
  case SPHERE:
    collision_objects[0].primitives[0].dimensions.resize(1);
    collision_objects[0].primitives[0].dimensions[0] = 0.02;

  /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = position[0];//0.5;
    collision_objects[0].primitive_poses[0].position.y = position[1];//0;
    collision_objects[0].primitive_poses[0].position.z = position[2];//0.5;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    break;
  case CYLINDER:
    collision_objects[0].primitives[0].dimensions.resize(2);
    collision_objects[0].primitives[0].dimensions[0] = object_dim[0];//0.2;
    collision_objects[0].primitives[0].dimensions[1] = object_dim[1];//0.02;

  /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = position[0];//0.5;
    collision_objects[0].primitive_poses[0].position.y = position[1];//0;
    collision_objects[0].primitive_poses[0].position.z = position[2];//0.5;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    break;
  
  default:
    break;
  
  
  //END_SUB_TUTORIAL
  collision_objects[0].operation = collision_objects[0].ADD;

  }
 
  
  
  
  planning_scene_interface_->applyCollisionObjects(collision_objects);
  ros::WallDuration(1.0).sleep();
}