#include "moveit_wrapper.h"

MoveitWrapper::MoveitWrapper(moveit::planning_interface::PlanningSceneInterface* planning_scene_interface,
  moveit::planning_interface::MoveGroupInterface* group)
{
    planning_scene_interface_ = planning_scene_interface;
    group_ = group;
    object_id_ = "";
}
MoveitWrapper::~MoveitWrapper(void)
{
    
}

void MoveitWrapper::open_gripper()
{

}
void MoveitWrapper::closed_gripper()
{

}
    
void MoveitWrapper::pick_object()
{
    pick();
}

void MoveitWrapper::place_object()
{
    place();
}

void MoveitWrapper::add_object(std::string object_type, int num_object, std::string object_id)
{
    addCollisionObjects(object_type, num_object, object_id);
}

// void MoveitWrapper::add_object(std::string object_type)
// {
//     addCollisionObjects(object_type);
// }

void MoveitWrapper::add_object(int num_object)
{   
    std::string std_object_type = "BOX";
    addCollisionObjects(std_object_type, num_object);
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
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
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
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
  ros::WallDuration(1.0).sleep();
}

void MoveitWrapper::pick()
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
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

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
  group_->setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  group_->pick(object_id_, grasps);
  // END_SUB_TUTORIAL
  ros::WallDuration(1.0).sleep();
}

void MoveitWrapper::place()
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
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

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
  group_->setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group_->place(object_id_, place_location);
  // END_SUB_TUTORIAL
  ros::WallDuration(1.0).sleep();
}

void MoveitWrapper::addCollisionObjects(std::string object_type, int num_object, std::string object_id)
{
    // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(num_object + 2);
  
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

  
  for (uint8_t i=2; i<(num_object+2); i++)
  {
    // Define the object that we will be manipulating
      object_id_ =  object_id; //"object"+ std::to_string(i+2);
      collision_objects[i].header.frame_id = "panda_link0";
      collision_objects[i].id = object_id_;
      /* Define the primitive and its dimensions. */
      collision_objects[i].primitives.resize(1);
      collision_objects[i].primitives[0].type = object_type_; //collision_objects[1].primitives[0].BOX;
      
      switch (object_type_)
      {
      case BOX:
        collision_objects[i].primitives[0].dimensions.resize(3);
        collision_objects[i].primitives[0].dimensions[0] = 0.02;
        collision_objects[i].primitives[0].dimensions[1] = 0.02;
        collision_objects[i].primitives[0].dimensions[2] = 0.2;

      /* Define the pose of the object. */
        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position.x = 0.5;
        collision_objects[i].primitive_poses[0].position.y = 0;
        collision_objects[i].primitive_poses[0].position.z = 0.5;
        collision_objects[i].primitive_poses[0].orientation.w = 1.0;
        break;
      case SPHERE:
        collision_objects[i].primitives[0].dimensions.resize(1);
        collision_objects[i].primitives[0].dimensions[0] = 0.02;

      /* Define the pose of the object. */
        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position.x = 0.5;
        collision_objects[i].primitive_poses[0].position.y = 0;
        collision_objects[i].primitive_poses[0].position.z = 0.5;
        collision_objects[i].primitive_poses[0].orientation.w = 1.0;
        break;
      case CYLINDER:
        collision_objects[i].primitives[0].dimensions.resize(2);
        collision_objects[i].primitives[0].dimensions[0] = 0.2;
        collision_objects[i].primitives[0].dimensions[1] = 0.02;
    
      /* Define the pose of the object. */
        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position.x = 0.5;
        collision_objects[i].primitive_poses[0].position.y = 0;
        collision_objects[i].primitive_poses[0].position.z = 0.5;
        collision_objects[i].primitive_poses[0].orientation.w = 1.0;
        break;
      
      default:
        break;
      }
      
      //END_SUB_TUTORIAL
     collision_objects[i].operation = collision_objects[i].ADD;

  }
 
  
  
  
  planning_scene_interface_->applyCollisionObjects(collision_objects);
  ros::WallDuration(1.0).sleep();
}