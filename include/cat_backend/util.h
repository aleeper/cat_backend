#ifndef _CAT_BACKEND_UTIL_
#define _CAT_BACKEND_UTIL_


//#include <moveit/robot_state/robot_state.h>
//#include <moveit/robot_state/joint_state_group.h>
//#include <trajectory_msgs/JointTrajectory.h>

//// ======== This is duplicated from CVX solver. Does this exist in MoveIt! already?(!)
//inline void robotStateVectorToJointTrajectory( const std::vector<robot_state::RobotStatePtr>& states,
//                                                   const std::string& group_name,
//                                                   trajectory_msgs::JointTrajectory & traj)
//{
//  int num_states = states.size();
//  traj.points.resize(num_states);
//  for(int i = 0; i < num_states; i++)
//  {
//    const std::vector<robot_state::JointState*>& jsv = states[i]->getJointStateGroup(group_name)->getJointStateVector();
//    int num_joints = jsv.size();
//    if(i == 0) // only do once per trajectory
//      traj.joint_names.resize(num_joints);
//    traj.points[i].positions.resize(num_joints);
//    for(size_t j = 0 ; j < num_joints; j++)
//    {
//      if( i == 0 ) // only do once per trajectory
//        traj.joint_names[j] = jsv[j]->getName();
//      traj.points[i].positions[j] = jsv[j]->getVariableValues()[0];
//      //if(js.velocity.size()) pt.velocities.push_back(js.velocity[i]);
//    }
//  }
//}


#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// ====================================================================================================================
// Convenience functions from object manipulator
// ====================================================================================================================
namespace{

// Stolen wholesale from object_manipulator::MechanismInterface
void positionAndAngleDist(Eigen::Affine3d start, Eigen::Affine3d end,
                                              double &pos_dist,
double &angle, Eigen::Vector3d &axis, Eigen::Vector3d &direction)
{
  //trans = end to start = global to start * end to global
  Eigen::Affine3d trans;
  trans = start.inverse() * end;
  Eigen::AngleAxis<double> angle_axis;
  angle_axis = trans.rotation();
  angle = angle_axis.angle();
  axis = angle_axis.axis();
  if(angle > M_PI)
  {
    angle = -(angle - 2*M_PI);
    axis = -axis;
  }
  direction = trans.translation();
  pos_dist = sqrt(direction.dot(direction));
  if(pos_dist) direction *= 1/pos_dist;
}

// Stolen wholesale from object_manipulator::MechanismInterface
geometry_msgs::PoseStamped clipDesiredPose(const geometry_msgs::PoseStamped &current_pose,
                                           const geometry_msgs::PoseStamped &desired_pose,
                                           double clip_dist, double clip_angle,
                                           double &resulting_clip_fraction)
{
  //no clipping desired
  if(clip_dist == 0 && clip_angle == 0)
  {
    resulting_clip_fraction = 0;
    return desired_pose;
  }

  //Get the position and angle dists between current and desired
  Eigen::Affine3d current_trans, desired_trans;
  double pos_dist, angle;
  Eigen::Vector3d axis, direction;
  tf::poseMsgToEigen(current_pose.pose, current_trans);
  tf::poseMsgToEigen(desired_pose.pose, desired_trans);
  positionAndAngleDist(current_trans, desired_trans, pos_dist, angle, axis, direction);

  //Clip the desired pose to be at most clip_dist and the desired angle to be at most clip_angle (proportional)
  //from the current
  double pos_mult, angle_mult;
  double pos_change, angle_change;
  angle_mult = fabs(angle / clip_angle);
  pos_mult = fabs(pos_dist / clip_dist);
  if(pos_mult <=1 && angle_mult <=1){
    return desired_pose;
  }
  double mult = (angle_mult > pos_mult) ? angle_mult : pos_mult;
  pos_change = pos_dist / mult;
  angle_change = angle / mult;
  resulting_clip_fraction = 1 / mult;

  Eigen::Affine3d clipped_trans;
  clipped_trans = current_trans;
  Eigen::Vector3d scaled_direction;
  scaled_direction = direction * pos_change;
  Eigen::Translation3d translation(scaled_direction);
  clipped_trans = clipped_trans * translation;
  Eigen::AngleAxis<double> angle_axis(angle_change, axis);
  clipped_trans = clipped_trans * angle_axis;
  geometry_msgs::PoseStamped clipped_pose;
  tf::poseEigenToMsg(clipped_trans, clipped_pose.pose);
  clipped_pose.header = desired_pose.header;
  return clipped_pose;
}

}

#endif // _CAT_BACKEND_UTIL_
