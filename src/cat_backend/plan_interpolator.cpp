#include <cat_backend/plan_interpolator.h>

namespace plan_interpolator
{

void PlanInterpolator::findIndexAtTimeFromStart(const ros::Duration& time, int& before, int& after, double &interpolate)
{
  size_t num_points = current_plan_->trajectory_.joint_trajectory.points.size();
  size_t index = 0;

  for( ; index < num_points; index++)
  {
    ROS_DEBUG("Trajectory index %zd of %zd. Time: %.3f vs. %.3f ",
             index,
             num_points,
             current_plan_->trajectory_.joint_trajectory.points[index].time_from_start.toSec(),
             time.toSec());
    if( current_plan_->trajectory_.joint_trajectory.points[index].time_from_start > time )
      break;
  }

  before = std::max<int>(index - 1, 0);
  after = std::min<int>(index, num_points - 1);
  ros::Duration before_time = current_plan_->trajectory_.joint_trajectory.points[before].time_from_start;
  ros::Duration after_time = current_plan_->trajectory_.joint_trajectory.points[after].time_from_start;
  ros::Duration interval = after_time - before_time;
  if(after == before || interval.toSec() <= 0 )
    interpolate = 1.0;
  else
    interpolate = (time - before_time).toSec() / interval.toSec();
}

void PlanInterpolator::getStateAtTime(const ros::Time &request_time, kinematic_state::KinematicStatePtr& start_state,
                                       moveit_msgs::RobotState& rs)
{
  if(plan_is_valid_ && current_plan_ && current_plan_->trajectory_.joint_trajectory.points.size() >= 2)
  {
    ros::Time plan_start_time = current_plan_->trajectory_.joint_trajectory.header.stamp;
    ros::Duration diff_time = request_time - plan_start_time;
    if( diff_time.toSec() < 0)
    {
      ROS_ERROR("diff_time is negative... what does this mean?");
      diff_time = ros::Duration(0.0);
    }

    kinematic_state::KinematicStatePtr future_start_state_1, future_start_state_2;
    future_start_state_1.reset( new kinematic_state::KinematicState(*start_state));
    future_start_state_2.reset( new kinematic_state::KinematicState(*start_state));

    int before=0, after=0;
    double interpolate = 1.0;
    findIndexAtTimeFromStart(diff_time, before, after, interpolate);
    ROS_DEBUG("Using indices %d and %d with interpolation %.3f.", before, after, interpolate);
    const trajectory_msgs::JointTrajectoryPoint& point_before = current_plan_->trajectory_.joint_trajectory.points[before];
    const trajectory_msgs::JointTrajectoryPoint& point_after = current_plan_->trajectory_.joint_trajectory.points[after];
    const std::vector<std::string>& joint_names = current_plan_->trajectory_.joint_trajectory.joint_names;

    future_start_state_1->setStateValues(current_plan_->trajectory_.joint_trajectory.joint_names, point_before.positions);
    future_start_state_2->setStateValues(current_plan_->trajectory_.joint_trajectory.joint_names, point_after.positions);
    future_start_state_1->interpolate(*future_start_state_2, interpolate, *start_state);

    kinematic_state::kinematicStateToRobotState(*start_state, rs);
    std::map<std::string, std::pair<double, double> > velocity_map;

    int num_joints = joint_names.size();
    int num_vel = point_after.velocities.size();
    int num_accel = point_after.accelerations.size();
    bool has_velocity_data     = num_joints == num_vel;
    bool has_acceleration_data = num_joints == num_accel;

    if(has_velocity_data || has_acceleration_data)
    //if(false)
    {
      if( (has_velocity_data && has_acceleration_data) && num_vel != num_accel )
      {
        ROS_ERROR("Velocity data [%d] and acceleration data [%d] are not the same length!", num_vel, num_accel);
      }
      else
      {
        ROS_DEBUG("Have either velocity [%d] or acceleration [%d] data; populating...",
                  num_vel, num_accel);
        for(int i=0; i < num_joints; i++)
        {
          double vel = has_velocity_data ?
                (point_after.velocities[i] - point_before.velocities[i])*interpolate + point_before.velocities[i] : 0;
          double accel = has_acceleration_data ?
                (point_after.accelerations[i] - point_before.accelerations[i])*interpolate + point_before.accelerations[i] : 0;
          velocity_map[joint_names[i]] = std::pair<double, double>(vel, accel);
          ROS_DEBUG("Joint [%s] is projected to have velocity [%.3f] and accel [%.3f]", joint_names[i].c_str(), vel, accel );
        }

        if( has_velocity_data )
          rs.joint_state.velocity.resize(rs.joint_state.name.size());
        if( has_acceleration_data )
          rs.joint_state.effort.resize(rs.joint_state.name.size());
        for(int j = 0; j < rs.joint_state.name.size(); j++)
        {
          std::map<std::string, std::pair<double, double> >::const_iterator it = velocity_map.find(rs.joint_state.name[j]);
          if(it != velocity_map.end())
          {
            if(has_velocity_data)
              rs.joint_state.velocity[j] = it->second.first;
            if(has_acceleration_data)
              rs.joint_state.effort[j] = it->second.second;
          }
          else
          {
            if(has_velocity_data)
              rs.joint_state.velocity[j] = 0.0;
            if(has_acceleration_data)
              rs.joint_state.effort[j] = 0.0;
          }
        }
        rs.joint_state.header.stamp = request_time;
      }
    }
    else
    {
      rs.joint_state.header.stamp = request_time;
    }
  }
  else
  {
    ROS_DEBUG("No stored plan, just returning input state...");
    kinematic_state::kinematicStateToRobotState(*start_state, rs);
    rs.joint_state.header.stamp = ros::Time(0);
  }
}

void PlanInterpolator::printPlan()
{
  if(current_plan_)
  {
    std::string valid_plan = plan_is_valid_ ? "valid" : "not valid";
    std::string new_plan   = has_new_plan_  ? "unused" : "already used";
    ROS_INFO("Current plan is %s and %s.", valid_plan.c_str(), new_plan.c_str());
    ROS_INFO_STREAM(current_plan_->trajectory_);
  }
}

} //namespace cat
