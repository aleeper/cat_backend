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

bool PlanInterpolator::getStateAtTime(const ros::Time &request_time, robot_state::RobotStatePtr& start_state,
                                       moveit_msgs::RobotState& rs, bool do_interpolate)
{
  if(plan_is_valid_ && current_plan_ && current_plan_->trajectory_.joint_trajectory.points.size() >= 2)
  {
    ros::Time plan_start_time = current_plan_->trajectory_.joint_trajectory.header.stamp;
    ros::Duration diff_time = request_time - plan_start_time;
    ros::Time output_time = request_time;

    if( diff_time.toSec() < 0)
    {
      //ROS_ERROR("diff_time is negative... what does this mean?");
      diff_time = ros::Duration(0);
    }

    robot_state::RobotStatePtr future_start_state_1, future_start_state_2;
    future_start_state_1.reset( new robot_state::RobotState(*start_state));
    future_start_state_2.reset( new robot_state::RobotState(*start_state));

    int before=0, after=0;
    double interpolation = 1.0;
    findIndexAtTimeFromStart(diff_time, before, after, interpolation);
    const trajectory_msgs::JointTrajectoryPoint& point_before = current_plan_->trajectory_.joint_trajectory.points[before];
    const trajectory_msgs::JointTrajectoryPoint& point_after = current_plan_->trajectory_.joint_trajectory.points[after];
    const std::vector<std::string>& joint_names = current_plan_->trajectory_.joint_trajectory.joint_names;
    if(do_interpolate)
    {
      ROS_DEBUG("Interpolating %.3f of the way between index %d and %d.", interpolation, before, after);

      future_start_state_1->setStateValues(joint_names, point_before.positions);
      future_start_state_2->setStateValues(joint_names, point_after.positions);
      future_start_state_1->interpolate(*future_start_state_2, interpolation, *start_state);
    }
    else
    {
      if(diff_time.isZero()) // exit, as we are getting ahead of ourselves
      {
        return false;
//        ROS_DEBUG("Time is %.3f of the way between index %d and %d. Rounding down to index %d.", interpolation, before, after, before);
//        interpolation = 0.0;
//        output_time = plan_start_time;
//        start_state->setStateValues(joint_names, point_before.positions);
      }
      else // round up
      {
        ROS_DEBUG("Time is %.3f of the way between index %d and %d. Rounding up to index %d.", interpolation, before, after, after);
        interpolation = 1.0;
        output_time = plan_start_time + point_after.time_from_start - ros::Duration(0.00001); // prevents double comparison issues in the low-level controller
        start_state->setStateValues(joint_names, point_after.positions);
      }
    }
    // Copy to msg
    robot_state::robotStateToRobotStateMsg(*start_state, rs);
    rs.joint_state.header.stamp = output_time;

    std::map<std::string, std::pair<double, double> > velocity_map;
    int num_joints = joint_names.size();
    int num_vel = point_after.velocities.size();
    int num_accel = point_after.accelerations.size();
    bool has_velocity_data     = num_joints == num_vel;
    bool has_acceleration_data = false;; //num_joints == num_accel;

    if(has_velocity_data || has_acceleration_data)
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
          double vel, accel;
          if(do_interpolate)
          {
            vel = has_velocity_data ?
                  (point_after.velocities[i] - point_before.velocities[i])*interpolation + point_before.velocities[i] : 0;
            accel = has_acceleration_data ?
                  (point_after.accelerations[i] - point_before.accelerations[i])*interpolation + point_before.accelerations[i] : 0;
          }
          else
          {
            vel = has_velocity_data ? point_after.velocities[i] : 0;
            accel = has_acceleration_data ? point_after.accelerations[i] : 0;
          }
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
      }
    }
  }
  else
  {
    ROS_DEBUG("No stored plan, just returning input state...");
    robot_state::robotStateToRobotStateMsg(*start_state, rs);
    rs.joint_state.header.stamp = ros::Time(0);
  }
  return true;
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

} //namespace
