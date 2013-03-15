#include <cat_backend/plan_interpolator.h>


namespace plan_interpolator
{

bool PlanInterpolator::getStateAtTime(const ros::Time &request_time,
                                      bool do_interpolate,
                                      robot_state::RobotStatePtr& start_state,
                                      ros::Time& actual_time)
{
  ros::Duration duration = request_time - start_time_;
  if ( duration.toSec() < 0 )
    return false;

  //TODO this is a bad check, we need to differtiate between "too early" and "not valid"
  if( plan_is_valid_ && trajectory_ ) // && current_plan_->trajectory_.joint_trajectory.points.size() >= 2)
  {

    double actual_duration = duration.toSec();

    trajectory_->getStateAtDurationFromStart(actual_duration, do_interpolate, start_state, actual_duration);
    ros::Time temp_time = start_time_ + ros::Duration(actual_duration - 0.000001) ; // avoid double comparison issues later
    if(request_time < temp_time)
      actual_time = temp_time;
    else
      actual_time = request_time;
  }
  return true;
}

void PlanInterpolator::printPlan()
{
//  if(current_plan_)
//  {
//    std::string valid_plan = plan_is_valid_ ? "valid" : "not valid";
//    std::string new_plan   = has_new_plan_  ? "unused" : "already used";
//    ROS_INFO("Current plan is %s and %s.", valid_plan.c_str(), new_plan.c_str());
//    ROS_INFO_STREAM(current_plan_->trajectory_);
//  }
}

} //namespace
