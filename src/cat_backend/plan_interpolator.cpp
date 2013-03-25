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
    double request_duration = duration.toSec();

    int before=0, after=0;
    double blend = 1.0;
    trajectory_->findWayPointIndicesForDurationAfterStart(request_duration, before, after, blend);

    //logInform("Time is %.3f of the way between index %d and %d. Rounding up to index %d.", blend, before, after, after);
    *start_state = trajectory_->getWayPoint(after);
    const std::vector<robot_state::JointState*> &jsv_in = trajectory_->getWayPoint(after).getJointStateVector();
    const std::vector<robot_state::JointState*> &jsv_out = start_state->getJointStateVector();
    for(int i = 0; i < jsv_out.size(); i++)
      jsv_out[i]->getVelocities() = jsv_in[i]->getVelocities();

    ros::Time temp_time = start_time_ +
        ros::Duration(trajectory_->getWaypointDurationFromStart(after) - 0.0001) ;
    // magic number is to try to avoid double comparison issues later
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
