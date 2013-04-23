#ifndef PLAN_INTERPOLATOR_H
#define PLAN_INTERPOLATOR_H


//#include <moveit/move_group/names.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/console.h>


namespace plan_interpolator
{

class PlanInterpolator
{
public:
  PlanInterpolator() : has_new_plan_(false), plan_is_valid_(false) {}
  ~PlanInterpolator(){}

  bool getStateAtTime(const ros::Time &request_time,
                      bool do_interpolate,
                      robot_state::RobotStatePtr& start_state,
                      ros::Time& actual_time);

  void printPlan();

  void clearPlan()
  {
    has_new_plan_ = false;
    plan_is_valid_ = false;
  }

  //void setPlan(const move_group_interface::::Plan& plan)
  void setPlan(robot_trajectory::RobotTrajectoryPtr& robot_traj, const ros::Time& start_time)
  {
    trajectory_ = robot_traj;
    start_time_ = start_time;
    has_new_plan_ = true;
    plan_is_valid_ = true;
  }

  bool hasNewPlan()
  {
    return has_new_plan_;
  }

  void setPlanAsOld()
  {
    has_new_plan_ = false;
  }

  //const move_group_interface::MoveGroup::Plan& getPlan()
  const robot_trajectory::RobotTrajectory& getPlan()
  {
    return *trajectory_;
  }

  moveit_msgs::RobotTrajectory getPlanAsMsg()
  {
    moveit_msgs::RobotTrajectory msg;
    trajectory_->getRobotTrajectoryMsg(msg);
    msg.joint_trajectory.header.stamp = start_time_;
    return msg;
  }

  bool has_new_plan_;
  bool plan_is_valid_;
  //boost::shared_ptr<move_group_interface::MoveGroup::Plan> current_plan_;
  robot_trajectory::RobotTrajectoryPtr trajectory_;
  ros::Time start_time_;

};

} // namespace

#endif // PLAN_INTERPOLATOR_H
