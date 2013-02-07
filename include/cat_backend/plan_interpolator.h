#ifndef PLAN_INTERPOLATOR_H
#define PLAN_INTERPOLATOR_H


#include <moveit/move_group/names.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/console.h>

namespace plan_interpolator
{

class PlanInterpolator
{
public:
  PlanInterpolator() : has_new_plan_(false), plan_is_valid_(false) {}
  ~PlanInterpolator(){}

  void findIndexAtTimeFromStart(const ros::Duration& time, int& before, int& after, double &interpolate);

  bool getStateAtTime(const ros::Time &request_time, robot_state::RobotStatePtr& start_state,
                      moveit_msgs::RobotState& rs, bool do_interpolate);

  void printPlan();

  void clearPlan()
  {
    has_new_plan_ = false;
    plan_is_valid_ = false;
  }

  void setPlan(const move_group_interface::MoveGroup::Plan& plan)
  {
    current_plan_.reset( new move_group_interface::MoveGroup::Plan(plan));
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

  const move_group_interface::MoveGroup::Plan& getPlan()
  {
    return *current_plan_;
  }

  bool has_new_plan_;
  bool plan_is_valid_;
  boost::shared_ptr<move_group_interface::MoveGroup::Plan> current_plan_;

};

} // namespace

#endif // PLAN_INTERPOLATOR_H
