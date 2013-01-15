/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Adam Leeper */

#include "cat_backend/cat_backend.h"
#include "moveit/kinematic_state/conversions.h"

#include <moveit/kinematic_constraints/utils.h>


namespace cat {

class CatBackend::DynamicReconfigureImpl
{
public:

  DynamicReconfigureImpl(CatBackend *owner) : owner_(owner),
    dynamic_reconfigure_server_(ros::NodeHandle("~/cat_backend"))
  {
    dynamic_reconfigure_server_.setCallback(boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }

private:

  void dynamicReconfigureCallback(cat_backend::BackendConfig &config, uint32_t level)
  {
    // do some stuff
    //ROS_ERROR("Dynamic reconfigure callback has not been implemented, but we are just copying the default configuration!");
    owner_->config_ = config;
    //if(level == 1)
    owner_->changedPlanningGroup();

    if(level == 16)
    {
      ROS_INFO("Teleop mode changed to %s", owner_->modeToStr(owner_->config_.teleop_mode).c_str());
      if(owner_->config_.teleop_mode != cat_backend::Backend_TELEOP_DISABLE)
        owner_->addBackgroundJob(boost::bind(&CatBackend::computeTeleopUpdate, owner_));
    }

  }

  CatBackend *owner_;
  dynamic_reconfigure::Server<cat_backend::BackendConfig> dynamic_reconfigure_server_;
};

} // namespace cat

// - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - - - - - - - - - - -

cat::CatBackend::CatBackend(bool debug)
  :
    node_handle_("~"),
    allow_trajectory_execution_(true)
{
  background_process_.setCompletionEvent(boost::bind(&CatBackend::backgroundJobCompleted, this));

  tfl_.reset(new tf::TransformListener());
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(move_group::ROBOT_DESCRIPTION, tfl_));

  if (planning_scene_monitor_->getPlanningScene() && planning_scene_monitor_->getPlanningScene()->isConfigured())
  {
    ROS_INFO("Starting world, scene, and state monitor.");
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
  }

  // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
  node_handle_.param("allow_trajectory_execution", allow_trajectory_execution_, true);

  plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, !allow_trajectory_execution_));
  pick_place_.reset(new pick_place::PickPlace(plan_execution_->getPlanningPipeline()));

  // configure the planning pipeline
  plan_execution_->getPlanningPipeline()->displayComputedMotionPlans(true);
  plan_execution_->getPlanningPipeline()->checkSolutionPaths(true);

  if (debug)
  {
    plan_execution_->displayCostSources(true);
    plan_execution_->getPlanningPipeline()->publishReceivedRequests(true);
  }

  // publishers for goal state:
  publish_goal_state_ = root_node_handle_.advertise<moveit_msgs::PlanningScene>("goal_state_scene", 1);

  reconfigure_impl_ = new DynamicReconfigureImpl(this);

  planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor_->UPDATE_SCENE);


  robot_interaction_.reset(new robot_interaction::RobotInteraction(getKinematicModel()));

  kinematic_state::KinematicStatePtr ks(new kinematic_state::KinematicState(getPlanningSceneRO()->getCurrentState()));
  query_goal_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("goal", *ks, planning_scene_monitor_->getTFClient()));
  query_goal_state_->setUpdateCallback(boost::bind(&CatBackend::onQueryGoalStateUpdate, this));
  query_goal_state_->setStateValidityCallback(boost::bind(&CatBackend::isIKSolutionCollisionFree, this, _1, _2));

  // Now we actually go...
  ROS_INFO("Finsihing constructor...");
  changedPlanningGroup();
}

cat::CatBackend::~CatBackend(void)
{
  // Nothing
}

std::string cat::CatBackend::modeToStr(int mode)
{
  switch (mode)
  {
  case cat_backend::Backend_TELEOP_DISABLE:
    return "TELEOP_DISABLE";
  case cat_backend::Backend_TELEOP_JT:
    return "TELEOP_JT";
  case cat_backend::Backend_TELEOP_IK:
    return "TELEOP_IK";
  case cat_backend::Backend_TELEOP_MP:
    return "TELEOP_MP";
  case cat_backend::Backend_TELEOP_CVX:
    return "TELEOP_CVX";
  default:
    return "UNKNOWN";
  }
}

void cat::CatBackend::backgroundJobCompleted(void)
{
  // Do nothing...
  //addMainLoopJob(boost::bind(&CatBackend::updateBackgroundJobProgressBar, this));
}

void cat::CatBackend::addBackgroundJob(const boost::function<void(void)> &job)
{
  background_process_.addJob(job);
  //addMainLoopJob(boost::bind(&MotionPlanningDisplay::updateBackgroundJobProgressBar, this));
}

void cat::CatBackend::addMainLoopJob(const boost::function<void(void)> &job)
{
  boost::mutex::scoped_lock slock(main_loop_jobs_lock_);
  main_loop_jobs_.push_back(job);
}

void cat::CatBackend::executeMainLoopJobs(void)
{
  main_loop_jobs_lock_.lock();
  while (!main_loop_jobs_.empty())
  {
    boost::function<void(void)> fn = main_loop_jobs_.front();
    main_loop_jobs_.pop_front();
    main_loop_jobs_lock_.unlock();
    try
    {
      fn();
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Exception caught executing main loop job: %s", ex.what());
    }
    catch(...)
    {
      ROS_ERROR("Exception caught executing main loop job");
    }
    main_loop_jobs_lock_.lock();
  }
  main_loop_jobs_lock_.unlock();
}

void cat::CatBackend::updateBackgroundJobProgressBar(void)
{
  std::size_t n = background_process_.getJobCount();

  // publish job count?
}

void cat::CatBackend::computeTeleopUpdate()
{
  ROS_ERROR("v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v");
  ros::Duration target_period = ros::Duration(config_.target_period);
  ros::Time update_start_time = ros::Time::now();

  switch(config_.teleop_mode)
  {
    case(cat_backend::Backend_TELEOP_JT):
      ROS_WARN("TELEOP_JT is not implemented!");
      break;
    case(cat_backend::Backend_TELEOP_IK):
      ROS_WARN("TELEOP_IK is not implemented!");
      break;
    case(cat_backend::Backend_TELEOP_MP):
      //ROS_WARN("TELEOP_MP is not implemented!");
      computeTeleopMPUpdate(target_period);
      break;
    case(cat_backend::Backend_TELEOP_CVX):
      ROS_WARN("TELEOP_CVX is not implemented!");
      break;
    case(cat_backend::Backend_TELEOP_DISABLE):
      ROS_WARN("It seems teleop was DISABLED sometime between the last queueing action and now!");
      break;
    default:
      ROS_ERROR("An unhandled teleop state was requested.");
  }

  ros::Duration time_used = ros::Time::now() - update_start_time;
  ros::Duration remaining_time = target_period - time_used;
  ROS_INFO("Time used: %.3f sec, sleeping for %.3f sec", time_used.toSec(), remaining_time.toSec());
  remaining_time.sleep();

  // When all is done, it gets ready to call itself again!
  if(config_.teleop_mode != cat_backend::Backend_TELEOP_DISABLE)
    addBackgroundJob(boost::bind(&CatBackend::computeTeleopUpdate, this));

}

void cat::CatBackend::computeTeleopMPUpdate(const ros::Duration &target_period)
{
  std::string group_name = getCurrentPlanningGroup();

  if (group_name.empty())
    return;

  kinematic_state::KinematicStatePtr future_start_state( new kinematic_state::KinematicState(getPlanningSceneMonitor()->getPlanningScene()->getCurrentState()));

  if(current_plan_ && current_plan_->trajectory_.joint_trajectory.points.size() >= 1)
  {
    size_t trajectory_index = current_plan_->trajectory_.joint_trajectory.points.size() - 1;
    for( ; trajectory_index >=0; trajectory_index--)
    {
      ROS_INFO("Trajectory index %zd of %zd. Time: %.3f vs. %.3f ",
               trajectory_index,
               current_plan_->trajectory_.joint_trajectory.points.size(),
               current_plan_->trajectory_.joint_trajectory.points[trajectory_index].time_from_start.toSec(),
               target_period.toSec());
      if( current_plan_->trajectory_.joint_trajectory.points[trajectory_index].time_from_start < target_period )
        break;
      // maybe need to bake in the delay between cycles too...
    }
    trajectory_index = std::min(trajectory_index + 1, current_plan_->trajectory_.joint_trajectory.points.size() - 1);
//    ROS_INFO("There are %zd joint names and %zd joint positions",
//             current_plan_->trajectory_.joint_trajectory.joint_names.size(),
//             current_plan_->trajectory_.joint_trajectory.points[trajectory_index].positions.size() );
    ROS_INFO("Using index %zd", trajectory_index);
    future_start_state->setStateValues(current_plan_->trajectory_.joint_trajectory.joint_names, current_plan_->trajectory_.joint_trajectory.points[trajectory_index].positions);
  }

  moveit_msgs::MotionPlanRequest mreq;
  mreq.allowed_planning_time = target_period*0.75;
  mreq.num_planning_attempts = 1;
  kinematic_state::kinematicStateToRobotState(*future_start_state, mreq.start_state);
  mreq.group_name = group_name;

//  mreq.goal_constraints.resize(1);
//  mreq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(link_name, goal_pose, goal_tolerance_);
  mreq.goal_constraints.resize(1);
  mreq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(getQueryGoalState()->getJointStateGroup(group_name), config_.goal_tolerance);
  //mreq.workspace_parameters = workspace_parameters_;

  plan_execution_->planOnly(mreq);
  plan_execution::PlanExecution::Result result = plan_execution_->getLastResult();
  if(result.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR("Planning failed for some reason...");
    current_plan_.reset();
    return;
  }

  plan_execution_->getTrajectoryExecutionManager()->pushAndExecute(result.planned_trajectory_); // TODO should specify the controller huh?
  current_plan_.reset(new move_group_interface::MoveGroup::Plan());
  current_plan_->trajectory_ = result.planned_trajectory_;
  current_plan_->start_state_ = mreq.start_state;

  //move_group_->move();
}

void cat::CatBackend::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  workspace_parameters_.header.frame_id = getKinematicModel()->getModelFrame();
  workspace_parameters_.header.stamp = ros::Time::now();
  workspace_parameters_.min_corner.x = minx;
  workspace_parameters_.min_corner.y = miny;
  workspace_parameters_.min_corner.z = minz;
  workspace_parameters_.max_corner.x = maxx;
  workspace_parameters_.max_corner.y = maxy;
  workspace_parameters_.max_corner.z = maxz;
}

void cat::CatBackend::onQueryGoalStateUpdate()
{
  //ROS_INFO("Publishing a goal state update!");
  moveit_msgs::PlanningScene gs;
  kinematic_state::kinematicStateToRobotState(*getQueryGoalState(), gs.robot_state);
  publish_goal_state_.publish(gs);
}

bool cat::CatBackend::isIKSolutionCollisionFree(kinematic_state::JointStateGroup *group, const std::vector<double> &ik_solution) const
{
  if ( planning_scene_monitor_)
  {
    group->setVariableValues(ik_solution);
    return !getPlanningSceneRO()->isStateColliding(*group->getKinematicState(), group->getName());
  }
  else
    return true;
}

void cat::CatBackend::publishInteractiveMarkers(void)
{
  if (robot_interaction_)
  {
    ROS_INFO("Publishing interactive markers...");
    robot_interaction_->clearInteractiveMarkers();
    //if ( config_.query_goal_state )
    if(true)
      robot_interaction_->addInteractiveMarkers(query_goal_state_, config_.marker_scale); // TODO magic number
    robot_interaction_->publishInteractiveMarkers();
  }
}

void cat::CatBackend::updateQueryGoalState(void)
{
  publishInteractiveMarkers();
//  addMainLoopJob(boost::bind(&MotionPlanningDisplay::changedQueryGoalState, this));
//  context_->queueRender();
}

void cat::CatBackend::setQueryGoalState(const kinematic_state::KinematicStatePtr &goal)
{
  query_goal_state_->setState(*goal);
  updateQueryGoalState();
}

std::string cat::CatBackend::getCurrentPlanningGroup(void) const
{
  return config_.planning_group;
}

void cat::CatBackend::changedPlanningGroup(void)
{
  std::string group = config_.planning_group;
  ROS_INFO("Changing to group [%s]", group.c_str());
  if (!group.empty())
    if (!getKinematicModel()->hasJointModelGroup(group))
    {
      ROS_ERROR("Didn't find JointModelGroup for group [%s]", group.c_str());
      config_.planning_group = "";
      return;
    }

  if (robot_interaction_)
  {
    ROS_INFO("Deciding active components for group [%s]", group.c_str());
    robot_interaction_->decideActiveComponents(group);
  }
  publishInteractiveMarkers();
}


const planning_scene_monitor::PlanningSceneMonitorPtr& cat::CatBackend::getPlanningSceneMonitor(void)
{
  return planning_scene_monitor_;
}

const kinematic_model::KinematicModelConstPtr& cat::CatBackend::getKinematicModel(void)
{
  if (planning_scene_monitor_)
    return planning_scene_monitor_->getKinematicModel();
  else
  {
    static kinematic_model::KinematicModelConstPtr empty;
    return empty;
  }
}

planning_scene_monitor::LockedPlanningSceneRO cat::CatBackend::getPlanningSceneRO(void) const
{
  return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
}

planning_scene_monitor::LockedPlanningSceneRW cat::CatBackend::getPlanningSceneRW(void)
{
  return planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_);
}


//} // namespace cat
