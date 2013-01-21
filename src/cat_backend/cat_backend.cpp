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
    cat_backend::BackendConfig old_config = owner_->config_ ;
    owner_->config_ = config;

    if(level == 1)
      owner_->changedPlanningGroup();

    if(owner_->query_goal_state_)
    {
      owner_->query_goal_state_->setIKAttempts(config.ik_attempts);
      owner_->query_goal_state_->setIKTimeout(config.ik_timeout);
      owner_->query_goal_state_->setInteractionMode( (config.ik_type == cat_backend::Backend_POSITION_IK) ?
                                               robot_interaction::RobotInteraction::InteractionHandler::POSITION_IK
                                             : robot_interaction::RobotInteraction::InteractionHandler::VELOCITY_IK);
    }

    if(level == 16)
    {
      ROS_INFO("Teleop mode changed to %s", owner_->modeToStr(owner_->config_.teleop_mode).c_str());
      if(old_config.teleop_mode != cat_backend::Backend_TELEOP_JT && config.teleop_mode == cat_backend::Backend_TELEOP_JT)
      {
        // Switch to JT controller
        if(owner_->trajectory_execution_manager_)
        {
          std::vector<std::string> list;
          owner_->trajectory_execution_manager_->getControllerManager()->getControllersList(list);
          for(int i = 0; i < list.size(); i++) ROS_INFO("Listed controller: %s", list[i].c_str());
          list.clear();
          owner_->trajectory_execution_manager_->getControllerManager()->getLoadedControllers(list);
          for(int i = 0; i < list.size(); i++) ROS_INFO("Loaded controller: %s", list[i].c_str());
          list.clear();
          owner_->trajectory_execution_manager_->getControllerManager()->getActiveControllers(list);
          for(int i = 0; i < list.size(); i++) ROS_INFO("Active controller: %s", list[i].c_str());
          list.clear();
        }
      }
      if(old_config.teleop_mode == cat_backend::Backend_TELEOP_JT && config.teleop_mode != cat_backend::Backend_TELEOP_JT)
      {
        // Switch to Joint Controller
      }
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
  // ===== Dynamic Reconfigure houses some of the parameters used below =====
  reconfigure_impl_ = new DynamicReconfigureImpl(this);
  background_process_.setCompletionEvent(boost::bind(&CatBackend::backgroundJobCompleted, this));

  // ===== Planning Scene =====
  ROS_INFO("CAT backend: Initializing planning scene monitor");
  tfl_.reset(new tf::TransformListener());
  std::string robot_description_name;
  node_handle_.param("robot_description_name", robot_description_name, std::string("robot_description"));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_name, tfl_));
  if (planning_scene_monitor_->getPlanningScene() && planning_scene_monitor_->getPlanningScene()->isConfigured())
  {
    planning_scene_monitor_->startWorldGeometryMonitor();
    //planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor_->UPDATE_SCENE);
  }

  // ===== Planning =====
  ROS_INFO("CAT backend: Initializing planning pipelines");
  const kinematic_model::KinematicModelConstPtr model = planning_scene_monitor_->getKinematicModel();
  if(!model)
    ROS_ERROR("The model is broken!");
  ompl_planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getKinematicModel()));
  cat_planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getKinematicModel(),
                                                              "cat_planning_plugin", "cat_request_adapters"));
  if (debug)
  {
    ROS_INFO("CAT backend: Configuring planners in debug mode");
    ompl_planning_pipeline_->publishReceivedRequests(true);
    ompl_planning_pipeline_->displayComputedMotionPlans(true);
    ompl_planning_pipeline_->checkSolutionPaths(true);
  }

  // ===== Execution =====
  node_handle_.param("allow_trajectory_execution", allow_trajectory_execution_, true);
  if (allow_trajectory_execution_)
  {
    ROS_INFO("CAT backend: Initializing trajectory execution manager");
    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(planning_scene_monitor_->getKinematicModel()));
  }

  // ===== Visualization =====
  ROS_INFO("CAT backend: Initializing robot interaction tools");
  publish_goal_state_ = root_node_handle_.advertise<moveit_msgs::PlanningScene>("goal_state_scene", 1);
  robot_interaction_.reset(new robot_interaction::RobotInteraction(getKinematicModel(), "cat_backend"));
  kinematic_state::KinematicStatePtr ks(new kinematic_state::KinematicState(getPlanningSceneRO()->getCurrentState()));
  last_goal_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("last_goal", *ks, planning_scene_monitor_->getTFClient()));
  query_goal_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("goal", *ks, planning_scene_monitor_->getTFClient()));
  query_goal_state_->setUpdateCallback(boost::bind(&CatBackend::onQueryGoalStateUpdate, this, _1));
  query_goal_state_->setStateValidityCallback(boost::bind(&CatBackend::isIKSolutionCollisionFree, this, _1, _2));
  query_goal_state_->setMeshesVisible(true);
  query_goal_state_->setIKAttempts(config_.ik_attempts);
  query_goal_state_->setIKTimeout(config_.ik_timeout);
  query_goal_state_->setInteractionMode( (config_.ik_type == cat_backend::Backend_POSITION_IK) ?
                                           robot_interaction::RobotInteraction::InteractionHandler::POSITION_IK
                                         : robot_interaction::RobotInteraction::InteractionHandler::VELOCITY_IK);

  // Now we actually go...
  changedPlanningGroup();
  ROS_DEBUG("CAT backend initialized!");
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

void cat::CatBackend::computeTeleopUpdate()
{
  ROS_ERROR("v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v");
  ros::Duration target_period = ros::Duration(config_.target_period);
  ros::Time update_start_time = ros::Time::now();

  //planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);

  try
  {
    switch(config_.teleop_mode)
    {
      case(cat_backend::Backend_TELEOP_JT):
        ROS_WARN("TELEOP_JT is not implemented!");
        break;
      case(cat_backend::Backend_TELEOP_IK):
        computeTeleopIKUpdate(target_period);
        break;
      case(cat_backend::Backend_TELEOP_MP):
        //ROS_WARN("TELEOP_MP is not implemented!");
        computeTeleopMPUpdate(target_period);
        break;
      case(cat_backend::Backend_TELEOP_CVX):
        computeTeleopCVXUpdate(target_period);
        break;
      case(cat_backend::Backend_TELEOP_DISABLE):
        ROS_WARN("It seems teleop was DISABLED sometime between the last queueing action and now!");
        break;
      default:
        ROS_ERROR("An unhandled teleop state was requested.");
    }
  }
  catch(...)
  {
    ROS_ERROR("Caught some kind of exception...");
  }

  ros::Duration time_used = ros::Time::now() - update_start_time;
  ros::Duration remaining_time = target_period - time_used;
  if(remaining_time.toSec() < 0.0)
  {
    remaining_time = ros::Duration(0.0);
    ROS_ERROR("Time used: %.3f sec exceeded target period (%.3f sec)", time_used.toSec(), target_period.toSec());
  }
  else
    ROS_INFO("Time used: %.3f sec, sleeping for %.3f sec", time_used.toSec(), remaining_time.toSec());
  remaining_time.sleep();

  // When all is done, it gets ready to call itself again!
  if(config_.teleop_mode != cat_backend::Backend_TELEOP_DISABLE)
    addBackgroundJob(boost::bind(&CatBackend::computeTeleopUpdate, this));
}

// =============================================================================
// =============================== CAT Planners ================================
// =============================================================================
void cat::CatBackend::computeTeleopCVXUpdate(const ros::Duration &target_period)
{
  ros::Time future_time = ros::Time::now() + target_period;
  ROS_DEBUG("TeleopCVXUpdate!");
  std::string group_name = getCurrentPlanningGroup();
  if (group_name.empty())
    return;

  const std::vector<robot_interaction::RobotInteraction::EndEffector>& aee = robot_interaction_->getActiveEndEffectors();
  if(aee.size() == 0)
  {
    ROS_WARN("No active end-effector, so can't do CVX update!");
    return;
  }

  if(aee.size() > 1)
    ROS_WARN("There are %zd active end-effectors, only handling the first one... (this will probably cause a crash in the planner)", aee.size());

  // TODO add an estimated velocity correction so that the arm moves smoothly!
  kinematic_state::KinematicStatePtr future_start_state( new kinematic_state::KinematicState(getPlanningSceneRO()->getCurrentState()));

  moveit_msgs::MotionPlanRequest req;
  geometry_msgs::PoseStamped goal_pose;
  getQueryGoalStateHandler()->getLastEndEffectorMarkerPose(aee[0], goal_pose);
  req.group_name = group_name;
  psi_.getStateAtTime( future_time, future_start_state, req.start_state);
  req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(aee[0].parent_link, goal_pose));
//    req.motion_plan_request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(future_start_state->getJointStateGroup(group_name),
//                                                                                                       .001, .001));
  req.num_planning_attempts = config_.velocity_constraint_only + 1;
  req.allowed_planning_time = target_period*0.5;  // TODO mgic number!

  moveit_msgs::MotionPlanResponse res;
  generatePlan(cat_planning_pipeline_, req, res, future_time);

  // ==========================================
  // Send out last plan for execution.
  // It is stamped with a time in the future, so it should be ok to send it now.
  if(psi_.hasNewPlan())
  {
    ROS_DEBUG_NAMED("cat_backend", "Sending most recent plan for execution.");
    trajectory_execution_manager_->pushAndExecute(psi_.getPlan().trajectory_); // TODO should specify the controller huh?
    psi_.setPlanAsOld();
  }
  else
  {
    ROS_DEBUG("No plan saved, not executing anything.");
  }

  ROS_DEBUG("Done with TeleopCVXUpdate");
}

// ============================================================================
// ============================ Inverse Kinematics ============================
// ============================================================================
void cat::CatBackend::computeTeleopIKUpdate(const ros::Duration &target_period)
{
  ROS_DEBUG("TeleopIKUpdate!");
  std::string group_name = getCurrentPlanningGroup();

  if (group_name.empty())
    return;

  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory.points.resize(1);
  traj.joint_trajectory.points[0].time_from_start = ros::Duration(0, 0);
  {
    last_goal_state_lock_.lock();
    const std::vector<kinematic_state::JointState*>& jsv = last_goal_state_->getState()->getJointStateGroup(group_name)->getJointStateVector();
    traj.joint_trajectory.joint_names.resize(jsv.size());
    traj.joint_trajectory.points[0].positions.resize(jsv.size());
    traj.joint_trajectory.points[0].velocities.resize(jsv.size());
    traj.joint_trajectory.header.frame_id = last_goal_state_->getState()->getKinematicModel()->getModelFrame();
    traj.joint_trajectory.header.stamp = ros::Time(0);

    // Now actually populate the joints...
    for(int i = 0; i < jsv.size(); i++ )
    {
      kinematic_state::JointState* js = jsv[i];
      traj.joint_trajectory.joint_names[i] = js->getName();
      traj.joint_trajectory.points[0].positions[i] = js->getVariableValues()[0];
      traj.joint_trajectory.points[0].velocities[i] = 0.0;  // TODO could we estimate velocity?
    }
    last_goal_state_lock_.unlock();
  }
  trajectory_execution_manager_->pushAndExecute(traj); // TODO should specify the controller huh?
  ROS_DEBUG("Done with TeleopIKUpdate");
}

// ============================================================================
// ============================= Motion Planning ==============================
// ============================================================================
void cat::CatBackend::computeTeleopMPUpdate(const ros::Duration &target_period)
{
  ros::Time future_time = ros::Time::now() + target_period;
  ROS_DEBUG("TeleopMPUpdate!");
  std::string group_name = getCurrentPlanningGroup();
  if (group_name.empty())
    return;

  kinematic_state::KinematicStatePtr future_start_state( new kinematic_state::KinematicState(getPlanningSceneRO()->getCurrentState()));

  ROS_DEBUG("Formulating planning request");
  moveit_msgs::MotionPlanRequest req;
  req.allowed_planning_time = target_period*0.75;
  req.num_planning_attempts = 1;
  psi_.getStateAtTime( future_time, future_start_state, req.start_state);
  req.group_name = group_name;

  ROS_DEBUG("Constructing goal constraint...");
  req.goal_constraints.resize(1);
  last_goal_state_lock_.lock();
  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(last_goal_state_->getState()->getJointStateGroup(group_name), config_.goal_tolerance);
  last_goal_state_lock_.unlock();

  moveit_msgs::MotionPlanResponse result;
  generatePlan(ompl_planning_pipeline_, req, result, future_time);

  // ==========================================
  // Send out last plan for execution.
  // It is stamped with a time in the future, so it should be ok to send it now.
  if(psi_.hasNewPlan())
  {
    trajectory_execution_manager_->pushAndExecute(psi_.getPlan().trajectory_); // TODO should specify the controller huh?
    psi_.setPlanAsOld();
  }
  else
  {
    ROS_DEBUG("No plan saved, not executing anything.");
  }
  ROS_DEBUG("Done with TeleopMPUpdate");
}

bool cat::CatBackend::generatePlan(const planning_pipeline::PlanningPipelinePtr &pipeline,
                                   moveit_msgs::MotionPlanRequest& req,
                                   moveit_msgs::MotionPlanResponse &res,
                                   const ros::Time& future_time_limit)
{
  bool solved = false;
  try
  {
    // lock the planning scene in this scope
    planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
    ROS_DEBUG_NAMED("cat_backend", "Issuing planning request.");
    solved = cat_planning_pipeline_->generatePlan(planning_scene_monitor_->getPlanningScene(), req, res);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
  }
  catch(...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
  }

  if(solved)
  {
    if(future_time_limit < ros::Time::now() + ros::Duration(0.001)) // TODO this offset is a (vetted) magic number...
    {
      ROS_WARN("Planning took too long, discarding result.");
    }
    else
    {
      ROS_DEBUG("Planning SUCCESS, saving plan.");
      move_group_interface::MoveGroup::Plan plan;
      plan.trajectory_ = res.trajectory;
      plan.start_state_ = req.start_state;
      plan.trajectory_.joint_trajectory.header.stamp = future_time_limit;
      psi_.setPlan(plan);
    }
  }
  else
  {
    ROS_WARN("Planning FAILED, not saving anything.");
  }
  return solved;
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

void cat::CatBackend::onQueryGoalStateUpdate(robot_interaction::RobotInteraction::InteractionHandler* ih)
{
  const std::vector<robot_interaction::RobotInteraction::EndEffector>& aee = robot_interaction_->getActiveEndEffectors();
  bool in_error = false;
  for(int i=0; i < aee.size(); i++)
  {
    in_error = ih->inError(aee[i]) || in_error;
  }
  //ROS_INFO("Publishing a goal state update!");
  moveit_msgs::PlanningScene gs;
  if(!in_error)
  {
    last_goal_state_lock_.lock();
    last_goal_state_->setState(*query_goal_state_->getState());
    kinematic_state::kinematicStateToRobotState(*last_goal_state_->getState(), gs.robot_state);
    last_goal_state_lock_.unlock();
    publish_goal_state_.publish(gs);
  }
  else
  {
    // Do nothing?
  }
  updateInactiveGroupsFromCurrentRobot();
  robot_interaction_->updateInteractiveMarkerProperties(getQueryGoalStateHandler());
}

void cat::CatBackend::updateInactiveGroupsFromCurrentRobot()
{
  ROS_DEBUG("Updating inactive groups!");
  kinematic_state::KinematicState new_state = getPlanningSceneRO()->getCurrentState();
  kinematic_state::JointStateGroup* jsg = query_goal_state_->getState()->getJointStateGroup(getCurrentPlanningGroup());
  if(jsg)
  {
    //ROS_INFO("Setting values for group %s", jsg->getName().c_str());
    const std::vector<kinematic_state::JointState*>& jsv =  jsg->getJointStateVector();
    for(size_t i=0; i < jsv.size(); i++)
    {
      //("Setting values for joint %s", jsv[i]->getName().c_str());
      new_state.getJointState(jsv[i]->getName())->setVariableValues(jsv[i]->getVariableValues());
    }
  }
  query_goal_state_->setState(new_state);
}

bool cat::CatBackend::isIKSolutionCollisionFree(kinematic_state::JointStateGroup *group, const std::vector<double> &ik_solution) const
{
  if ( config_.collision_aware_ik && planning_scene_monitor_)
  {
    group->setVariableValues(ik_solution);
    return !getPlanningSceneRO()->isStateColliding(*group->getKinematicState(), group->getName());
  }
  else
    return true;
}

void cat::CatBackend::clearAndRenewInteractiveMarkers(void)
{
  if (robot_interaction_)
  {
    ROS_INFO("Clearing and republishing robot interaction markers.");
    robot_interaction_->clearInteractiveMarkers();
    robot_interaction_->addInteractiveMarkers(query_goal_state_, config_.marker_scale); // TODO magic number
    robot_interaction_->publishInteractiveMarkers();
  }
}

//void cat::CatBackend::updateQueryGoalState(void)
//{
//  publishInteractiveMarkers();
////  addMainLoopJob(boost::bind(&MotionPlanningDisplay::changedQueryGoalState, this));
////  context_->queueRender();
//}

//void cat::CatBackend::setQueryGoalState(const kinematic_state::KinematicStatePtr &goal)
//{
//  query_goal_state_->setState(*goal);
//  updateQueryGoalState();
//}

std::string cat::CatBackend::getCurrentPlanningGroup(void) const
{
  return config_.planning_group;
}

void cat::CatBackend::changedPlanningGroup(void)
{
  std::string group = config_.planning_group;
  if (!group.empty())
    if (!getKinematicModel()->hasJointModelGroup(group))
    {
      ROS_ERROR("Didn't find JointModelGroup for group [%s]", group.c_str());
      config_.planning_group = "";
      return;
    }

  if (robot_interaction_)
  {
    ROS_INFO("Changing to group [%s]", group.c_str());
    robot_interaction_->decideActiveComponents(group);
    clearAndRenewInteractiveMarkers();
  }
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

// =====================================================================================
// ======================== Stuff that may never get used ==============================
// =====================================================================================

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
