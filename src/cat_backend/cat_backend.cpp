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
    ROS_ERROR("Dynamic reconfigure callback has not been implemented, but we are just copying the default configuration!");
    owner_->config_ = config;
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
  tfl_.reset(new tf::TransformListener());
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(move_group::ROBOT_DESCRIPTION, tfl_));

  if (planning_scene_monitor_->getPlanningScene() && planning_scene_monitor_->getPlanningScene()->isConfigured())
  {
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

//  // start the service servers
//  plan_service_ = root_node_handle_.advertiseService(PLANNER_SERVICE_NAME, &MoveGroupServer::computePlanService, this);
//  execute_service_ = root_node_handle_.advertiseService(EXECUTE_SERVICE_NAME, &MoveGroupServer::executeTrajectoryService, this);
//  query_service_ = root_node_handle_.advertiseService(QUERY_SERVICE_NAME, &MoveGroupServer::queryInterface, this);

//  // start the move action server
//  move_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(root_node_handle_, move_group::MOVE_ACTION,
//                                                                                            boost::bind(&MoveGroupServer::executeMoveCallback, this, _1), false));
//  move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupServer::preemptMoveCallback, this));
//  move_action_server_->start();

//  // start the pickup action server
//  pickup_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::PickupAction>(root_node_handle_, move_group::PICKUP_ACTION,
//                                                                                           boost::bind(&MoveGroupServer::executePickupCallback, this, _1), false));
//  pickup_action_server_->registerPreemptCallback(boost::bind(&MoveGroupServer::preemptPickupCallback, this));
//  pickup_action_server_->start();


  // don't need to publish the start or goal...

  reconfigure_impl_ = new DynamicReconfigureImpl(this);

  planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor_->UPDATE_SCENE);


  kinematic_state::KinematicStatePtr ks(new kinematic_state::KinematicState(getPlanningSceneRO()->getCurrentState()));
  query_goal_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("goal", *ks, planning_scene_monitor_->getTFClient()));
  //query_goal_state_->setUpdateCallback(boost::bind(&CatBackend::drawQueryGoalState, this));
  query_goal_state_->setStateValidityCallback(boost::bind(&CatBackend::isIKSolutionCollisionFree, this, _1, _2));

  // Now we actually go...
  ROS_INFO("Finsihing constructor...");
  changedPlanningGroup();
}

cat::CatBackend::~CatBackend(void)
{
  // Nothing
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
    robot_interaction_->clearInteractiveMarkers();
    //if ( config_.query_goal_state )
    if(true)
      robot_interaction_->addInteractiveMarkers(query_goal_state_, 1);
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
    robot_interaction_->decideActiveComponents(group);
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
