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
#ifndef _CAT_BACKEND_
#define _CAT_BACKEND_

#include <cat_backend/background_processing.h>
#include <cat_backend/BackendConfig.h>
#include <cat_backend/plan_interpolator.h>

#include <moveit/robot_interaction/robot_interaction.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group/names.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/kinematic_state/conversions.h>

//#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_listener.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/pick_place/pick_place.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>

namespace cat
{

// ***************************************************************************************
class CatBackend
{
public:
  CatBackend(bool debug);
  ~CatBackend(void);

  const kinematic_model::KinematicModelConstPtr& getKinematicModel(void);
  planning_scene_monitor::LockedPlanningSceneRO getPlanningSceneRO(void) const;
  planning_scene_monitor::LockedPlanningSceneRW getPlanningSceneRW(void);
  const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor(void);

  // pass the execution of this function call to a separate thread that runs in the background
  void addTeleopJob(const boost::function<void(void)> &job);
  void addPerceptionJob(const boost::function<void(void)> &job);

  // queue the execution of this function for the next time the main update() loop gets called
  void addMainLoopJob(const boost::function<void(void)> &job);


protected: // methods

  void clearAndRenewInteractiveMarkers(void);
  void changedPlanningGroup(void);
  bool isIKSolutionCollisionFree(kinematic_state::JointStateGroup *group, const std::vector<double> &ik_solution) const;

  void executeMainLoopJobs(void);
  void updateBackgroundJobProgressBar(void);
  void backgroundJobCompleted(void);

  void updatePlanningScene();

  void computeTeleopUpdate(const ros::Duration& target_period);
  void computeTeleopJTUpdate(const ros::Duration& target_period);
  void computeTeleopIKUpdate(const ros::Duration& target_period);
  void computeTeleopMPUpdate(const ros::Duration& target_period);
  void computeTeleopCVXUpdate(const ros::Duration& target_period);
  bool generatePlan(const planning_pipeline::PlanningPipelinePtr &pipeline,
                    moveit_msgs::MotionPlanRequest& req,
                    moveit_msgs::MotionPlanResponse &res,
                    const ros::Time& future_time_limit);  

  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz);

  std::string modeToStr(int mode);


//  const kinematic_state::KinematicStatePtr& getQueryStartState(void) const
//  {
//    return query_start_state_->getState();
//  }

//  const kinematic_state::KinematicStateConstPtr& getQueryGoalState(void) const
//  {
//    return query_goal_state_->getState();
//  }

  const robot_interaction::RobotInteractionPtr& getRobotInteraction(void) const
  {
    return robot_interaction_;
  }

//  const robot_interaction::RobotInteraction::InteractionHandlerPtr& getQueryStartStateHandler(void) const
//  {
//    return query_start_state_;
//  }

  const robot_interaction::RobotInteraction::InteractionHandlerPtr& getQueryGoalStateHandler(void) const
  {
    return query_goal_state_;
  }

//  void setQueryStartState(const kinematic_state::KinematicStatePtr &start);
//  void setQueryGoalState(const kinematic_state::KinematicStatePtr &goal);

//  void updateQueryStartState(void);
//  void updateQueryGoalState(void);

  void onQueryGoalStateUpdate(robot_interaction::RobotInteraction::InteractionHandler* ih, bool needs_refresh);

  void setAndPublishLastGoalState(const kinematic_state::KinematicStateConstPtr& state);

  void setAndPublishLastCurrentState(const sensor_msgs::JointStateConstPtr &joint_state);

  void onPlanningSceneMonitorUpdate(const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType& type);

  void updateInactiveGroupsFromCurrentRobot();

  std::string getCurrentPlanningGroup(void) const;

protected:
  void publishErrorMetrics();
  void timerCallback();


protected: // members

  // robot interaction
  robot_interaction::RobotInteractionPtr robot_interaction_;
  robot_interaction::RobotInteraction::InteractionHandlerPtr query_goal_state_;
  kinematic_state::KinematicStatePtr last_goal_state_;
  kinematic_state::KinematicStatePtr last_current_state_;


  boost::shared_ptr<tf::TransformListener> tfl_;

  ros::NodeHandle root_node_handle_;
  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr planning_scene_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  planning_pipeline::PlanningPipelinePtr ompl_planning_pipeline_;
  planning_pipeline::PlanningPipelinePtr cat_planning_pipeline_;
  bool allow_trajectory_execution_;

  ros::Publisher publish_goal_state_;
  ros::Publisher publish_current_state_;
  ros::Publisher publish_cartesian_goal_left_;
  ros::Publisher publish_cartesian_goal_right_;

  ros::Publisher publish_error_;


  BackgroundProcessing teleop_process_;
  BackgroundProcessing perception_process_;
  std::deque<boost::function<void(void)> > main_loop_jobs_;
  boost::mutex main_loop_jobs_lock_;
  boost::mutex last_goal_state_lock_;
  boost::mutex last_current_state_lock_;
  boost::mutex scene_lock_;
  ros::Time last_scene_update_time_;

  ros::Timer timer_;

  plan_interpolator::PlanInterpolator psi_;

  //trajectory_processing::IterativeParabolicTimeParameterization smoother_;


  moveit_msgs::WorkspaceParameters workspace_parameters_;

//  MoveGroupState move_state_;
//  MoveGroupState pickup_state_;

  class DynamicReconfigureImpl;
  DynamicReconfigureImpl *reconfigure_impl_;

  cat_backend::BackendConfig config_;

  boost::mutex teleop_lock_;

  int cycle_id_;
};


} //namespace cat

#endif
