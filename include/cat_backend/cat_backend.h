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

#include <moveit/robot_interaction/robot_interaction.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group/names.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit/move_group_interface/move_group.h>

//#include <moveit_msgs/ExecuteKnownTrajectory.h>
//#include <moveit_msgs/QueryPlannerInterfaces.h>

#include <tf/transform_listener.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/pick_place/pick_place.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>



namespace cat
{
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
  void addBackgroundJob(const boost::function<void(void)> &job);

  // queue the execution of this function for the next time the main update() loop gets called
  void addMainLoopJob(const boost::function<void(void)> &job);


protected: // methods

  void publishInteractiveMarkers(void);
  void changedPlanningGroup(void);
  bool isIKSolutionCollisionFree(kinematic_state::JointStateGroup *group, const std::vector<double> &ik_solution) const;

  void executeMainLoopJobs(void);
  void updateBackgroundJobProgressBar(void);
  void backgroundJobCompleted(void);

  void computeTeleopUpdate();
  void computeTeleopMPUpdate(const ros::Duration& target_period);

  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz);

  std::string modeToStr(int mode);


//  const kinematic_state::KinematicStatePtr& getQueryStartState(void) const
//  {
//    return query_start_state_->getState();
//  }

  const kinematic_state::KinematicStatePtr& getQueryGoalState(void) const
  {
    return query_goal_state_->getState();
  }

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

  void setQueryStartState(const kinematic_state::KinematicStatePtr &start);
  void setQueryGoalState(const kinematic_state::KinematicStatePtr &goal);

  void updateQueryStartState(void);
  void updateQueryGoalState(void);

  void onQueryGoalStateUpdate(void);

  std::string getCurrentPlanningGroup(void) const;


protected: // members

  // These could be useful...
  //BackgroundProcessing background_process_;
  //std::deque<boost::function<void(void)> > main_loop_jobs_;
  //boost::mutex main_loop_jobs_lock_;


  // For drawing the robot...
  //KinematicStateVisualizationPtr query_robot_start_;                  ///< Handles drawing the robot at the start configuration
  //KinematicStateVisualizationPtr query_robot_goal_;                   ///< Handles drawing the robot at the goal configuration
  //KinematicStateVisualizationPtr display_path_robot_;                 ///< Handles actually drawing the robot along motion plans

  // Maybe useful.
  //bool animating_path_;
  //int current_state_;
  //float current_state_time_;

  // robot interaction
  robot_interaction::RobotInteractionPtr robot_interaction_;
//  robot_interaction::RobotInteraction::InteractionHandlerPtr query_start_state_;
  robot_interaction::RobotInteraction::InteractionHandlerPtr query_goal_state_;
  // I don't know what these are for yet.
  //std::map<std::string, int> collision_links_start_;
  //std::map<std::string, int> collision_links_goal_;

  boost::shared_ptr<tf::TransformListener> tfl_;

  ros::NodeHandle root_node_handle_;
  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  plan_execution::PlanExecutionPtr plan_execution_;
  pick_place::PickPlacePtr pick_place_;
  bool allow_trajectory_execution_;

//  boost::scoped_ptr<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction> > move_action_server_;
//  moveit_msgs::MoveGroupFeedback move_feedback_;

//  boost::scoped_ptr<actionlib::SimpleActionServer<moveit_msgs::PickupAction> > pickup_action_server_;
//  moveit_msgs::PickupFeedback pickup_feedback_;

//  ros::ServiceServer plan_service_;
//  ros::ServiceServer execute_service_;
//  ros::ServiceServer query_service_;

  //ros::Publisher publish_start_state_;
  ros::Publisher publish_goal_state_;
  ros::Publisher publish_current_state_;

  boost::shared_ptr<move_group_interface::MoveGroup::Plan> current_plan_;


  BackgroundProcessing background_process_;
  std::deque<boost::function<void(void)> > main_loop_jobs_;
  boost::mutex main_loop_jobs_lock_;


  moveit_msgs::WorkspaceParameters workspace_parameters_;


//  MoveGroupState move_state_;
//  MoveGroupState pickup_state_;

  class DynamicReconfigureImpl;
  DynamicReconfigureImpl *reconfigure_impl_;

  cat_backend::BackendConfig config_;

};


} //namespace cat

#endif
