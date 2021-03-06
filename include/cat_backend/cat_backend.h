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


// This must come before any files that include pluglinlib/class_loader.h
// We don't know why...
#include <moveit/warehouse/state_storage.h>

// Other stuff
#include <cat_backend/background_processing.h>
#include <cat_backend/BackendConfig.h>
#include <cat_backend/plan_interpolator.h>

#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <moveit/move_group/names.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/trajectory_tools.h>


#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/MoveGroupAction.h>

#include <moveit/robot_state/conversions.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/WrenchStamped.h>

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>


//namespace moveit_warehouse
//{
//class PlanningSceneStorage;
//class ConstraintsStorage;
//class RobotStateStorage;
//}

namespace cat
{

// ***************************************************************************************
class CatBackend
{
public:
  CatBackend(bool debug);
  ~CatBackend(void);

  const robot_model::RobotModelConstPtr& getRobotModel(void);
  planning_scene_monitor::LockedPlanningSceneRO getPlanningSceneRO(void) const;
  planning_scene_monitor::LockedPlanningSceneRW getPlanningSceneRW(void);
  const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor(void);

  // pass the execution of this function call to a separate thread that runs in the background
  void addTeleopJob(const boost::function<void(void)> &job);
  void addPerceptionJob(const boost::function<void(void)> &job);

  // queue the execution of this function for the next time the main update() loop gets called
  //void addMainLoopJob(const boost::function<void(void)> &job);


protected: // methods

  void clearAndRenewInteractiveMarkers(void);
  void changedPlanningGroup(void);
  bool isIKSolutionCollisionFree(robot_state::JointStateGroup *group, const std::vector<double> &ik_solution) const;

  //void executeMainLoopJobs(void);
  //void updateBackgroundJobProgressBar(void);
  void backgroundJobCompleted(void);

  void updatePlanningScene();

  bool isOutsideDeadband();
  void addMPGoals(planning_interface::MotionPlanRequest& req);
  void addCATGoals(planning_interface::MotionPlanRequest& req);
  void computeTeleopUpdate(const ros::Duration& target_period);
  void computeTeleopJTUpdate(const ros::Duration& target_period);
  void computeTeleopIKUpdate(const ros::Duration& target_period);
  bool computeTeleopPlanningUpdate(const ros::Duration& target_period, const std::string& type);

  //bool computeTeleopMPUpdate(const ros::Duration& target_period);
  //void computeTeleopCVXUpdate(const ros::Duration& target_period);
  bool generatePlan(const planning_pipeline::PlanningPipelinePtr &pipeline,
                    planning_interface::MotionPlanRequest &req,
                    planning_interface::MotionPlanResponse &res,
                    const ros::Time &future_time_limit);

  std::string getCurrentPlannerId();

  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz);

  void zeroFTSensor();

  std::string modeToStr(int mode);


//  const robot_state::RobotStatePtr& getQueryStartState(void) const
//  {
//    return query_start_state_->getState();
//  }

//  const robot_state::RobotStateConstPtr& getQueryGoalState(void) const
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

//  void setQueryStartState(const robot_state::RobotStatePtr &start);
//  void setQueryGoalState(const robot_state::RobotStatePtr &goal);

//  void updateQueryStartState(void);
//  void updateQueryGoalState(void);

  void onQueryGoalStateUpdate(robot_interaction::RobotInteraction::InteractionHandler* ih, bool needs_refresh);

  void setAndPublishLastGoalState(const robot_state::RobotStateConstPtr& state);

  void setAndPublishLastCurrentState(const sensor_msgs::JointStateConstPtr &joint_state);

  void onPlanningSceneMonitorUpdate(const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType& type);

  void updateInactiveGroupsFromCurrentRobot();

  std::string getCurrentPlanningGroup(void) const;

protected:

  void initWarehouse();
  void publishErrorMetrics(const robot_interaction::RobotInteraction::EndEffector& eef);
  void timerCallback();
  void onNewWrench(const geometry_msgs::WrenchStamped::ConstPtr& wrench);

  void goToRobotState(const std::string& pose_name);
  void fakeInteractiveMarkerFeedbackAtState(const robot_state::RobotState& state);

  bool openDataFileForWriting(const std::string& file_name);
  void closeDataFile();

protected: // members

  // robot interaction
  robot_interaction::RobotInteractionPtr robot_interaction_;
  robot_interaction::RobotInteraction::InteractionHandlerPtr query_goal_state_;
  robot_state::RobotStatePtr last_goal_state_;
  robot_state::RobotStatePtr last_current_state_;


  boost::shared_ptr<tf::TransformListener> tfl_;

  ros::NodeHandle root_node_handle_;
  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr planning_scene_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  planning_pipeline::PlanningPipelinePtr ompl_planning_pipeline_;
  planning_pipeline::PlanningPipelinePtr cat_planning_pipeline_;
  bool allow_trajectory_execution_;
  bool show_ik_solution_;
  bool use_warehouse_;
  bool warehouse_connected_;

  ros::Publisher publish_goal_state_;
  ros::Publisher publish_current_state_;
  ros::Publisher publish_cartesian_goal_left_;
  ros::Publisher publish_cartesian_goal_right_;

  ros::Publisher publish_error_;
  ros::Publisher publish_zero_ft_;

  // RSS stuff that should maybe come out?
  ros::Subscriber subscribe_ft_wrench_;
  geometry_msgs::WrenchStamped last_wrench_;

  bool record_data_;
  ros::Time data_start_time_;
  std::ofstream data_file_stream_;

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
  //robot_trajectory::RobotTrajectory last_trajectory_;
  //ros::Time last_trajectory_time_;

  trajectory_processing::IterativeParabolicTimeParameterization smoother_;

  boost::shared_ptr<moveit_warehouse::RobotStateStorage> robot_state_storage_;

  //moveit_msgs::WorkspaceParameters workspace_parameters_;

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
