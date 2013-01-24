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
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/kinematic_constraints/utils.h>
#include <std_msgs/Float64.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


// TODO get these out of here!!!
namespace{

// Stolen wholesale from object_manipulator::MechanismInterface
void positionAndAngleDist(Eigen::Affine3d start, Eigen::Affine3d end,
                                              double &pos_dist,
double &angle, Eigen::Vector3d &axis, Eigen::Vector3d &direction)
{
  //trans = end to start = global to start * end to global
  Eigen::Affine3d trans;
  trans = start.inverse() * end;
  Eigen::AngleAxis<double> angle_axis;
  angle_axis = trans.rotation();
  angle = angle_axis.angle();
  axis = angle_axis.axis();
  if(angle > M_PI)
  {
    angle = -(angle - 2*M_PI);
    axis = -axis;
  }
  direction = trans.translation();
  pos_dist = sqrt(direction.dot(direction));
  if(pos_dist) direction *= 1/pos_dist;
}

// Stolen wholesale from object_manipulator::MechanismInterface
geometry_msgs::PoseStamped clipDesiredPose(const geometry_msgs::PoseStamped &current_pose,
                                           const geometry_msgs::PoseStamped &desired_pose,
                                           double clip_dist, double clip_angle,
                                           double &resulting_clip_fraction)
{
  //no clipping desired
  if(clip_dist == 0 && clip_angle == 0) return desired_pose;

  //Get the position and angle dists between current and desired
  Eigen::Affine3d current_trans, desired_trans;
  double pos_dist, angle;
  Eigen::Vector3d axis, direction;
  tf::poseMsgToEigen(current_pose.pose, current_trans);
  tf::poseMsgToEigen(desired_pose.pose, desired_trans);
  positionAndAngleDist(current_trans, desired_trans, pos_dist, angle, axis, direction);

  //Clip the desired pose to be at most clip_dist and the desired angle to be at most clip_angle (proportional)
  //from the current
  double pos_mult, angle_mult;
  double pos_change, angle_change;
  angle_mult = fabs(angle / clip_angle);
  pos_mult = fabs(pos_dist / clip_dist);
  if(pos_mult <=1 && angle_mult <=1){
    return desired_pose;
  }
  double mult = pos_mult;
  if(angle_mult > pos_mult) mult = angle_mult;
  pos_change = pos_dist / mult;
  angle_change = angle / mult;
  resulting_clip_fraction = 1 / mult;

  Eigen::Affine3d clipped_trans;
  clipped_trans = current_trans;
  Eigen::Vector3d scaled_direction;
  scaled_direction = direction * pos_change;
  Eigen::Translation3d translation(scaled_direction);
  clipped_trans = clipped_trans * translation;
  Eigen::AngleAxis<double> angle_axis(angle_change, axis);
  clipped_trans = clipped_trans * angle_axis;
  geometry_msgs::PoseStamped clipped_pose;
  tf::poseEigenToMsg(clipped_trans, clipped_pose.pose);
  clipped_pose.header = desired_pose.header;
  return clipped_pose;
}

}


namespace cat {

static const std::string CARTESIAN_RIGHT_ARM = "r_cart";
static const std::string CARTESIAN_LEFT_ARM = "l_cart";
static const std::string CARTESIAN_COMMAND_SUFFIX = "/command_pose";
static const std::string CARTESIAN_POSTURE_SUFFIX = "/command_posture";


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
    ROS_DEBUG("Dynamic reconfigure waiting for lock...");
    boost::mutex::scoped_lock slock(owner_->teleop_lock_);
    ROS_DEBUG("Dynamic reconfigure got lock!");

    cat_backend::BackendConfig old_config = owner_->config_ ;
    owner_->config_ = config;

    if(level == cat_backend::Backend_INITIALIZE)
    {
      // Always want to set disabled on re-launch
      ROS_INFO("Dynamic reconfigure is setting initial settings");
      owner_->config_.teleop_mode = config.teleop_mode = cat_backend::Backend_TELEOP_DISABLE;
    }

    if(level == cat_backend::Backend_PLANNING_GROUP)
    {
      owner_->psi_.clearPlan();
      owner_->changedPlanningGroup();
    }

    if(owner_->query_goal_state_)
    {
      owner_->query_goal_state_->setIKAttempts(config.ik_attempts);
      owner_->query_goal_state_->setIKTimeout(config.ik_timeout);
      owner_->query_goal_state_->setInteractionMode( (config.ik_type == cat_backend::Backend_POSITION_IK) ?
                                               robot_interaction::RobotInteraction::InteractionHandler::POSITION_IK
                                             : robot_interaction::RobotInteraction::InteractionHandler::VELOCITY_IK);
    }

    if(level == cat_backend::Backend_RESET_STATE)
    {
      owner_->psi_.clearPlan();
      owner_->changedPlanningGroup();
    }

    if(level == cat_backend::Backend_TELEOP_MODE)
    {
      owner_->psi_.clearPlan();
      std::vector<std::string> cartesian_controllers(2), joint_controllers(2);
      cartesian_controllers[0] = "r_cart";
      cartesian_controllers[1] = "l_cart";
      joint_controllers[0] = "r_arm_controller";
      joint_controllers[1] = "l_arm_controller";

      ROS_INFO("Teleop mode changed to %s", owner_->modeToStr(owner_->config_.teleop_mode).c_str());
      if(old_config.teleop_mode != cat_backend::Backend_TELEOP_JT && config.teleop_mode == cat_backend::Backend_TELEOP_JT
         && owner_->trajectory_execution_manager_)
      {
        // Switch to JT controller
        owner_->trajectory_execution_manager_->getControllerManager()->switchControllers(cartesian_controllers, joint_controllers);
      }
      if(old_config.teleop_mode == cat_backend::Backend_TELEOP_JT && config.teleop_mode != cat_backend::Backend_TELEOP_JT
         && owner_->trajectory_execution_manager_)
      {
        // Switch to Joint Controller
        owner_->trajectory_execution_manager_->getControllerManager()->switchControllers(joint_controllers, cartesian_controllers);
      }

      switch(config.teleop_mode)
      {
      case cat_backend::Backend_TELEOP_JT:
        owner_->config_.target_period = config.target_period = 0.015; // TODO magic number! (parameter?)
        break;
      case cat_backend::Backend_TELEOP_IK:
        owner_->config_.target_period = config.target_period = 0.033; // TODO magic number! (parameter?)
        break;
      case cat_backend::Backend_TELEOP_MP:
        owner_->config_.target_period = config.target_period = 0.3; // TODO magic number! (parameter?)
        break;
      case cat_backend::Backend_TELEOP_CVX:
        owner_->config_.target_period = config.target_period = 0.033; // TODO magic number! (parameter?)
        break;
      default:
        // do nothing
        break;
      }

      if(old_config.teleop_mode == cat_backend::Backend_TELEOP_DISABLE &&
         config.teleop_mode != cat_backend::Backend_TELEOP_DISABLE)
        owner_->addBackgroundJob(boost::bind(&CatBackend::computeTeleopUpdate, owner_, ros::Duration(config.target_period)));
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
    allow_trajectory_execution_(true),
    cycle_id_(0)
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
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor_->UPDATE_SCENE);
  }
  planning_scene_ = planning_scene::PlanningScene::clone(getPlanningSceneRO());

  // ===== Planning =====
  ROS_INFO("CAT backend: Initializing planning pipelines");
  const kinematic_model::KinematicModelConstPtr model = planning_scene_monitor_->getKinematicModel();
  if(!model)
    ROS_ERROR("The model is broken!");
  ompl_planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getKinematicModel(),
                                                                        "ompl/planning_plugin", "ompl/request_adapters"));
  cat_planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getKinematicModel(),
                                                                       "cat/planning_plugin", "cat/request_adapters"));
  cat_planning_pipeline_->checkSolutionPaths(false);

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
  publish_cartesian_goal_right_ = root_node_handle_.advertise<geometry_msgs::PoseStamped>(CARTESIAN_RIGHT_ARM + CARTESIAN_COMMAND_SUFFIX, 1);
  publish_cartesian_goal_left_ = root_node_handle_.advertise<geometry_msgs::PoseStamped>(CARTESIAN_LEFT_ARM + CARTESIAN_COMMAND_SUFFIX, 1);

  // ===== Visualization =====
  ROS_INFO("CAT backend: Initializing robot interaction tools");
  publish_goal_state_ = node_handle_.advertise<moveit_msgs::PlanningScene>("goal_state_robot", 1);
  publish_current_state_ = node_handle_.advertise<moveit_msgs::PlanningScene>("current_state_robot", 1);
  robot_interaction_.reset(new robot_interaction::RobotInteraction(getKinematicModel(), "cat_backend"));
  kinematic_state::KinematicStatePtr ks(new kinematic_state::KinematicState(getPlanningSceneRO()->getCurrentState()));
  last_goal_state_.reset(new kinematic_state::KinematicState(*ks));
  last_current_state_.reset(new kinematic_state::KinematicState(*ks));
  query_goal_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("goal", *ks, planning_scene_monitor_->getTFClient()));
  query_goal_state_->setUpdateCallback(boost::bind(&CatBackend::onQueryGoalStateUpdate, this, _1, _2));
  query_goal_state_->setStateValidityCallback(boost::bind(&CatBackend::isIKSolutionCollisionFree, this, _1, _2));
  query_goal_state_->setMeshesVisible(true);
  query_goal_state_->setIKAttempts(config_.ik_attempts);
  query_goal_state_->setIKTimeout(config_.ik_timeout);
  query_goal_state_->setInteractionMode( (config_.ik_type == cat_backend::Backend_POSITION_IK) ?
                                           robot_interaction::RobotInteraction::InteractionHandler::POSITION_IK
                                         : robot_interaction::RobotInteraction::InteractionHandler::VELOCITY_IK);

  // ===== Visualization =====
  publish_error_ = node_handle_.advertise<std_msgs::Float64>("metrics/error_magnitude", 10);

  // Now we actually go...
  ROS_INFO("CAT backend configuring intial planning group");
  changedPlanningGroup();
  ROS_INFO("CAT backend initialized!");
}

cat::CatBackend::~CatBackend(void)
{
  if(reconfigure_impl_)
    delete reconfigure_impl_;
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

void cat::CatBackend::computeTeleopUpdate(const ros::Duration& target_period)
{
  ROS_DEBUG("Teleop update waiting for lock...");
  //boost::mutex::scoped_lock slock(teleop_lock_);
  teleop_lock_.lock();
  ROS_DEBUG("Teleop update got lock! v v v v v v v v v v v v v v v v v v v v v");

  planning_scene_ = planning_scene::PlanningScene::clone(getPlanningSceneRO());

  ROS_DEBUG("Copied planning scene.");
  ros::Time update_start_time = ros::Time::now();
  cycle_id_ += 1;

  try
  {
    switch(config_.teleop_mode)
    {
      case(cat_backend::Backend_TELEOP_JT):
        computeTeleopJTUpdate(target_period);
        break;
      case(cat_backend::Backend_TELEOP_IK):
        computeTeleopIKUpdate(target_period);
        break;
      case(cat_backend::Backend_TELEOP_MP):
        computeTeleopMPUpdate(target_period);
        break;
      case(cat_backend::Backend_TELEOP_CVX):
        computeTeleopCVXUpdate(target_period);
        break;
      case(cat_backend::Backend_TELEOP_DISABLE):
        ROS_WARN("It seems teleop was DISABLED sometime between the last queueing action and now!");
        return;
        break;
      default:
        ROS_ERROR("An unhandled teleop state was requested.");
    }
  }
  catch(...)
  {
    ROS_ERROR("Caught some kind of exception...");
  }
  teleop_lock_.unlock();

  ros::Duration time_used = ros::Time::now() - update_start_time;
  ros::Duration remaining_time = target_period - time_used - ros::Duration(0.001); // TODO magic number!
  ros::Duration next_cycle_allowed_time(config_.target_period); // default
  if(remaining_time.toSec() < 0.0)
  {
    remaining_time = ros::Duration(0.0);
    next_cycle_allowed_time.fromSec( target_period.toSec() * config_.growth_factor );
    ROS_ERROR("Cycle [%d] Time used: %.3f sec exceeded target of %.3f sec. Next cycle target is %.3f",
              cycle_id_, time_used.toSec(), target_period.toSec(), next_cycle_allowed_time.toSec());
  }
  else
  {
    double time = std::max<double>( config_.target_period,
                                    target_period.toSec() - config_.shrink_factor*remaining_time.toSec());
    next_cycle_allowed_time.fromSec( time );
    if (config_.sleep_remainder)
    {
      ROS_INFO("Cycle [%d] Time used: %.3f sec, sleeping for %.3f sec, next cycle target is %.3f sec",
               cycle_id_, time_used.toSec(), remaining_time.toSec(), next_cycle_allowed_time.toSec());
      remaining_time.sleep();
    }
    else
      ROS_INFO("Cycle [%d] Time used: %.3f sec of %.3f sec allowed (%.1f %%), next cycle target is %.3f",
              cycle_id_,
              time_used.toSec(), target_period.toSec(),
              time_used.toSec()/ target_period.toSec()*100.0,
              next_cycle_allowed_time.toSec());
  }

  // When all is done, it gets ready to call itself again!
  if(config_.teleop_mode != cat_backend::Backend_TELEOP_DISABLE)
    addBackgroundJob(boost::bind(&CatBackend::computeTeleopUpdate, this, next_cycle_allowed_time));
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
  kinematic_state::KinematicStatePtr future_start_state( new kinematic_state::KinematicState(planning_scene_->getCurrentState()));

  moveit_msgs::MotionPlanRequest req;
  geometry_msgs::PoseStamped goal_pose;
  getQueryGoalStateHandler()->getLastEndEffectorMarkerPose(aee[0], goal_pose);
  req.group_name = group_name;
  psi_.getStateAtTime( future_time, future_start_state, req.start_state);
  req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(aee[0].parent_link, goal_pose));
//    req.motion_plan_request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(future_start_state->getJointStateGroup(group_name),
//                                                                                                       .001, .001));
  req.num_planning_attempts = 1;
  req.allowed_planning_time = target_period*0.65;  // TODO mgic number!

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
// ================================ J-Transpose ===============================
// ============================================================================
void cat::CatBackend::computeTeleopJTUpdate(const ros::Duration &target_period)
{
  ROS_DEBUG("TeleopJTUpdate!");

  const std::vector<robot_interaction::RobotInteraction::EndEffector>& aee = robot_interaction_->getActiveEndEffectors();
  for(int i = 0; i < aee.size(); i++)
  {
    // TODO clip the commanded pose!
    geometry_msgs::PoseStamped goal_pose;
    getQueryGoalStateHandler()->getLastEndEffectorMarkerPose(aee[i], goal_pose);
    geometry_msgs::PoseStamped current_pose;
    tf::poseEigenToMsg(planning_scene_->getCurrentState().getLinkState(aee[i].parent_link)->getGlobalLinkTransform(), current_pose.pose);
    double linear_clip = target_period.toSec()*config_.jt_linear_clip_ratio;
    double angle_clip = target_period.toSec()*config_.jt_angle_clip_ratio;
    double clip_fraction = 0;
    goal_pose = clipDesiredPose(current_pose, goal_pose, linear_clip, angle_clip, clip_fraction );
    goal_pose.header.stamp = ros::Time(0);
    if(aee[i].parent_link == "r_wrist_roll_link")
      publish_cartesian_goal_right_.publish(goal_pose);
    else if(aee[i].parent_link == "l_wrist_roll_link")
      publish_cartesian_goal_left_.publish(goal_pose);
    else
      ROS_WARN("Don't have a publisher for end-effector [%s] with parent link [%s]",
               aee[i].eef_group.c_str(), aee[i].parent_link.c_str());
  }
  ROS_DEBUG("Done with TeleopJTUpdate");
}

// ============================================================================
// ============================ Inverse Kinematics ============================
// ============================================================================
void cat::CatBackend::computeTeleopIKUpdate(const ros::Duration &target_period)
{
  ros::Time future_time = ros::Time::now() + target_period;
  ROS_DEBUG("TeleopIKUpdate!");
  std::string group_name = getCurrentPlanningGroup();

  if (group_name.empty())
    return;

//  kinematic_state::KinematicStatePtr future_start_state( new kinematic_state::KinematicState(planning_scene_->getCurrentState()));
//  moveit_msgs::RobotState start_state;
//  psi_.getStateAtTime( future_time, future_start_state, start_state);
//  std::vector<moveit_msgs::JointLimits> limits = future_start_state->getKinematicModel()->getJointModelGroup(group_name)->getVariableLimits();
//  for(size_t i = 0; i < limits.size(); ++i)
//  {
//    moveit_msgs::JointLimits& l = limits[i];
//    if(l.has_velocity_limits)
//      l.max_velocity *= 2.0; // TODO magic number!
//  }

//  trajectory_msgs::JointTrajectory joint_trajectory;

//  last_goal_state_lock_.lock();
//  std::vector< const std::vector<kinematic_state::JointState*> * > states(2);
//  states[0] = &(future_start_state->getJointStateGroup(group_name)->getJointStateVector());
//  states[1] = &(last_goal_state_->getJointStateGroup(group_name)->getJointStateVector());
//  if( states[0]->size() != states[1]->size() )
//  {
//    ROS_ERROR("Joint vectors not the same size! Start has %zd while goal has %zd joints.", states[0]->size(), states[1]->size());
//    return;
//  }
//  joint_trajectory.joint_names.resize(states[0]->size());
//  joint_trajectory.points.resize(2);
//  joint_trajectory.header.frame_id = last_goal_state_->getKinematicModel()->getModelFrame();
//  joint_trajectory.header.stamp = start_state.joint_state.header.stamp;

//  for(size_t i = 0; i < states.size(); ++i)
//  {
//    joint_trajectory.points[i].positions.resize(states[i]->size());
//    joint_trajectory.points[i].velocities.resize(states[i]->size());

//    // Now actually populate the joints...
//    for(int j = 0; j < states[i]->size(); j++ )
//    {
//      kinematic_state::JointState* js = (*(states[i]))[j];
//      joint_trajectory.joint_names[j] = js->getName();
//      joint_trajectory.points[i].positions[j] = js->getVariableValues()[0];
//      joint_trajectory.points[i].velocities[j] = 0.0;  // TODO could we estimate velocity?
//    }
//  }
//  last_goal_state_lock_.unlock();
//  smoother_.computeTimeStamps(joint_trajectory, limits, start_state);

//  if(future_time < ros::Time::now() + ros::Duration(0.001)) // TODO this offset is a (vetted) magic number...
//  {
//    ROS_WARN("Planning took too long, discarding result.");
//  }
//  else
//  {
//    ROS_DEBUG("Planning SUCCESS, saving plan.");
//    move_group_interface::MoveGroup::Plan plan;
//    plan.trajectory_.joint_trajectory = joint_trajectory;
//    plan.start_state_ = start_state;
//    plan.trajectory_.joint_trajectory.header.stamp = future_time;
//    psi_.setPlan(plan);
//  }

//  // Send out last plan for execution.
//  // It is stamped with a time in the future, so it should be ok to send it now.
//  if(psi_.hasNewPlan())
//  {
//    trajectory_execution_manager_->pushAndExecute(psi_.getPlan().trajectory_); // TODO should specify the controller huh?
//    psi_.setPlanAsOld();
//  }
//  else
//  {
//    ROS_DEBUG("No plan saved, not executing anything.");
//  }


  last_goal_state_lock_.lock();

  //double distance = planning_scene_->getCurrentState().distance(*last_goal_state_);

  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory.points.resize(1);
  traj.joint_trajectory.points[0].time_from_start = ros::Duration(config_.ik_traj_time);

  const std::vector<kinematic_state::JointState*>& jsv = last_goal_state_->getJointStateGroup(group_name)->getJointStateVector();
  traj.joint_trajectory.joint_names.resize(jsv.size());
  traj.joint_trajectory.points[0].positions.resize(jsv.size());
  traj.joint_trajectory.points[0].velocities.resize(jsv.size());
  traj.joint_trajectory.header.frame_id = last_goal_state_->getKinematicModel()->getModelFrame();
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

  kinematic_state::KinematicStatePtr future_start_state( new kinematic_state::KinematicState(planning_scene_->getCurrentState()));

  ROS_DEBUG("Formulating planning request");
  moveit_msgs::MotionPlanRequest req;
  req.allowed_planning_time = target_period*0.75;
  req.num_planning_attempts = 1;
  psi_.getStateAtTime( future_time, future_start_state, req.start_state);
  req.group_name = group_name;

  ROS_DEBUG("Constructing goal constraint...");
  req.goal_constraints.resize(1);
  last_goal_state_lock_.lock();
  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(last_goal_state_->getJointStateGroup(group_name), config_.goal_tolerance);
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
    //planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);
    ROS_DEBUG_NAMED("cat_backend", "Issuing planning request.");
    solved = pipeline->generatePlan(planning_scene_, req, res);
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

void cat::CatBackend::onQueryGoalStateUpdate(robot_interaction::RobotInteraction::InteractionHandler* ih, bool needs_refresh)
{
  ROS_DEBUG("Processing a query goal update.");
  const std::vector<robot_interaction::RobotInteraction::EndEffector>& aee = robot_interaction_->getActiveEndEffectors();
  bool in_error = false;
  for(int i=0; i < aee.size(); i++)
  {
    in_error = ih->inError(aee[i]) || in_error;
  }

  if(!in_error)
  {
    setAndPublishLastGoalState(query_goal_state_->getState());
  }
  else
  {
    // Do nothing?
  }
  setAndPublishLastCurrentState();
  updateInactiveGroupsFromCurrentRobot();

  for(int i=0; i < aee.size(); i++)
  {
    const robot_interaction::RobotInteraction::EndEffector& eef = aee[0]; // only handle the first one...
    tf::Pose link_pose, goal_pose;
    geometry_msgs::PoseStamped goal_pose_msg;
    query_goal_state_->getLastEndEffectorMarkerPose(eef, goal_pose_msg);
    tf::poseMsgToTF(goal_pose_msg.pose, goal_pose);
    tf::poseEigenToTF(last_current_state_->getLinkState(eef.parent_link)->getGlobalLinkTransform(), link_pose);
    tf::Vector3 trans_error = link_pose.getOrigin() - goal_pose.getOrigin();
    double distance = trans_error.length();
    double angle = link_pose.getRotation().angleShortestPath(goal_pose.getRotation());
    std_msgs::Float64 msg;
    msg.data = distance; // just use distance for now
    publish_error_.publish(msg);
  }

  // Update the markers
  ROS_DEBUG("Refreshing markers.");
  if(needs_refresh)
  {
    robot_interaction_->addInteractiveMarkers(getQueryGoalStateHandler());
    robot_interaction_->publishInteractiveMarkers();
  }
}

void cat::CatBackend::setAndPublishLastGoalState(const kinematic_state::KinematicStateConstPtr& state)
{
  boost::mutex::scoped_lock slock(last_goal_state_lock_);
  ROS_DEBUG("Saving new last_goal_state");
  moveit_msgs::PlanningScene gs;
  *last_goal_state_ = *state;
  kinematic_state::kinematicStateToRobotState(*last_goal_state_, gs.robot_state);
  publish_goal_state_.publish(gs);
}

void cat::CatBackend::setAndPublishLastCurrentState()
{
  boost::mutex::scoped_lock slock(last_current_state_lock_);
  ROS_DEBUG("Saving new last_current_state");
  moveit_msgs::PlanningScene gs;
  *last_current_state_ = getPlanningSceneRO()->getCurrentState();
  kinematic_state::kinematicStateToRobotState(*last_current_state_, gs.robot_state);
  publish_current_state_.publish(gs);
}

void cat::CatBackend::publishErrorMetrics()
{
  ROS_ERROR("Publish error metrics doesn't do anything... do we need it?");
}

void cat::CatBackend::updateInactiveGroupsFromCurrentRobot()
{
  ROS_DEBUG("Updating inactive groups!");
  boost::mutex::scoped_lock slock(last_current_state_lock_);
  kinematic_state::KinematicState new_state = *last_current_state_;
    const kinematic_state::JointStateGroup* jsg = query_goal_state_->getState()->getJointStateGroup(getCurrentPlanningGroup());
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
    query_goal_state_->clearPoseOffsets();
    query_goal_state_->clearSavedMarkerPoses();
    robot_interaction_->decideActiveComponents(group);

    // Set offsets
    geometry_msgs::Pose offset;
    offset.position.x = config_.offset_x;
    offset.orientation.w = 1.0;
    const std::vector<robot_interaction::RobotInteraction::EndEffector>& aeef = robot_interaction_->getActiveEndEffectors();
    for(int i = 0; i < aeef.size(); i++)
    {
      if(aeef[i].eef_group.find("gripper") != std::string::npos)
      {
        ROS_DEBUG("Setting offset pose for end-effector [%s]", aeef[i].eef_group.c_str());
        query_goal_state_->setPoseOffset(aeef[i], offset);
      }
    }

    // Set query to current state to help with IK seed.
    query_goal_state_->setState(getPlanningSceneRO()->getCurrentState());
    setAndPublishLastGoalState(query_goal_state_->getState());

    query_goal_state_->setControlsVisible(config_.show_controls);
    // Renew markers
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
