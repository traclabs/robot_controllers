// Author: Stephen Hart

#ifndef ROBOT_CONTROLLERS_INTERFACE_PLANNER_MANAGER_H
#define ROBOT_CONTROLLERS_INTERFACE_PLANNER_MANAGER_H

#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <robot_controllers_msgs/QueryPlannerStatesAction.h>

#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/planner.h>
#include <robot_controllers_interface/planner_loader.h>

#include <craftsman_utils/plan_cache_container.h>

namespace robot_controllers
{

/** @brief Base class for a planner manager. */
class PlannerManager
{
  typedef actionlib::SimpleActionServer<robot_controllers_msgs::QueryPlannerStatesAction> server_t;

  typedef std::vector<PlannerLoaderPtr> PlannerList;
  typedef std::vector<JointHandlePtr> JointHandleList;

public:
  PlannerManager();

  /** @brief Ensure proper shutdown with virtual destructor. */
  virtual ~PlannerManager()
  {
  }

  /**
   * @brief Startup the planner manager, loading default planners.
   * @param nh The proper node handle for finding parameters.
   * @returns 0 if success, negative values are error codes.
   *
   * Note: JointHandles should be added before this is called.
   */
  virtual int init(ros::NodeHandle& nh);

  /** @brief Start a planner. */
  virtual int requestStart(const std::string& name);

  /** @brief Stop a planner. */
  virtual int requestStop(const std::string& name);

  /** @brief Update active planners. */
  virtual void update(const ros::Time& time, const ros::Duration& dt);

  /** @brief Reset all planners. */
  virtual void reset();

  /** @brief Add a joint handle. */
  bool addJointHandle(JointHandlePtr& j);

  /**
   * @brief Get the handle associated with a particular joint/planner name.
   * @param name The name of the joint/planner.
   */
  HandlePtr getHandle(const std::string& name);

  /**
   * @brief Get the joint handle associated with a particular joint name.
   * @param name The name of the joint.
   *
   * This is mainly a convienence function.
   */
  JointHandlePtr getJointHandle(const std::string& name);

  /**
   * @brief Get the handle associated with a particular planner name.
   * @param name The name of the planner.
   *
   * This is mainly a convienence function.
   */
  PlannerPtr getPlanner(const std::string& name);

  /**
   * @brief Get the names of planners that have been loaded into the manager.
   * @returns list of the names of loaded planners.
   *
   */
  std::vector<std::string> getPlannerNames();

  /** @brief The container of planner caches for each robot. */
  craftsman_utils::PlanCacheContainer* cache_container_;

private:
  /** @brief Action callback. */
  void execute(const robot_controllers_msgs::QueryPlannerStatesGoalConstPtr& goal);

  /** @brief Fill in the current state of planners. */
  void getState(robot_controllers_msgs::QueryPlannerStatesResult& result);

  /** @brief Load a planner. */
  bool load(const std::string& name);

  PlannerList planners_;
  JointHandleList joints_;

  boost::shared_ptr<server_t> server_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_INTERFACE_PLANNER_MANAGER_H
