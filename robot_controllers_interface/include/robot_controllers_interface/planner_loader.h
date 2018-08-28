// Author: Stephen Hart

#ifndef ROBOT_CONTROLLERS_INTERFACE_PLANNER_LOADER_H
#define ROBOT_CONTROLLERS_INTERFACE_PLANNER_LOADER_H

#include <string>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <robot_controllers_interface/planner.h>

namespace robot_controllers
{

// Forward def
class PlannerManager;

/** @brief Class for loading and managing a single planner. */
class PlannerLoader
{
public:
  /** @brief Initialize this loader */
  PlannerLoader();

  /** @brief Load the planner. */
  bool init(const std::string& name, PlannerManager* manager);

  /** @brief This calls through to planner, saves state locally. */
  bool start();

  /** @brief This calls through to planner, saves state locally. */
  bool stop(bool force);

  /** @brief This calls through to planner. */
  bool reset();

  /** @brief If planner is active, calls through to planner. */
  void update(const ros::Time& time, const ros::Duration& dt);

  /** @brief Returns true if the planner is active. */
  bool isActive();

  /** @brief Returns the planner held by this loader. */
  PlannerPtr getPlanner();

private:
  pluginlib::ClassLoader<robot_controllers::Planner> plugin_loader_;
  PlannerPtr planner_;
  bool active_;
};

typedef boost::shared_ptr<PlannerLoader> PlannerLoaderPtr;

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_INTERFACE_PLANNER_LOADER_H
