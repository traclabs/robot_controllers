// Author: Stephen Hart

#ifndef ROBOT_CONTROLLERS_INTERFACE_PLANNER_H
#define ROBOT_CONTROLLERS_INTERFACE_PLANNER_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <robot_controllers_interface/handle.h>

/**
 * \mainpage
 * \section overview Overview
 * The robot_controllers_interface package provides ROS messages and header
 * files that define how robot planners are created and managed. There
 * are several key components:
 *   - A PlannerManager, which manages the loading, starting and stopping
 *     of planners.
 *   - One or more Controllers, which actually convert ROS messages into
 *     commands to the mechanism.
 *   - One or more Handles, which expose an API so that Controllers can pass
 *     commands to the mechanism.
 */

namespace robot_planners
{

// Forward define
class PlannerManager;

/**
 * @brief Base class for a planner. Is derived from a Handle, so that
 *        planners can be passed from PlannerManager::getHandle(),
 *        thus allowing planners to access other planners (to stack
 *        their commands.
 */
class Planner : public robot_controllers::Handle
{
public:
  /**
   * @brief Default constructor, does almost nothing, all setup is done in init().
   */
  Planner()
  {
  }

  /** @brief Ensure proper cleanup with virtual destructor. */
  virtual ~Planner()
  {
  }

  /**
   * @brief Initialize the planner and any required data structures.
   * @param nh Node handle for this planner.
   * @param manager The planner manager instance, this is needed for the
   *        planner to get information about joints, etc.
   * @returns 0 if succesfully configured, negative values are error codes.
   */
  virtual int init(ros::NodeHandle& nh, PlannerManager* manager)
  {
    name_ = nh.getNamespace();
    // remove leading slash
    if (name_.at(0) == '/')
      name_.erase(0, 1);

    return 0;
  }

  /**
   * @brief Attempt to start the planner. This should be called only by the
   *        PlannerManager instance.
   * @returns True if successfully started, false otherwise.
   */
  virtual bool start() = 0;

  /**
   * @brief Attempt to stop the planner. This should be called only by the
   *        PlannerManager instance.
   * @param force Should we force the planner to stop? Some planners
   *        may wish to continue running until they absolutely have to stop.
   * @returns True if successfully stopped, false otherwise.
   */
  virtual bool stop(bool force) = 0;

  /**
   * @brief Cleanly reset the planner to it's initial state. Some planners
   *        may choose to stop themselves. This is mainly used in the case of the
   *        the robot exiting some fault condition.
   * @returns True if successfully reset, false otherwise.
   */
  virtual bool reset() = 0;

  /**
   * @brief This is the update loop for the planner.
   * @param time The system time.
   * @param dt The timestep since last call to update.
   */
  virtual void update(const ros::Time& time, const ros::Duration& dt) = 0;

  /** @brief Get the name of this planner */
  std::string getName()
  {
    return name_;
  }

  /** @brief Get the type of this planner. */
  virtual std::string getType()
  {
    return "UnknownType";
  }

  /** @brief Get the names of joints/planners which this planner commands. */
  virtual std::vector<std::string> getCommandedNames() = 0;

  /** @brief Get the names of joints/planners which this planner exclusively claims. */
  virtual std::vector<std::string> getClaimedNames() = 0;


protected:

  /** @brief The name of the planner */
  std::string name_;

};

// Some typedefs
typedef boost::shared_ptr<Planner> PlannerPtr;



}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_INTERFACE_PLANNER_H
