/*
 * Copyright (c) 2014-2015, Fetch Robotics Inc.
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
 *     * Neither the name of the Fetch Robotics Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Michael Ferguson

#include <sstream>
#include <robot_controllers_interface/planner_manager.h>

namespace robot_controllers
{

PlannerManager::PlannerManager()
{
}

int PlannerManager::init(ros::NodeHandle& nh)
{
  // Find and load default planners
  XmlRpc::XmlRpcValue planner_params;
  if (nh.getParam("default_planners", planner_params))
  {
    if (planner_params.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR_NAMED("PlannerManager", "Parameter 'default_planners' should be a list.");
      return -1;
    }
    else
    {
      // Load each planner
      for (int c = 0; c < planner_params.size(); c++)
      {
        // Make sure name is valid
        XmlRpc::XmlRpcValue &planner_name = planner_params[c];
        if (planner_name.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_WARN_NAMED("PlannerManager", "Planner name is not a string?");
          continue;
        }

        // Create planner (in a loader)
        load(static_cast<std::string>(planner_name));
      }
    }
  }
  else
  {
    ROS_WARN_NAMED("PlannerManager", "No planners loaded.");
    return -1;
  }

  // Setup actionlib server
  server_.reset(new server_t(nh, "/query_planner_states",
                             boost::bind(&PlannerManager::execute, this, _1),
                             false));
  server_->start();

  return 0;
}

int PlannerManager::requestStart(const std::string& name)
{
  // Find requested planner
  PlannerLoaderPtr planner;
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    if ((*p)->getPlanner()->getName() == name)
    {
      planner = *p;
      break;
    }
  }

  // Does planner exist?
  if (!planner)
  {
    ROS_ERROR_STREAM_NAMED("PlannerManager", "Unable to start " << name.c_str() << ": no such planner.");
    return -1;
  }

  // Is planner already running?
  if (planner->isActive())
  {
    ROS_DEBUG_STREAM_NAMED("PlannerManager", "Unable to start " << name.c_str() << ": already running.");
    return 0;
  }

  // Check for conflicts
  std::vector<std::string> names = planner->getPlanner()->getCommandedNames();
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    // Only care about active planners
    if (!(*p)->isActive())
      continue;

    std::vector<std::string> names2 = (*p)->getPlanner()->getClaimedNames();
    bool conflict = false;
    for (size_t i = 0; i < names.size(); i++)
    {
      for (size_t i2 = 0; i2 < names2.size(); i2++)
      {
        if (names[i] == names2[i2])
        {
          conflict = true;
          break;
        }
      }
      if (conflict) break;
    }
    // Have conflict, try to stop planner (without force)
    if (conflict)
    {
      if ((*p)->stop(false))
      {
        ROS_INFO_STREAM_NAMED("PlannerManager", "Stopped " << (*p)->getPlanner()->getName().c_str());
      }
      else
      {
        // Unable to stop c, cannot start planner
        ROS_ERROR_STREAM_NAMED("PlannerManager", "Unable to stop " <<
                                                    (*p)->getPlanner()->getName().c_str() <<
                                                    " when trying to start " << name.c_str());
        return -1;
      }
    }
  }

  // Start planner
  if (planner->start())
  {
    ROS_INFO_STREAM_NAMED("PlannerManager", "Started " << planner->getPlanner()->getName().c_str());
    return 0;
  }

  return -1;  // Unable to start
}

int PlannerManager::requestStop(const std::string& name)
{
  // Find planner
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    if ((*p)->getPlanner()->getName() == name)
    {
      // Stop planner (with force)
      if ((*p)->stop(true))
      {
        ROS_INFO_STREAM_NAMED("PlannerManager", "Stopped " << (*p)->getPlanner()->getName().c_str());
        return 0;
      }
      else
      {
        return -1;  // planner decided not to stop
      }
    }
  }
  return -1;  // no such planner
}

void PlannerManager::update(const ros::Time& time, const ros::Duration& dt)
{
  // Reset handles
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
    (*j)->reset();

  // Update planners
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    (*p)->update(time, dt);
  }
}

void PlannerManager::reset()
{
  // Update planners
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    (*p)->reset();
  }
}

bool PlannerManager::addJointHandle(JointHandlePtr& j)
{
  // TODO: check for duplicate names?
  joints_.push_back(j);
  return true;
}

HandlePtr PlannerManager::getHandle(const std::string& name)
{
  // Try joints first
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
  {
    if ((*j)->getName() == name)
      return *j;
  }

  // Then planners
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    if ((*p)->getPlanner()->getName() == name)
      return (*p)->getPlanner();
  }

  // Not found
  return HandlePtr();
}

JointHandlePtr PlannerManager::getJointHandle(const std::string& name)
{
  // Try joints first
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
  {
    if ((*j)->getName() == name)
      return *j;
  }

  // Not found
  return JointHandlePtr();
}

PlannerPtr PlannerManager::getPlanner(const std::string& name)
{
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    if ((*p)->getPlanner()->getName() == name)
      return (*p)->getPlanner();
  }

  // Not found
  return PlannerPtr();
}

std::vector<std::string> PlannerManager::getPlannerNames()
{
  std::vector<std::string> planner_names;
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    planner_names.push_back((*p)->getPlanner()->getName());
  }
  return planner_names;
}


void PlannerManager::execute(const robot_controllers_msgs::QueryPlannerStatesGoalConstPtr& goal)
{
  robot_controllers_msgs::QueryPlannerStatesFeedback feedback;
  robot_controllers_msgs::QueryPlannerStatesResult result;

  for (size_t i = 0; i < goal->updates.size(); i++)
  {
    // Update this planner
    robot_controllers_msgs::PlannerState state = goal->updates[i];

    // Make sure planner exists
    bool in_planner_list = false;
    for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
    {
      if ((*p)->getPlanner()->getName() == state.name)
      {
        if (state.type != "")
        {
          if (state.type == (*p)->getPlanner()->getType())
          {
            in_planner_list = true;
            break;
          }
          else
          {
            std::stringstream ss;
            ss << "Planner " << state.name << " is of type " << (*p)->getPlanner()->getType() << " not " << state.type;
            getState(result);
            server_->setAborted(result, ss.str());
            return;
          }
        }
        in_planner_list = true;
        break;
      }
    }
    if (!in_planner_list)
    {
      // Check if planner exists on parameter server
      ros::NodeHandle nh;
      if (nh.hasParam(state.name))
      { 
        // Create planner (in a loader)
        if (!load(static_cast<std::string>(state.name)))
        {
          std::stringstream ss;
          ss << "Failed to load planner: " << state.name;
          getState(result);
          server_->setAborted(result, ss.str());
          return;
        }
      }
      else
      {
        std::stringstream ss;
        ss << "No such planner to update: " << state.name;
        getState(result);
        server_->setAborted(result, ss.str());
        return;
      }
    }

    // Update state
    if (state.state == state.STOPPED)
    {
      if (requestStop(state.name) != 0)
      {
        std::stringstream ss;
        ss << "Unable to stop " << state.name;
        getState(result);
        server_->setAborted(result, ss.str());
        return;
      }
    }
    else if (state.state == state.RUNNING)
    {
      if (requestStart(state.name) != 0)
      {
        std::stringstream ss;
        ss << "Unable to start " << state.name;
        getState(result);
        server_->setAborted(result, ss.str());
        return;
      }
    }
    else
    {
      std::stringstream ss;
      ss << "Invalid state for planner " << state.name << ": " << static_cast<int>(state.state);
      getState(result);
      server_->setAborted(result, ss.str());
      return;
    }
  }

  // Send result
  getState(result);
  server_->setSucceeded(result);
}

void PlannerManager::getState(
    robot_controllers_msgs::QueryPlannerStatesResult& result)
{
  result.state.clear();
  for (PlannerList::iterator p = planners_.begin(); p != planners_.end(); p++)
  {
    robot_controllers_msgs::PlannerState state;
    state.name = (*p)->getPlanner()->getName();
    state.type = (*p)->getPlanner()->getType();
    if ((*p)->isActive())
    {
      state.state = state.RUNNING;
    }
    else
    {
      state.state = state.STOPPED;
    }
    result.state.push_back(state);
  }
}

// NOTE: this function should be called only by one thread
bool PlannerManager::load(const std::string& name)
{
  // Create planner (in a loader)
  PlannerLoaderPtr planner(new PlannerLoader());
  // Push back planner (so that autostart will work)
  planners_.push_back(planner);
  // Now initialize planner
  if (!planner->init(name, this))
  {
    // Remove if init fails
    planners_.pop_back();
    return false;
  }
  return true;
}

}  // namespace robot_controllers
