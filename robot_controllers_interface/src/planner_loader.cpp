// Author: Stephen Hart

#include <robot_controllers_interface/planner_loader.h>

namespace robot_planners
{

PlannerLoader::PlannerLoader() :
    plugin_loader_("robot_controllers", "robot_planners::Planner"),
    active_(false)
{
}

bool PlannerLoader::init(const std::string& name, PlannerManager* manager)
{
  ros::NodeHandle nh(name);
  std::string planner_type;

  if (nh.getParam("type", planner_type))
  {
    // If plugin is bad, catch pluginlib exception
    try
    {
      planner_ = plugin_loader_.createInstance(planner_type);
      planner_->init(nh, manager);
    }
    catch (pluginlib::LibraryLoadException e)
    {
      ROS_ERROR_STREAM("Plugin error while loading planner: " << e.what());
      return false;
    }
    return true;
  }
  ROS_ERROR_STREAM("Unable to load planner " << name.c_str() << " of type: " << planner_type);
  return false;
}

bool PlannerLoader::start()
{
  active_ = planner_->start();
  return active_;
}

bool PlannerLoader::stop(bool force)
{
  bool stopped = planner_->stop(force);
  if (stopped)
  {
    active_ = false;
  }
  return stopped;
}

bool PlannerLoader::reset()
{
  if (active_)
  {
    return planner_->reset();
  }
  return true;
}

bool PlannerLoader::isActive()
{
  return active_;
}

void PlannerLoader::update(const ros::Time& time, const ros::Duration& dt)
{
  if (active_)
  {
    planner_->update(time, dt);
  }
}

PlannerPtr PlannerLoader::getPlanner()
{
  return planner_;
}

}  // namespace robot_planners
