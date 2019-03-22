#include <robot_controllers_interface/parameter_parser.h>

using namespace robot_controllers;

ParameterParser::ParameterParser(const ros::NodeHandle _nh, const std::string _type) :
  nh_(_nh),
  parsing_type_(_type)
{
  ROS_INFO_STREAM("ParameterParser() -- creating parser for: " << parsing_type_);
}


ParameterParser::~ParameterParser()
{

}

  // std::string at_lib;
  // bool found_at_lib = false;
  // // Find at_lib
  // for (auto lib_i : library_pkgs_)
  // {
  //   if (_at_full_filename.find(lib_i) != std::string::npos)
  //   {
  //     at_lib = lib_i;
  //     found_at_lib = true;
  //     break;
  //   }
  // }
  // if (!found_at_lib)
  // {
  //   ROS_WARN("[loadTemplateFilename] Didn't find at_lib");
  //   return false;
  // }
  // std::lock_guard<std::mutex> lock(structure_map_mutex_);
  // boost::filesystem::path p(_at_full_filename);
  // std::string filename = p.filename().string();

  // if (!boost::filesystem::exists(p))
  // {
  //   ROS_WARN("loadTemplateFilename: AT filename %s is not a valid file", _at_full_filename.c_str());
  //   return false;
  // }
  // if (p.extension() != std::string(".json"))
  // {
  //   ROS_WARN("loadTemplateFilename: File %s does not have .json extension!", filename.c_str());
  //   return false;
  // }
bool ParameterParser::parseParams(const std::string rospkg, const std::string file)
{
  ROS_INFO_STREAM("ParameterParser::parseParams() -- parsing parameters from file: " << file << " in rospkg: " << rospkg);
  pkg_path_ = rospkg;
  file_path_ = file;

  return true;
}