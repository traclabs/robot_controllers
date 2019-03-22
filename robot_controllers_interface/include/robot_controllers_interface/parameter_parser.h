#ifndef PARAMETER_PARSER_H
#define PARAMETER_PARSER_H

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>

// #include <boost/thread.hpp>
// #define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
// #include <boost/algorithm/string.hpp>


namespace robot_controllers
{
class ParameterParser
{
  void findFilesInDir(std::string path, std::vector<std::string>& files_found);

  ros::NodeHandle nh_;
  std::string filename_;
  std::string pkg_path_;
  std::string file_path_;
  std::string parsing_type_;

public:
  ParameterParser(const ros::NodeHandle _nh, const std::string _type);
  ~ParameterParser();

  bool parseParams(const std::string rospkg, const std::string file);
};
}

#endif
