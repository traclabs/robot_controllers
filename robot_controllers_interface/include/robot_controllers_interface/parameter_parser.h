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

typedef std::map<std::string, std::string> ParamGroup;

namespace robot_controllers
{
class ParameterParser
{
  void findFilesInDir(std::string path, std::vector<std::string>& files_found);
  void expandParamStruct(XmlRpc::XmlRpcValue& val, std::string& param_name);
  void expandParamArray(XmlRpc::XmlRpcValue& val, std::string& param_name);

  ros::NodeHandle nh_;
  std::string controller_name_;
  std::string filename_;
  std::string pkg_path_;
  std::string file_path_;
  std::string parsing_type_;

  std::map<std::string, std::vector<std::string> > top_level_type_names_;
  std::map<std::string, std::vector<ParamGroup> > type_name_params_;

  std::map<std::string, std::string> dynamic_strings_;
  std::map<std::string, double> dynamic_doubles_;
  std::map<std::string, int> dynamic_ints_;
  std::map<std::string, bool> dynamic_bools_;

  std::map<std::string, XmlRpc::XmlRpcValue> dynamic_param_vals_;

public:
  ParameterParser(const ros::NodeHandle _nh, const std::string name, const std::string _type);
  ~ParameterParser();

  bool parseYamlParams(const std::string param_base);
  bool parseFileParams(const std::string rospkg, const std::string file);
};
}

#endif
