#ifndef PARAMETER_PARSER_H
#define PARAMETER_PARSER_H

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

typedef std::map<std::string, std::string> ParamGroup;

namespace robot_controllers
{

const std::string TypeString[] =
{
  "TypeInvalid", "TypeBoolean", "TypeInt", "TypeDouble", "TypeString",
  "TypeDateTime", "TypeBase64", "TypeArray", "TypeStruct"
};

class ParameterParser
{
  void setParams(std::string param_name, std::string name_str, std::map<std::string, XmlRpc::XmlRpcValue>& param_map);
  void findFilesInDir(std::string path, std::vector<std::string>& files_found);
  void expandParamStruct(XmlRpc::XmlRpcValue& val, std::string& param_name);
  void expandParamArray(XmlRpc::XmlRpcValue& val, std::string& param_name);
  void paramUpdatesCallback(const ros::TimerEvent&);

  ros::NodeHandle nh_;

  ros::Timer param_update_timer_;

  std::string manager_name_;
  std::string filename_;
  std::string pkg_path_;
  std::string file_path_;
  std::string parsing_type_;

  std::map<std::string, std::vector<std::string> > top_level_type_names_;
  std::map<std::string, std::vector<ParamGroup> > type_name_params_;

  std::map<std::string, XmlRpc::XmlRpcValue> dynamic_param_vals_;

  // std::map<std::string, std::string> names_to_params_;  // e.g. "/valkyrie_arm/arm/craftsman_controllers/CartesianPoseController/params/fb_trans/p_gain/value" : "fb_trans/p_gain"
  std::map<std::string, XmlRpc::XmlRpcValue> params_to_monitor_;  // e.g. "/valkyrie_arm/arm/craftsman_controllers/CartesianPoseController/params/fb_trans/p_gain/value" : XmlRpc::TypeDouble

  // look into having dynamic_other_maps
  // dynamic_doubles_ e.g. "fb_trans/p_gain" : *(&fb_trans_p_value)
  std::map<std::string, double*> dynamic_doubles_;
  std::map<std::string, int*> dynamic_integers_;
  std::map<std::string, std::string*> dynamic_strings_;
  std::map<std::string, bool*> dynamic_bools_;

public:
  ParameterParser(const ros::NodeHandle _nh, const std::string name, const std::string _type);
  ~ParameterParser();

  bool parseYamlParams(const std::string param_base);
  bool parseFileParams(const std::string rospkg, const std::string file);

  // model after this
  // parser_->registerDoubleRange("fb_trans/p_gain", min_val_, max_val_, cur_val_, "spinbox", &trans_p_gain_);
  bool registerDouble(const std::string param, double* ptr);

};
}  // namespace robot_controllers

#endif
