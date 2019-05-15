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

#include <craftsman_msgs/ParamMap.h>
#include <craftsman_msgs/ParamFile.h>
#include <craftsman_msgs/ParamKeyVal.h>
#include <craftsman_msgs/ParamRestore.h>


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
  void expandParamStruct(XmlRpc::XmlRpcValue& val, std::string& param_name);
  void expandParamArray(XmlRpc::XmlRpcValue& val, std::string& param_name);
  void paramUpdatesCallback(const ros::TimerEvent&);
  void findFilesInDir(std::string path, std::vector<std::string>& files_found);
  bool saveService(craftsman_msgs::ParamFile::Request &req, craftsman_msgs::ParamFile::Response &res);
  bool loadService(craftsman_msgs::ParamFile::Request &req, craftsman_msgs::ParamFile::Response &res);
  bool restoreService(craftsman_msgs::ParamRestore::Request &req, craftsman_msgs::ParamRestore::Response &res);
  bool loadFromFile(std::string file);
  bool restoreFileParams(std::string type);
  bool restoreDefaultParams(std::string type);
  std::string toString(std::string str);

  ros::NodeHandle nh_;

  ros::Timer param_update_timer_;

  ros::ServiceServer save_srv_;
  ros::ServiceServer load_srv_;

  // class members
  std::string manager_name_;
  std::string parsing_type_;

  // file parsing members
  std::string filename_;
  std::string pkg_path_;
  std::string file_path_;

  // e.g. "/valkyrie_arm/arm/craftsman_controllers/CartesianPoseController/params/fb_trans/p_gain/value" : XmlRpc::TypeDouble  // NOLINT
  std::map<std::string, XmlRpc::XmlRpcValue> dynamic_param_vals_;  // hold current values
  std::map<std::string, XmlRpc::XmlRpcValue> default_param_vals_;  // these will be default as defined in the the controller/planner files when registering
  std::map<std::string, XmlRpc::XmlRpcValue> file_param_vals_;  // these will start out as the default values, will be updated if there is a save file loaded in

  // dynamic maps e.g. "fb_trans/p_gain" : *fb_trans_p_value
  std::map<std::string, std::shared_ptr<bool>> dynamic_bools_;
  std::map<std::string, std::shared_ptr<int>> dynamic_integers_;
  std::map<std::string, std::shared_ptr<double>> dynamic_doubles_;
  std::map<std::string, std::shared_ptr<std::string> > dynamic_strings_;
  std::map<std::string, std::pair<std::shared_ptr<std::string>, std::vector<std::string> > > dynamic_options_;

public:
  ParameterParser(const ros::NodeHandle _nh, const std::string name, const std::string _type);
  ~ParameterParser();

  bool parseYamlParams(const std::string param_base);
  bool parseFileParams(const std::string rospkg, const std::string file);

  bool registerBool(const std::string param, std::shared_ptr<bool> ptr, std::string ui_type="radio");  // NOLINT
  bool registerEnum(const std::string param, std::shared_ptr<std::string> ptr, std::vector<std::string> options);  // NOLINT
  bool registerString(const std::string param, std::shared_ptr<std::string> ptr, std::string ui_type="textedit");  // NOLINT
  bool registerInt(const std::string param, std::shared_ptr<int> ptr, int min=0, int max=1, std::string ui_type="slider");  // NOLINT
  bool registerDouble(const std::string param, std::shared_ptr<double> ptr, double min=0.0, double max=1.0, std::string ui_type="spinbox");  // NOLINT
};
}  // namespace robot_controllers

#endif
