#include <robot_controllers_interface/parameter_parser.h>


using namespace robot_controllers;


ParameterParser::ParameterParser(const ros::NodeHandle _nh, const std::string _name, const std::string _type) :
  nh_(_nh),
  manager_name_(_name),
  parsing_type_(_type)
{
  ROS_INFO_STREAM("ParameterParser() -- creating parser for: \'"
                  << parsing_type_ << "\' for \'" << manager_name_ << "\'");
  param_update_timer_ = nh_.createTimer(ros::Duration(1), &ParameterParser::paramUpdatesCallback, this, false);
  param_update_timer_.start();

  // todo make sure we aren't already advertising these services
  save_srv_ = nh_.advertiseService("/parameter_parser/save_reconfigure_values", &ParameterParser::saveService, this);
  load_srv_ = nh_.advertiseService("/parameter_parser/load_reconfigure_values", &ParameterParser::loadService, this);
}


ParameterParser::~ParameterParser()
{
  param_update_timer_.stop();
}


void ParameterParser::paramUpdatesCallback(const ros::TimerEvent&)
{
  if (dynamic_param_vals_.empty())
  {
    return;
  }

  for (auto &param : dynamic_param_vals_)
  {
    XmlRpc::XmlRpcValue xmlval;
    if (nh_.getParamCached(param.first, xmlval))
    {
      if (xmlval.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        std::string type_str = static_cast<std::string>(xmlval);
        std::string compare_str = static_cast<std::string>(param.second);
        if (type_str != compare_str)
        {
          param.second = xmlval;

          // for text edit strings only
          for (auto &dyn : dynamic_strings_)
          {
            if (param.first.find(dyn.first) != std::string::npos)
            {
              std::string expanded_param_name = param.first + "/" + type_str;
              ROS_INFO_STREAM("ParameterParser::paramUpdatesCallback() -- updating STRING: "
                              << type_str << " under parameter: " << expanded_param_name);
              *dyn.second = type_str;
              break;
            }
          }

          // for dropdowns
          for (auto &dyn : dynamic_options_)
          {
            if (param.first.find(dyn.first) != std::string::npos)
            {
              ROS_INFO_STREAM("ParameterParser::paramUpdatesCallback() -- updating STRING: "
                              << type_str << " under parameter: " << param.first);
              *dyn.second.first = type_str;
              break;
            }
          }
        }
      }
      else if (xmlval.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {
        double param_val = static_cast<double>(xmlval);
        double compare_val = static_cast<double>(param.second);
        if (param_val != compare_val)
        {
          ROS_INFO_STREAM("ParameterParser::paramUpdatesCallback() -- updating DOUBLE: "
                           << param_val << " under parameter: " << param.first);
          param.second = xmlval;

          for (auto &dyn : dynamic_doubles_)
          {
            if (param.first.find(dyn.first) != std::string::npos)
            {
              *dyn.second = param_val;
              break;
            }
          }
        }
      }
      else if (xmlval.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        int param_val = static_cast<int>(xmlval);
        int compare_val = static_cast<int>(param.second);
        if (param_val != compare_val)
        {
          ROS_INFO_STREAM("ParameterParser::paramUpdatesCallback() -- updating INT: "
                           << param_val << " under parameter: " << param.first);
          param.second = xmlval;

          for (auto &dyn : dynamic_integers_)
          {
            if (param.first.find(dyn.first) != std::string::npos)
            {
              *dyn.second = param_val;
              break;
            }
          }
        }
      }
      else if (xmlval.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      {
        bool param_val = static_cast<bool>(xmlval);
        bool compare_val = static_cast<bool>(param.second);
        if (param_val != compare_val)
        {
          ROS_INFO_STREAM("ParameterParser::paramUpdatesCallback() -- updating BOOL: "
                           << param_val << " under parameter: " << param.first);
          param.second = xmlval;

          for (auto &dyn : dynamic_bools_)
          {
            if (param.first.find(dyn.first) != std::string::npos)
            {
              *dyn.second = param_val;
              break;
            }
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM("ParameterParser::paramUpdatesCallback() -- unexpected XmlRpc type: "
                         << TypeString[xmlval.getType()]);
      }
    }
    else
    {
      ROS_WARN_STREAM("ParameterParser::paramUpdatesCallback() -- parameter \'"
                      << param.first << "\' not found on server");
    }
  }
}


bool ParameterParser::parseYamlParams(const std::string param_base)
{
  XmlRpc::XmlRpcValue base_param;
  if (nh_.getParamCached(param_base, base_param))
  {
    if (!base_param.valid())
    {
      ROS_ERROR_STREAM("ParameterParser::parseYamlParams() -- param \'" << param_base << "\' is not valid!");
      return false;
    }

    std::string param_name = "/" + manager_name_;
    if (base_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR_STREAM("ParameterParser::parseYamlParams() -- \'"
                       << param_base << "\' is not a TypeArray; fix the .yaml structure");
      return false;
    }

    // go through the array entries
    for (auto i = 0; i < base_param.size(); ++i)
    {
      if (base_param[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_STREAM("ParameterParser::parseYamlParams() -- invalid yaml \'"
                         << param_name << "\' should be TypeStruct");
        return false;
      }

      // parse first struct
      // we really only care about the "params: " entry
      // but want to match up the correct type that matches to the manager name
      XmlRpc::XmlRpcValue xmlrpc_params;
      std::string expanded_str;
      std::string manager_type;
      for (auto v : base_param[i])
      {
        std::string val_str = static_cast<std::string>(v.first);
        if ((v.second.getType() == XmlRpc::XmlRpcValue::TypeStruct
            || v.second.getType() == XmlRpc::XmlRpcValue::TypeArray)
            && val_str == "params")
        {
          expanded_str = param_name + "/" + val_str;
          xmlrpc_params = v.second;
        }
        else if (val_str == "type")
        {
          manager_type = static_cast<std::string>(v.second);
        }
      }

      if (xmlrpc_params.valid() && (manager_name_.find(manager_type) != std::string::npos))
      {
        if (xmlrpc_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          expandParamStruct(xmlrpc_params, expanded_str);
        }
        else if (xmlrpc_params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          expandParamArray(xmlrpc_params, expanded_str);
        }
        else
        {
          ROS_ERROR_STREAM("ParameterParser::parseYamlParams() -- invalid type: "
                           << TypeString[xmlrpc_params.getType()]);
        }
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM("ParameterParser::parseYamlParams() -- no param \'"
                     << param_base << "\' found on server");
    return false;
  }
  return true;
}


void ParameterParser::expandParamStruct(XmlRpc::XmlRpcValue& val, std::string& param_name)
{
  // temporarily hold onto params that we read in
  // we need to find the name of the param to build the string
  std::string named_param = "";
  std::map<std::string, XmlRpc::XmlRpcValue> tmp_to_add;

  for (auto v : val)
  {
    std::string val_str = static_cast<std::string>(v.first);
    std::string expanded_param_name = param_name + "/" + val_str;

    if (v.second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      return expandParamStruct(v.second, expanded_param_name);
    }
    else if (v.second.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (auto i=0; i < v.second.size(); ++i)
      {
        if (v.second[i].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          std::string tmp_str = val_str + "/" + static_cast<std::string>(v.second[i]);
          tmp_to_add[tmp_str] = v.second[i];
        }
        else
        {
          ROS_WARN_STREAM("ParameterParser::expandParamStruct() -- error reading in parameter for \'"
                          << val_str << "\'");
        }
      }
    }
    else if (v.second.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      if (val_str.find("name") != std::string::npos)
      {
        std::string tmp_str = static_cast<std::string>(v.second);
        named_param = tmp_str;
      }
      else
      {
        tmp_to_add[v.first] = v.second;
      }
    }
    else
    {
      tmp_to_add[v.first] = v.second;
    }
  }

  if (!tmp_to_add.empty())
  {
    setParams(param_name, named_param, tmp_to_add);
  }
  else
  {
    ROS_WARN_STREAM("ParameterParser::expandParamStruct() -- no parameters to add to param server");
  }
}


void ParameterParser::expandParamArray(XmlRpc::XmlRpcValue& val, std::string& param_name)
{
  // temporarily hold onto params that we read in
  // we need to find the name of the param to build the string
  std::string named_param = "";
  std::map<std::string, XmlRpc::XmlRpcValue> tmp_to_add;

  for (auto i=0; i < val.size(); ++i)
  {
    if (val[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      expandParamStruct(val[i], param_name);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      return expandParamArray(val[i], param_name);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string type_str = static_cast<std::string>(val[i]);
      std::string expanded_param_name = param_name + "/" + type_str;
      ROS_INFO_STREAM("ParameterParser::expandParamArray() -- added dynamic STRING: "
                       << type_str << " under parameter: " << expanded_param_name);
      dynamic_param_vals_[expanded_param_name] = val[i];
      nh_.setParam(expanded_param_name, type_str);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      double param_val = static_cast<double>(val[i]);
      ROS_INFO_STREAM("ParameterParser::expandParamArray() -- added dynamic DOUBLE: "
                       << param_val << " under parameter: " << param_name);
      dynamic_param_vals_[param_name] = val[i];
      nh_.setParam(param_name, param_val);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int param_val = static_cast<int>(val[i]);
      ROS_INFO_STREAM("ParameterParser::expandParamArray() -- added dynamic INT: "
                       << param_val << " under parameter: " << param_name);
      dynamic_param_vals_[param_name] = val[i];
      nh_.setParam(param_name, param_val);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      bool param_val = static_cast<bool>(val[i]);
      ROS_INFO_STREAM("ParameterParser::expandParamArray() -- added dynamic BOOL: "
                       << param_val << " under parameter: " << param_name);
      dynamic_param_vals_[param_name] = val[i];
      nh_.setParam(param_name, param_val);
    }
    else
    {
      ROS_ERROR_STREAM("ParameterParser::expandParamArray() -- unexpected XmlRpc type: "
                       << TypeString[val[i].getType()]);
    }
  }
}


void ParameterParser::setParams(std::string param_name,
                                std::string name_str,
                                std::map<std::string, XmlRpc::XmlRpcValue>& param_map)
{
  // assuming we have a name now
  // post the params to the rosparam server
  // note: it _does_ still work without a name being set for params but you run a risk of params
  //   being overwritten if they share a name like two params having a "value" entry without having a name

  for (auto param : param_map)
  {
    std::string full_param_name;
    if (name_str.empty())
      full_param_name = param_name + "/" + param.first;
    else
      full_param_name = param_name + "/" + name_str + "/" + param.first;

    dynamic_param_vals_[full_param_name] = param.second;

    if (full_param_name.find("value") != std::string::npos)
    {
      dynamic_param_vals_[full_param_name] = param.second;
    }

    if (param.second.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string param_val = static_cast<std::string>(param.second);
      nh_.setParam(full_param_name, param_val);
      ROS_INFO_STREAM("ParameterParser::setParams() -- added dynamic STRING: "
                       << param_val << " under parameter: " << full_param_name);
    }
    else if (param.second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      double param_val = static_cast<double>(param.second);
      nh_.setParam(full_param_name, param_val);
      ROS_INFO_STREAM("ParameterParser::setParams() -- added dynamic DOUBLE: "
                       << param_val << " under parameter: " << full_param_name);
    }
    else if (param.second.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int param_val = static_cast<int>(param.second);
      nh_.setParam(full_param_name, param_val);
      ROS_INFO_STREAM("ParameterParser::setParams() -- added dynamic INT: "
                       << param_val << " under parameter: " << full_param_name);
    }
    else if (param.second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      bool param_val = static_cast<bool>(param.second);
      nh_.setParam(full_param_name, param_val);
      ROS_INFO_STREAM("ParameterParser::setParams() -- added dynamic BOOL: "
                       << param_val << " under parameter: " << full_param_name);
    }
    else
    {
      ROS_ERROR_STREAM("ParameterParser::setParams() -- unexpected XmlRpc type: "
                       << TypeString[param.second.getType()]);
    }
  }
}


// "spinbox" is default ui_type
// 0.0 is default min
// 1.0 is default max
bool ParameterParser::registerDouble(const std::string param,
                                     std::shared_ptr<double> ptr,
                                     double min, double max,
                                     std::string ui_type)
{
  if (ptr == NULL)
  {
    return false;
  }

  ROS_INFO_STREAM("ParameterParser::registerDouble() -- registering param: \'" << param <<
                  "\' with value: \'" << *ptr << "\' at address: " << ptr);

  std::string param_base = ((param.at(0) == '/') ? "params" : "params/") + param;
  std::string param_value = param_base + "/value";

  nh_.setParam(param_value, *ptr);
  nh_.setParam((param_base + "/min_val"), min);
  nh_.setParam((param_base + "/max_val"), max);
  nh_.setParam((param_base + "/ui_type"), ui_type);
  nh_.setParam((param_base + "/type"), "double_range");

  dynamic_doubles_[param] = ptr;

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);

  return true;
}


// "radio" is default ui_type
bool ParameterParser::registerBool(const std::string param,
                                   std::shared_ptr<bool> ptr,
                                   std::string ui_type)
{
  if (ptr == NULL)
  {
    return false;
  }

  ROS_INFO_STREAM("ParameterParser::registerBool() -- registering param: \'" << param <<
                  "\' with value: \'" << *ptr << "\' at address: " << ptr);

  std::string param_base = ((param.at(0) == '/') ? "params" : "params/") + param;
  std::string param_value = param_base + "/value";

  nh_.setParam(param_value, *ptr);
  nh_.setParam((param_base + "/ui_type"), ui_type);

  dynamic_bools_[param] = ptr;

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);

  return true;
}


// "textedit" is default ui_type
bool ParameterParser::registerString(const std::string param,
                                     std::shared_ptr<std::string> ptr,
                                     std::string ui_type)
{
  if (ptr == NULL)
  {
    return false;
  }

  ROS_INFO_STREAM("ParameterParser::registerString() -- registering param: \'" << param <<
                  "\' with value: \'" << *ptr << "\' at address: " << ptr);

  std::string param_base = ((param.at(0) == '/') ? "params" : "params/") + param;
  std::string param_value = param_base + "/value";

  nh_.setParam(param_value, *ptr);
  nh_.setParam((param_base + "/ui_type"), ui_type);

  dynamic_strings_[param] = ptr;

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);

  return true;
}


// "slider" is default ui_type
// 0 is default min
// 1 is default max
bool ParameterParser::registerInt(const std::string param,
                                  std::shared_ptr<int> ptr,
                                  int min, int max,
                                  std::string ui_type)
{
  if (ptr == NULL)
  {
    return false;
  }

  ROS_INFO_STREAM("ParameterParser::registerInt() -- registering param: \'" << param <<
                  "\' with value: \'" << *ptr << "\' at address: " << ptr);

  std::string param_base = ((param.at(0) == '/') ? "params" : "params/") + param;
  std::string param_value = param_base + "/value";

  nh_.setParam(param_value, *ptr);
  nh_.setParam((param_base + "/min_val"), min);
  nh_.setParam((param_base + "/max_val"), max);
  nh_.setParam((param_base + "/ui_type"), ui_type);
  nh_.setParam((param_base + "/type"), "int_range");

  dynamic_integers_[param] = ptr;

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);

  return true;
}


// ui_type has to be dropdown
// ptr* should have the default selected option
bool ParameterParser::registerEnum(const std::string param,
                                   std::shared_ptr<std::string> ptr,
                                   std::vector<std::string> options)
{
  if (ptr == NULL)
  {
    return false;
  }

  ROS_INFO_STREAM("ParameterParser::registerDouble() -- registering param: \'" << param <<
                  "\' with value: \'" << *ptr << "\' at address: " << ptr);

  if (options.empty())
  {
    ROS_WARN_STREAM("ParameterParser::registerEnum() -- empty options list, adding \'"
                    << *ptr << "\' as the only option");
    options.push_back(*ptr);
  }

  std::string param_base = ((param.at(0) == '/') ? "params" : "params/") + param;
  std::string param_value = param_base + "/value";

  nh_.setParam(param_value, *ptr);
  nh_.setParam((param_base + "/options"), options);
  nh_.setParam((param_base + "/ui_type"), "dropdown");

  dynamic_options_[param] = std::make_pair(ptr, options);

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);

  return true;
}


bool ParameterParser::saveService(craftsman_msgs::ParamFile::Request  &req,
                                  craftsman_msgs::ParamFile::Response &res)
{
  ROS_DEBUG_STREAM("ParameterParser::saveService() -- saving parameters for robot: \'" << req.robot << "\'"
                  << ", group: \'" << req.group << "\' for type: \'" << req.type << "\' in pkg: " << req.pkg);

  std::string yaml_path = ros::package::getPath("craftsman_common");
  if (yaml_path.empty())
  {
    ROS_ERROR_STREAM("ParameterParser::parseFileParams() -- couldn't find path to package");
    return false;
  }

  std::string file = toString(req.robot + "_" + req.group + "_" + req.type);
  std::string fileext = file + ".dat";
  std::string filepath = yaml_path + "/config/" + fileext;
  ROS_INFO_STREAM("ParameterParser::saveService() -- saving parameters to: " << filepath);

  // convert our map of params we monitor to a ROS msg for easy serialization
  craftsman_msgs::ParamMap param_map;
  param_map.type = file;
  for (auto dpv : dynamic_param_vals_)
  {
    craftsman_msgs::ParamKeyVal param;
    param.key = dpv.first;
    param.value = dpv.second.toXml();
    param_map.keyvalues.push_back(param);
  }

  // write parameters to (binary) file
  try
  {
    std::ofstream ofs(filepath, std::ios::out | std::ios::binary);

    uint32_t serial_size = ros::serialization::serializationLength(param_map);
    boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, param_map);
    ofs.write(reinterpret_cast<char*>(obuffer.get()), serial_size);

    ofs.close();
  }
  catch (...)
  {
    ROS_ERROR_STREAM("ParameterParser::saveService() -- error writing file");
    return true;
  }

  if (!loadFromFile(file))
  {
    ROS_ERROR_STREAM("ParameterParser::saveService() -- could not load recently saved parameters!");
  }

  return true;
}


bool ParameterParser::loadService(craftsman_msgs::ParamFile::Request  &req,
                                  craftsman_msgs::ParamFile::Response &res)
{
  ROS_INFO_STREAM("ParameterParser::loadService() -- loading parameters for robot: \'" << req.robot << "\'"
                  << ", group: \'" << req.group << "\' of type: \'" << req.type << "\'");

  std::string file = req.robot + "_" + req.group + "_" + req.type;
  res.success = loadFromFile(toString(file));

  return true;
}


std::string ParameterParser::toString(std::string str)
{
  std::size_t found = str.find_first_of("/");

  if (found == std::string::npos)
    return str;

  while (found != std::string::npos)
  {
    str[found] = '_';
    found = str.find_first_of("/", found + 1);
  }

  return str;
}


bool ParameterParser::loadFromFile(std::string file)
{
  ROS_DEBUG_STREAM("ParameterParser::loadFromFile() -- attempting to load saved parameters from file: \'"
                   << file << "\'");

  std::string yaml_path = ros::package::getPath("craftsman_common");
  if (yaml_path.empty())
  {
    ROS_ERROR_STREAM("ParameterParser::loadFromFile() -- couldn't find path to package");
    return false;
  }

  std::string fileext = file + ".dat";
  std::string filepath = yaml_path + "/config/" + fileext;

  // read from bin info file
  try
  {
    craftsman_msgs::ParamMap param_map;

    std::ifstream ifs(filepath, std::ios::in | std::ios::binary);
    ifs.seekg(0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::streampos begin = ifs.tellg();

    uint32_t file_size = end - begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read(reinterpret_cast<char*>(ibuffer.get()), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);
    ros::serialization::deserialize(istream, param_map);
    ifs.close();

    if (param_map.type == file)
    {
      for (auto keyval : param_map.keyvalues)
      {
        if (dynamic_param_vals_.find(keyval.key) != dynamic_param_vals_.end())
        {
          XmlRpc::XmlRpcValue xmlval;
          int offset = 0;
          int* offset_ptr = &offset;
          if (xmlval.fromXml(keyval.value, offset_ptr))
          {
            ROS_INFO_STREAM("ParameterParser::loadFromFile() -- found saved value for parameter: \'"
                            << keyval.key << "\'");
            dynamic_param_vals_[keyval.key] = xmlval;
            nh_.setParam(keyval.key, xmlval);
          }
          else
          {
            ROS_ERROR_STREAM("ParameterParser::loadFromFile() -- couldnt convert XML: \'" << keyval.value << "\'");
          }
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("ParameterParser::loadFromFile() -- couldn't match type: \'" << param_map.type
                      << "\' to \'" << toString(manager_name_) << "\'");
    }
  }
  catch (...)
  {
    ROS_WARN_STREAM("ParameterParser::loadFromFile() -- failed to read file: \'"
                    << fileext << "\'; file may not exist");
    return false;
  }

  return true;
}


//
// TODO -- this may be worth adding later
// bool ParameterParser::parseFileParams(const std::string rospkg, const std::string file)
// {
//   pkg_path_ = rospkg;
//   filename_ = file;

//   ROS_INFO_STREAM("ParameterParser::parseFileParams() -- parsing parameters from file: "
//                   << filename_ << " in rospkg: " << pkg_path_);

//   std::string yaml_path = ros::package::getPath(pkg_path_);
//   if (yaml_path.empty())
//   {
//     ROS_ERROR_STREAM("ParameterParser::parseFileParams() -- couldn't find path to package: " << pkg_path_);
//     return false;
//   }

//   // crawl directories looking for the config file
//   std::vector<std::string> file_list;
//   boost::filesystem::path curerent_path(yaml_path);
//   if (boost::filesystem::exists(curerent_path))
//   {
//     if (boost::filesystem::is_directory(curerent_path))
//     {
//       for (boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(curerent_path))
//       {
//         if (boost::filesystem::is_directory(x.path()))
//         {
//           findFilesInDir(x.path().string(), file_list);
//         }
//         else
//         {
//           file_list.push_back(x.path().string());
//         }
//       }
//     }
//   }

//   for (auto fp : file_list)
//   {
//     if (fp.find(filename_) != std::string::npos)
//     {
//       ROS_INFO_STREAM("ParameterParser::parseFileParams() -- found config file at: " << fp);
//       file_path_ = fp;
//       break;
//     }
//   }

//   return true;
// }


// void ParameterParser::findFilesInDir(std::string path, std::vector<std::string>& files_found)
// {
//   ROS_INFO_STREAM("ParameterParser::findFilesInDir() -- searching directory: \'" << path << "\' for files");

//   boost::filesystem::path dir_path(path);
//   if (boost::filesystem::exists(dir_path))
//   {
//     if (boost::filesystem::is_directory(dir_path))
//     {
//       for (boost::filesystem::directory_entry& dir_entry : boost::filesystem::directory_iterator(dir_path))
//       {
//         // check if the path is the same as our pkg and filename
//         if (boost::filesystem::is_directory(dir_entry.path()))
//         {
//           ROS_INFO_STREAM("ParameterParser::findFilesInDir() -- found new directory to search...");
//           return findFilesInDir(dir_entry.path().string(), files_found);
//         }
//         else
//         {
//           files_found.push_back(dir_entry.path().string());
//         }
//       }
//     }
//     else
//     {
//       files_found.push_back(path);
//     }
//   }
//   else
//   {
//     ROS_ERROR_STREAM("ParameterParser::findFilesInDir() -- path doesn't exist!");
//   }
// }