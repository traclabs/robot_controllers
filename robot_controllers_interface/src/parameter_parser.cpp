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
      if (xmlval.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN_STREAM("ParameterParser::paramUpdatesCallback() -- found ARRAY");
      }
      else if (xmlval.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN_STREAM("ParameterParser::paramUpdatesCallback() -- found ARRAY >> "<<xmlval);
      }
      else if (xmlval.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        std::string type_str = static_cast<std::string>(xmlval);
        std::string compare_str = static_cast<std::string>(param.second);
        if (type_str != compare_str)
        {
          param.second = xmlval;

          // for text edit strings only
          for (auto &dyn : dynamic_strings_)
          {
            ROS_WARN_STREAM("looking for dynamic string param: "<<dyn.first<<" in string: "<<param.first);
            if (param.first.find(dyn.first) != std::string::npos)
            {
              std::string expanded_param_name = param.first + "/" + type_str;
              ROS_INFO_STREAM("ParameterParser::paramUpdatesCallback() -- updating STRING: "
                              << type_str << " under parameter: " << expanded_param_name);
              *dyn.second = type_str;
              ROS_ERROR_STREAM("matched param >> "<<param.first<<" and set to "<<*dyn.second<<" at ptr: "<<dyn.second);
              break;
            }
          }

          // for dropdowns
          for (auto &dyn : dynamic_options_)
          {
            ROS_WARN_STREAM("looking for dynamic option param: "<<dyn.first<<" in string: "<<param.first);
            if (param.first.find(dyn.first) != std::string::npos)
            {
              // std::string expanded_param_name = param.first + "/" + type_str;
              ROS_INFO_STREAM("ParameterParser::paramUpdatesCallback() -- updating STRING: "
                              << type_str << " under parameter: " << param.first);
              *dyn.second.first = type_str;
              ROS_ERROR_STREAM("matched param >> "<<param.first<<" and set to "<<type_str<<" at ptr: "<<dyn.second.first);
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
            ROS_WARN_STREAM("looking for dynamic dbl param: "<<dyn.first<<" in string: "<<param.first);
            if (param.first.find(dyn.first) != std::string::npos)
            {
              *dyn.second = param_val;
              ROS_ERROR_STREAM("matched param >> "<<param.first<<" and set to "<<*dyn.second<<" at ptr: "<<dyn.second);
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
            ROS_WARN_STREAM("looking for dynamic string param: "<<dyn.first<<" in string: "<<param.first);
            if (param.first.find(dyn.first) != std::string::npos)
            {
              *dyn.second = param_val;
              ROS_ERROR_STREAM("matched param >> "<<param.first<<" and set to "<<*dyn.second<<" at ptr: "<<dyn.second);
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
            ROS_WARN_STREAM("looking for dynamic string param: "<<dyn.first<<" in string: "<<param.first);
            if (param.first.find(dyn.first) != std::string::npos)
            {
              *dyn.second = param_val;
              ROS_ERROR_STREAM("matched param >> "<<param.first<<" and set to "<<*dyn.second<<" at ptr: "<<dyn.second);
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


void ParameterParser::setParams(std::string param_name, std::string name_str, std::map<std::string, XmlRpc::XmlRpcValue>& param_map)
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
bool ParameterParser::registerDouble(const std::string param, double* ptr, double min, double max, std::string ui_type)
{
  if (ptr == NULL)
  {
    return false;
  }

  ROS_WARN_STREAM("ParameterParser::registerDouble() -- registering param "<<param<<" of value: "<<*ptr<<" at address "<<ptr);

  // todo -- look to see if there is a slash to start with
  std::string param_base = "params/" + param;
  std::string param_value = param_base + "/value";
  ROS_WARN_STREAM("  **  generated path: "<<param_value);
  ROS_WARN_STREAM("  **  value: "<<*ptr);
  nh_.setParam(param_value, *ptr);

  nh_.setParam((param_base + "/min_val"), min);
  nh_.setParam((param_base + "/max_val"), max);
  nh_.setParam((param_base + "/ui_type"), ui_type);
  nh_.setParam((param_base + "/type"), "double_range");

  dynamic_doubles_[param] = ptr;

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);
  ROS_ERROR_STREAM("putting param base: "<<full_param<<" onto server monitoring value: "<<*ptr<<" at addr:"<< ptr<<" with xml type: "<<TypeString[dynamic_param_vals_[full_param].getType()]);

  return true;
}


// "radio" is default ui_type
bool ParameterParser::registerBool(const std::string param, bool* ptr, std::string ui_type)
{
  if (ptr == NULL)
  {
    return false;
  }

  std::string param_base = "params/" + param;
  std::string param_value = param_base + "/value";
  ROS_WARN_STREAM("  **  generated path: "<<param_value);
  ROS_WARN_STREAM("  **  value: "<<*ptr);
  nh_.setParam(param_value, *ptr);

  nh_.setParam((param_base + "/ui_type"), ui_type);

  dynamic_bools_[param] = ptr;

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);
  ROS_ERROR_STREAM("putting param base: "<<full_param<<" onto server monitoring value: "<<*ptr<<" at addr:"<< ptr<<" with xml type: "<<TypeString[dynamic_param_vals_[full_param].getType()]);

  return true;
}


// "textedit" is default ui_type
bool ParameterParser::registerString(const std::string param, std::string* ptr, std::string ui_type)
{
  if (ptr == NULL)
  {
    return false;
  }

  std::string param_base = "params/" + param;
  std::string param_value = param_base + "/value";
  ROS_WARN_STREAM("  **  generated path: "<<param_value);
  ROS_WARN_STREAM("  **  value: "<<*ptr);
  nh_.setParam(param_value, *ptr);

  nh_.setParam((param_base + "/ui_type"), ui_type);

  dynamic_strings_[param] = ptr;

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);
  ROS_ERROR_STREAM("putting param base: "<<full_param<<" onto server monitoring value: "<<*ptr<<" at addr:"<< ptr<<" with xml type: "<<TypeString[dynamic_param_vals_[full_param].getType()]);

  return true;
}


// "slider" is default ui_type
// 0 is default min
// 1 is default max
bool ParameterParser::registerInt(const std::string param, int* ptr, int min, int max, std::string ui_type)
{
  if (ptr == NULL)
  {
    return false;
  }

  std::string param_base = "params/" + param;
  std::string param_value = param_base + "/value";
  ROS_WARN_STREAM("  **  generated path: "<<param_value);
  ROS_WARN_STREAM("  **  value: "<<*ptr);
  nh_.setParam(param_value, *ptr);

  nh_.setParam((param_base + "/min_val"), min);
  nh_.setParam((param_base + "/max_val"), max);
  nh_.setParam((param_base + "/ui_type"), ui_type);
  nh_.setParam((param_base + "/type"), "int_range");

  dynamic_integers_[param] = ptr;

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);
  ROS_ERROR_STREAM("putting param base: "<<full_param<<" onto server monitoring value: "<<*ptr<<" at addr:"<< ptr<<" with xml type: "<<TypeString[dynamic_param_vals_[full_param].getType()]);

  return true;
}


// ui_type has to be dropdown
// ptr* should have the default selected option
bool ParameterParser::registerEnum(const std::string param, std::string* ptr, std::vector<std::string> options)
{
  if (ptr == NULL)
  {
    return false;
  }

  if (options.empty())
  {
    ROS_WARN_STREAM("ParameterParser::registerEnum() -- empty options list, adding \'"
                    << *ptr << "\' as the only option");
    options.push_back(*ptr);
  }

  std::string param_base = "params/" + param;
  std::string param_value = param_base + "/value";
  ROS_WARN_STREAM("  **  generated path: "<<param_value);
  ROS_WARN_STREAM("  **  value: "<<*ptr);
  for (auto opt : options)
    ROS_WARN_STREAM("    **  have option: "<<opt);

  nh_.setParam((param_base + "/options"), options);
  nh_.setParam(param_value, *ptr);
  nh_.setParam((param_base + "/ui_type"), "dropdown");

  dynamic_options_[param] = std::make_pair(ptr, options);

  std::string full_param = nh_.getNamespace() + "/" + param_value;
  dynamic_param_vals_[full_param] = XmlRpc::XmlRpcValue(*ptr);
  ROS_ERROR_STREAM("putting param base: "<<full_param<<" onto server monitoring value: "<<*ptr<<" at addr:"<< ptr<<" with xml type: "<<TypeString[dynamic_param_vals_[full_param].getType()]);

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
//       ROS_INFO_STREAM("ParameterParser::parseFileParams() -- found config fire at: " << fp);
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