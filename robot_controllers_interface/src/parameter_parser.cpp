#include <robot_controllers_interface/parameter_parser.h>

using namespace robot_controllers;

ParameterParser::ParameterParser(const ros::NodeHandle _nh, const std::string _type) :
  nh_(_nh),
  parsing_type_(_type)
{
  ROS_INFO_STREAM("ParameterParser() -- creating parser for: " << parsing_type_);

  // top_level_type_names_.clear();
}


ParameterParser::~ParameterParser() {}


bool ParameterParser::parseYamlParams(const std::string param_base)
{
  // todo - take "controllers/" out of input string ??
  //

  XmlRpc::XmlRpcValue base_param;
  if (nh_.getParamCached(param_base, base_param))
  {
    if (base_param.valid())
    {
      std::cout<<std::endl;
      std::cout<<std::endl;

      std::string param_name = param_base;

      // todo - put assert in here?
      //   we are assuming that the yaml is going to be an array to start
      //
      if (base_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (auto i = 0; i < base_param.size(); ++i)
        {
          ROS_INFO_STREAM("toplevel param name: "<<param_name);

          if (base_param[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            // ROS_INFO_STREAM("top level STRUCT");
            
            // parse first struct
            for (auto v : base_param[i])
            {
              std::string val_str = static_cast<std::string>(v.first);
              std::string expanded_param_name = param_name + "/" + val_str;
              ROS_INFO_STREAM("  midlevel param name: "<<expanded_param_name);

              if (v.second.getType() == XmlRpc::XmlRpcValue::TypeArray)
              {
                expandParamArray(v.second, expanded_param_name);
              }
              else if (v.second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
              {
                expandParamStruct(v.second, expanded_param_name);
              }
              else
              {
                ROS_WARN_STREAM("  mid level SOMETHING ELSE");
              }
            }
          }
          else if (base_param[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            ROS_WARN_STREAM("top level ARRAY");
          }
          else if (base_param[i].getType() == XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_INFO_STREAM("top level STRING");
            std::string val_str = static_cast<std::string>(base_param[i]);
            top_level_type_names_[param_name].push_back(val_str);
          }
          else
          {
            ROS_WARN_STREAM("top level SOMETHING ELSE -- "<<base_param[i].getType());
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM("ParameterParser::parseYamlParams() -- invalid yaml; top level item: "
                         << param_name << " should be array");
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM("couldn't find params under name: "<<param_base);
  }

  // for (auto tl : top_level_type_names_)
  // {
  //   for (auto sub : tl.second)
  //     ROS_WARN_STREAM("controller type: "<<tl.first<<" has member "<<sub);
  // }

  std::cout<<std::endl;
  std::cout<<std::endl;

  return true;
}


bool ParameterParser::parseFileParams(const std::string rospkg, const std::string file)
{
  pkg_path_ = rospkg;
  filename_ = file;

  ROS_INFO_STREAM("ParameterParser::parseParams() -- parsing parameters from file: "
                  << filename_ << " in rospkg: " << pkg_path_);

  std::string yaml_path = ros::package::getPath(pkg_path_);
  if (yaml_path.empty())
  {
    ROS_ERROR_STREAM("ParameterParser::parseParams() -- couldn't find path to package: " << pkg_path_);
    return false;
  }

  // crawl directories looking for the config file
  std::vector<std::string> file_list;
  boost::filesystem::path curerent_path(yaml_path);
  if (boost::filesystem::exists(curerent_path))
  {
    if (boost::filesystem::is_directory(curerent_path))
    {
      for (boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(curerent_path))
      {
        if (boost::filesystem::is_directory(x.path()))
        {
          findFilesInDir(x.path().string(), file_list);
        }
        else
        {
          file_list.push_back(x.path().string());
        }
      }
    }
  }

  for (auto fp : file_list)
  {
    if (fp.find(filename_) != std::string::npos)
    {
      ROS_INFO_STREAM("ParameterParser::parseParams() -- found config fire at: " << fp);
      file_path_ = fp;
      break;
    }
  }

  return true;
}

// enum  Type {
//   TypeInvalid, TypeBoolean, TypeInt, TypeDouble,
//   TypeString, TypeDateTime, TypeBase64, TypeArray,
//   TypeStruct
// }


void ParameterParser::expandParamStruct(XmlRpc::XmlRpcValue& val, std::string& param_name)
{
  for (auto v : val)
  {
    std::string val_str = static_cast<std::string>(v.first);
    std::string expanded_param_name = param_name + "/" + val_str;

    if (v.second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_INFO_STREAM("    another struct; recursive!");
      return expandParamStruct(v.second, expanded_param_name);
    }
    else if (v.second.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_INFO_STREAM("    array in struct");
      expandParamArray(v.second, expanded_param_name);
    }
    else if (v.second.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      dynamic_strings_[expanded_param_name] = static_cast<std::string>(v.second);
      ROS_INFO_STREAM("      -- added " << dynamic_strings_[expanded_param_name] << " from struct to dynamic strings");
      nh_.setParam(expanded_param_name, dynamic_strings_[expanded_param_name]);
    }
    else if (v.second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      dynamic_doubles_[expanded_param_name] = static_cast<double>(v.second);
      ROS_INFO_STREAM("      -- added " << dynamic_doubles_[expanded_param_name] << " from struct to dynamic doubles");
      nh_.setParam(expanded_param_name, dynamic_doubles_[expanded_param_name]);
    }
    else if (v.second.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      dynamic_ints_[expanded_param_name] = static_cast<int>(v.second);
      ROS_INFO_STREAM("      -- added " << dynamic_ints_[expanded_param_name] << " from struct to dynamic ints");
      nh_.setParam(expanded_param_name, dynamic_ints_[expanded_param_name]);
    }
    else if (v.second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      dynamic_bools_[expanded_param_name] = static_cast<bool>(v.second);
      ROS_INFO_STREAM("      -- added " << dynamic_bools_[expanded_param_name] << " from struct to dynamic bools");
      nh_.setParam(expanded_param_name, dynamic_bools_[expanded_param_name]);
    }
    else
    {
      // todo -- make this into a LUT with strings for types
      ROS_ERROR_STREAM("ParameterParser::expandParamStruct() -- unexpected XmlRpc type: " << v.second.getType());
    }
  }
}


void ParameterParser::expandParamArray(XmlRpc::XmlRpcValue& val, std::string& param_name)
{
  std::vector<int> test_arry;

  for (auto i=0; i < val.size(); ++i)
  {
    ROS_WARN_STREAM("have: "<<val[i]<<" in param: "<<param_name);
    if (val[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN_STREAM("struct in array");
      expandParamStruct(val[i], param_name);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN_STREAM("another array; recursive call!");
      return expandParamArray(val[i], param_name);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string type_str = static_cast<std::string>(val[i]);
      std::string expanded_param_name = param_name + "/" + type_str;
      
      dynamic_strings_[expanded_param_name] = type_str;
      ROS_INFO_STREAM("      -- added " << dynamic_strings_[expanded_param_name] << " from array to dynamic strings");
      nh_.setParam(expanded_param_name, dynamic_strings_[expanded_param_name]);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      // dynamic_doubles_[expanded_param_name] = static_cast<double>(val[i]);
      ROS_INFO_STREAM("      -- (todo) added " << dynamic_doubles_[param_name] << " from array to dynamic doubles");
      // nh_.setParam(expanded_param_name, dynamic_doubles_[expanded_param_name]);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      // dynamic_ints_[expanded_param_name] = static_cast<int>(val[i]);
      test_arry.push_back(static_cast<int>(val[i]));
      ROS_INFO_STREAM("      -- (todo) added " << dynamic_ints_[param_name] << " from array to dynamic ints");
      // nh_.setParam(expanded_param_name, dynamic_ints_[expanded_param_name]);
    }
    else if (val[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      // dynamic_bools_[expanded_param_name] = static_cast<bool>(val[i]);
      ROS_INFO_STREAM("      -- (todo) added " << dynamic_bools_[param_name] << " from array to dynamic bools");
      // nh_.setParam(expanded_param_name, dynamic_bools_[expanded_param_name]);
    }
    else
    {
      // todo -- make this into a LUT with strings for types
      ROS_ERROR_STREAM("ParameterParser::expandParamArray() -- unexpected XmlRpc type: " << val[i].getType());
    }
  }

  // todo !!! make all these options
  // could probably use templated options to "addArrayItem" or maybe overload the functions so we don't have to keep around
  // int array, double array, bool array, etc 
  if (!test_arry.empty())
  {
    nh_.setParam(param_name, test_arry);
    for (auto ta : test_arry)
      ROS_WARN_STREAM("      ** need to add: "<<ta<<" to param options"); 
  }
}


void ParameterParser::findFilesInDir(std::string path, std::vector<std::string>& files_found)
{
  ROS_INFO_STREAM("ParameterParser::findFilesInDir() -- searching directory: " << path << " for files");

  boost::filesystem::path dir_path(path);
  if (boost::filesystem::exists(dir_path))
  {
    if (boost::filesystem::is_directory(dir_path))
    {
      for (boost::filesystem::directory_entry& dir_entry : boost::filesystem::directory_iterator(dir_path))
      {
        // check if the path is the same as our pkg and filename
        if (boost::filesystem::is_directory(dir_entry.path()))
        {
          ROS_INFO_STREAM("ParameterParser::findFilesInDir() -- found new directory to search...");
          return findFilesInDir(dir_entry.path().string(), files_found);
        }
        else
        {
          files_found.push_back(dir_entry.path().string());
        }
      }
    }
    else
    {
      files_found.push_back(path);
    }
  }
  else
  {
    ROS_ERROR_STREAM("ParameterParser::findFilesInDir() -- path doesn't exist!");
  }
}
