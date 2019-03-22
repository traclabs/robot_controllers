#include <robot_controllers_interface/parameter_parser.h>

using namespace robot_controllers;

ParameterParser::ParameterParser(const ros::NodeHandle _nh, const std::string _type) :
  nh_(_nh),
  parsing_type_(_type)
{
  ROS_INFO_STREAM("ParameterParser() -- creating parser for: " << parsing_type_);
}


ParameterParser::~ParameterParser() {}

bool ParameterParser::parseParams(const std::string rospkg, const std::string file)
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
