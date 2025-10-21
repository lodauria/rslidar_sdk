/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace robosense::lidar;

namespace rslidar_sdk
{

class RSLidarSdk : public rclcpp::Node
{
public:
  explicit RSLidarSdk(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rslidar_sdk_node", options)
  {
    RS_TITLE << "********************************************************" << RS_REND;
    RS_TITLE << "**********                                    **********" << RS_REND;
    RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR 
      << "." << RSLIDAR_VERSION_MINOR 
      << "." << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
    RS_TITLE << "**********                                    **********" << RS_REND;
    RS_TITLE << "********************************************************" << RS_REND;

    std::string config_path;
    config_path = (std::string)PROJECT_PATH;
    config_path += "/config/config.yaml";

    std::string path = this->declare_parameter<std::string>("config_path", "");

    if (!path.empty())
    {
      config_path = path;
    }

    YAML::Node config;
    try
    {
      config = YAML::LoadFile(config_path);
      RS_INFO << "--------------------------------------------------------" << RS_REND;
      RS_INFO << "Config loaded from PATH:" << RS_REND;
      RS_INFO << config_path << RS_REND;
      RS_INFO << "--------------------------------------------------------" << RS_REND;
    }
    catch (...)
    {
      RS_ERROR << "The format of config file " << config_path << " is wrong. Please check (e.g. indentation)." << RS_REND;
      return;
    }

    demo_ptr = std::make_shared<NodeManager>();
    demo_ptr->init(config);
    demo_ptr->start();

    RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;
  }

private:
  std::shared_ptr<NodeManager> demo_ptr;
};

}  // namespace rslidar_sdk

RCLCPP_COMPONENTS_REGISTER_NODE(rslidar_sdk::RSLidarSdk)
