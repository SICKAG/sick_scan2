/**
* \file
* \brief Laser Scanner Entry Point
*
* Copyright (C) 2019,2018,2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2019,2018,2017, SICK AG, Waldkirch
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrück University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*  Last modified: 20th July 2020
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*
*
*  Copyright 2018/2019 SICK AG
*  Copyright 2018/2019 Ing.-Büro Dr. Michael Lehning



*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sick_scan/sick_generic_laser.h"
#include "sick_scan/binScanf.hpp"
#include "sick_scan/binPrintf.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

#define MAX_NAME_LEN (1024)

// 000.001.009 ROS2 specific driver
#define SICK_GENERIC_MAJOR_VER "001"
#define SICK_GENERIC_MINOR_VER "009"
#define SICK_GENERIC_PATCH_LEVEL "000"

#include <algorithm> // for std::min


// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <math.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#define DEG2RAD M_PI / 180.0


/*!
\brief Startup routine - if called with no argmuments we assume debug session.
       Set scanner name variable by parsing for "__name:=". This will be changed in the future
	   by setting a parameter. Calls mainGenericLaser after parsing.

\param argc: Number of Arguments
\param argv: Argument variable
\return exit-code
\sa mainGenericLaser
*/

int main(int argc, char **argv)
{


  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  //node_options.automatically_declare_initial_parameters(true);
  auto node = rclcpp::Node::make_shared("sick_scan2", "", node_options);
  std::vector<std::string> paramList;

  //auto node = rclcpp::Node::make_shared("sick_scan2");

  setMainNode(node);
  std::string paramString;
  //rclcpp::Logger node_logger = node->get_logger();
#if 0
  // Update with setting from yaml param file or command line parameters
  enum PARAM_LIST_ENUM {FRAME_ID_DECLARED,
      IMU_FRAME_ID_DECLARED,
      HOSTNAME_DECLARED,
      SCANNER_NAME_DECLARED,
      PORT_DECLARED,
      MIN_ANG_DECLARED,
      MAX_ANG_DECLARED,
      IMU_ENABLE_DECLARED,
      SKIP_DECLARED,
      USE_SOFTWARE_PLL_DECLARED,
      NUM_DECLARED};
  std::map<int,std::string> parameterNameListMapping;
  parameterNameListMapping[FRAME_ID_DECLARED]= "frame_id";
  parameterNameListMapping[IMU_FRAME_ID_DECLARED]= "imu_frame_id";
  parameterNameListMapping[SCANNER_NAME_DECLARED]= "scanner_name";
  parameterNameListMapping[HOSTNAME_DECLARED]= "hostname";
  parameterNameListMapping[PORT_DECLARED]= "port";
  parameterNameListMapping[MAX_ANG_DECLARED]= "max_ang";
  parameterNameListMapping[MIN_ANG_DECLARED]= "min_ang";
  parameterNameListMapping[IMU_ENABLE_DECLARED]= "imu_enable";
  parameterNameListMapping[SKIP_DECLARED]= "skip";
  parameterNameListMapping[USE_SOFTWARE_PLL_DECLARED]= "use_software_pll";
#endif

  // default Values
  std::string frameId = "world";
  std::string imu_frameId = "imu";
  std::string hostName = "192.168.0.1";
  std::string scanner_name = "undefined";
  int port = 2112;
  double min_ang=-M_PI;
  double max_ang=M_PI;
  double min_range = 0.05;
  double max_range = 100.0;
  bool imu_enable=false;
  bool use_software_pll=true;
  int skip=0;
  bool sw_pll_only_publish=true;
  bool intensity=true;

  // Declare default parameters
  node->declare_parameter<std::string>("frame_id", "world");
  node->declare_parameter<std::string>("imu_frame_id", "imu");
  node->declare_parameter<std::string>("hostname", hostName);
  node->declare_parameter<std::string>("scanner_name", scanner_name);
  node->declare_parameter<int>("port", port);
  node->declare_parameter<double>("min_ang", min_ang);
  node->declare_parameter<double>("max_ang", max_ang);
  node->declare_parameter<double>("min_range", min_range);
  node->declare_parameter<double>("max_range", max_range);
  node->declare_parameter<bool>("imu_enable", imu_enable);
  node->declare_parameter<int>("skip", skip);
  node->declare_parameter<bool>("use_software_pll", use_software_pll);
  node->declare_parameter<bool>("sw_pll_only_publish", sw_pll_only_publish);
  node->declare_parameter<bool>("intensity",intensity);

#if 0
  //handling if params had been defined before
  std::vector<bool> paramListDeclaredFlag;
  paramListDeclaredFlag.resize(NUM_DECLARED);
  for (int i = 0; i < paramListDeclaredFlag.size(); i++)
  {
    std::string paramName=parameterNameListMapping[i];
    paramListDeclaredFlag[i] = node->has_parameter(paramName);// parameter not declared
  }
  for (int i = 0; i < paramListDeclaredFlag.size(); i++)
  {
    if (paramListDeclaredFlag[i] == false) // parameter not declared
    {
      switch(i)
      {
        case FRAME_ID_DECLARED:
          node->declare_parameter<std::string>(parameterNameListMapping[FRAME_ID_DECLARED], frameId);
          break;
        case IMU_FRAME_ID_DECLARED:
          node->declare_parameter<std::string>(parameterNameListMapping[IMU_FRAME_ID_DECLARED], imu_frameId);
          break;
        case SCANNER_NAME_DECLARED:
          node->declare_parameter<std::string>(parameterNameListMapping[SCANNER_NAME_DECLARED], scanner_name);
          break;
        case HOSTNAME_DECLARED:
          node->declare_parameter<std::string>(parameterNameListMapping[HOSTNAME_DECLARED], hostName);
          break;
        case PORT_DECLARED:
          node->declare_parameter<int>(parameterNameListMapping[PORT_DECLARED], port);
          break;
        case MAX_ANG_DECLARED:
          node->declare_parameter<double>(parameterNameListMapping[MAX_ANG_DECLARED], max_ang);
          break;
        case MIN_ANG_DECLARED:
          node->declare_parameter<double>(parameterNameListMapping[MIN_ANG_DECLARED], min_ang);
          break;
        case IMU_ENABLE_DECLARED:
          node->declare_parameter<bool>(parameterNameListMapping[IMU_ENABLE_DECLARED], imu_enable);
          break;
        case SKIP_DECLARED:
          node->declare_parameter<int>(parameterNameListMapping[SKIP_DECLARED], skip);
          break;
        case USE_SOFTWARE_PLL_DECLARED:
          node->declare_parameter<bool>(parameterNameListMapping[USE_SOFTWARE_PLL_DECLARED], use_software_pll);
          break;
      }
    }
  }
#endif
  node->get_parameter("frame_id", frameId);
  node->get_parameter("imu_frame_id", imu_frameId);
  node->get_parameter("hostname", hostName);
  node->get_parameter("scanner_name", scanner_name);
  node->get_parameter("port", port);
  node->get_parameter("min_ang", min_ang);
  node->get_parameter("max_ang", max_ang);
  node->get_parameter("min_range", min_range);
  node->get_parameter("max_range", max_range);
  node->get_parameter("imu_enable", imu_enable);
  node->get_parameter("skip", skip);
  node->get_parameter("use_software_pll", use_software_pll);
  node->get_parameter("sw_pll_only_publish", sw_pll_only_publish);
  node->get_parameter("intensity", intensity);

  // node->get_parameters(paramList);
  char nameId[] = "__name:=";
  char nameVal[MAX_NAME_LEN] = {0};
  char **argv_tmp; // argv_tmp[0][0] argv_tmp[0] identisch ist zu (*argv_tmp)
  int argc_tmp;


  argc_tmp = argc;
  argv_tmp = argv;

  const int MAX_STR_LEN = 1024;
  char nameTagVal[MAX_STR_LEN] = {0};
  char logTagVal[MAX_STR_LEN] = {0};
  char internalDebugTagVal[MAX_STR_LEN] = {0};
  char sensorEmulVal[MAX_STR_LEN] = {0};

  if (argc == 1) // just for testing without calling by roslaunch
  {
    // recommended call for internal debugging as an example: __name:=sick_rms_320 __internalDebug:=1
    // strcpy(nameTagVal, "__name:=sick_rms_3xx");  // sick_rms_320 -> radar
    strcpy(nameTagVal, "__name:=sick_tim_5xx");  // sick_rms_320 -> radar
    strcpy(logTagVal, "__log:=/tmp/tmp.log");
    strcpy(internalDebugTagVal, "__internalDebug:=1");
    // strcpy(sensorEmulVal, "__emulSensor:=1");
    strcpy(sensorEmulVal, "__emulSensor:=0");
    argc_tmp = 5;
    argv_tmp = (char **) malloc(sizeof(char *) * argc_tmp);

    argv_tmp[0] = argv[0];
    argv_tmp[1] = nameTagVal;
    argv_tmp[2] = logTagVal;
    argv_tmp[3] = internalDebugTagVal;
    argv_tmp[4] = sensorEmulVal;

  }
  RCLCPP_INFO(getMainNode()->get_logger(), "sick_generic_caller V. %s.%s.%s", SICK_GENERIC_MAJOR_VER, SICK_GENERIC_MINOR_VER,
              SICK_GENERIC_PATCH_LEVEL);
  for (int i = 0; i < argc_tmp; i++)
  {
    if (strstr(argv_tmp[i], nameId) == argv_tmp[i])
    {
      strcpy(nameVal, argv_tmp[i] + strlen(nameId));
      scanner_name = nameVal;
    }
    if(strncmp("hostname:=", argv_tmp[i], 10) == 0)
    {
      node->set_parameter(rclcpp::Parameter("hostname", argv_tmp[i] + 10));
      node->get_parameter("hostname", hostName);
      RCLCPP_INFO(getMainNode()->get_logger(), "hostname: %s", hostName.c_str());
    }
    if(strncmp("port:=", argv_tmp[i], 6) == 0)
    {
      node->set_parameter(rclcpp::Parameter("port", atoi(argv_tmp[i] + 6)));
      node->get_parameter("port", port);
      RCLCPP_INFO(getMainNode()->get_logger(), "port: %d", port);
    }
    RCLCPP_INFO(getMainNode()->get_logger(), "Program arguments: %s", argv_tmp[i]);
  }
  RCLCPP_INFO(getMainNode()->get_logger(), "sick_generic_caller: scanner_name: %s, hostname: %s, port: %d", scanner_name.c_str(), hostName.c_str(), port);


  int result = mainGenericLaser(argc_tmp, argv_tmp, scanner_name);
  return result;

}
