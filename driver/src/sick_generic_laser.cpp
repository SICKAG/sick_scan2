/**
* \file
* \brief Laser Scanner Main Handling
* Copyright (C) 2013,     Osnabrück University
* Copyright (C) 2017,2018 Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017,2018 SICK AG, Waldkirch
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
*  Last modified: 23rd Oct 2019
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
*
*/

#ifdef _MSC_VER
#define _WIN32_WINNT 0x0501
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#endif

#ifndef _MSC_VER


#endif

#include <sick_scan/sick_generic_laser.h>
#include <sick_scan/sick_scan_common_tcp.h>

#include <sick_scan/sick_generic_parser.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif
#define _USE_MATH_DEFINES

#include <math.h>
#include "string"
#include <stdio.h>
#include <stdlib.h>

/*!
\brief splitting expressions like <tag>:=<value> into <tag> and <value>
\param [In] tagVal: string expression like <tag>:=<value>
\param [Out] tag: Tag after Parsing
\param [Ozt] val: Value after Parsing
\return Result of matching process (true: matching expression found, false: no match found)
*/

bool getTagVal(std::string tagVal, std::string &tag, std::string &val)
  {
  bool ret = false;
  std::size_t pos;
  pos = tagVal.find(":=");
  tag = "";
  val = "";
  if (pos == std::string::npos)
  {
    ret = false;
  }
  else
  {
    tag = tagVal.substr(0, pos);
    val = tagVal.substr(pos + 2);
    ret = true;
  }
  return (ret);
  }

void
h_sig_sigint( int signum )
{
    std::cout << "Receive signum: " << signum << std::endl;
    rclcpp::shutdown();
}


/*!
\brief Internal Startup routine.
\param argc: Number of Arguments
\param argv: Argument variable
\param nodeName name of the ROS-node
\return exit-code
\sa main
*/
int mainGenericLaser(int argc, char **argv, std::string nodeName)
  {
  std::string tag;
  std::string val;

  auto node = getMainNode();
  bool doInternalDebug = false;
  bool emulSensor = false;
/*
  std::vector<std::string> paramArr;
  paramArr.push_back("hostname");
  paramArr.push_back("port");
  paramArr.push_back("name");
  paramArr.push_back("frame_id");


    std::string scannerName = "sick_tim_5xx";
    std::string hostname = "192.168.0.61";
    std::string port = "2112";
    std::string frame_id="laser";


    for (int i = 0; i < argc; i++)
  {
    std::string s = argv[i];
    if (getTagVal(s, tag, val))
    {
      if (tag.compare("__internalDebug") == 0)
      {
        int debugState = 0;
        sscanf(val.c_str(), "%d", &debugState);
        if (debugState > 0)
        {
          doInternalDebug = true;
        }
      }
      if (tag.compare("__emulSensor") == 0)
      {
        int dummyState = 0;
        sscanf(val.c_str(), "%d", &dummyState);
        if (dummyState > 0)
        {
          emulSensor = true;
        }
      }

      for (int i = 0; i < paramArr.size(); i++)
      {
        std::string callParamName = std::string("__");
        callParamName +=  paramArr[i];
        if (tag.compare(callParamName) == 0)
        {
          node->declare_parameter(paramArr[i], val);
          std::string s = "Set param " + paramArr[i] + " to " + val;
          RCLCPP_INFO(node->get_logger(),s.c_str());

          // hacky - will be changed ...
          if (paramArr[i].compare("hostname") == 0)
          {
            hostname = val;
          }
          if (paramArr[i].compare("port") == 0)
          {
            port = val;
          }
          if (paramArr[i].compare("name") == 0)
          {
            scannerName = val;
          }
          if (paramArr[i].compare("frame_id") == 0)
          {
            frame_id = val;
          }
        }
      }
    }
  }
*/

#if TODO
  if (false == nhPriv.getParam("scanner_type", scannerName))
  {
    ROS_ERROR("cannot find parameter ""scanner_type"" in the param set. Please specify scanner_type.");
    ROS_ERROR("Try to set %s as fallback.\n", nodeName.c_str());
    scannerName = nodeName;
  }
#endif

  if (doInternalDebug)
  {
#ifdef _MSC_VER
    nhPriv.setParam("name", scannerName);
    rossimu_settings(nhPriv);  // just for tiny simulations under Visual C++
#endif
  }

  bool useTCP = true;
/*
  std::map<std::string,std::string> paramTagValMap;

  paramTagValMap["name"] = scannerName;
  paramTagValMap["hostname"] = hostname;
  paramTagValMap["port"] = port;
  paramTagValMap["frame_id"] = frame_id;

  //paramTagValMap["min_ang"] = -M_PI;
  //paramTagValMap["max_ang"] = M_PI;

  for(std::map<std::string,std::string>::iterator iter = paramTagValMap.begin(); iter != paramTagValMap.end(); ++iter)
  {
    std::string paramName =  iter->first;
    std::string dummyVal;
    if (false == node->get_parameter(paramName, dummyVal))
    {
      node->declare_parameter(paramName, iter->second);
    }
  }
*/

  std::string frameId = "world";
  std::string hostName = "192.168.0.1";
  std::string scannerName = "undefined";
  int port = 2112;
  double min_ang=-M_PI;
  double max_ang=M_PI;
  node->get_parameter("frame_id", frameId);
  node->get_parameter("hostname", hostName);
  node->get_parameter("scanner_name", scannerName);
  node->get_parameter("port", port);
  node->get_parameter("min_ang", min_ang);
  node->get_parameter("max_ang", max_ang);
  int timelimit = 5;
#if TODO
  if (nhPriv.getParam("hostname", hostname))
  {
    useTCP = true;
  }

  nhPriv.param<std::string>("port", port, "2112");
  nhPriv.param("timelimit", timelimit, 5);
#endif

  bool subscribe_datagram = false;
  int device_number = 0;
#if TODO
  nhPriv.param("subscribe_datagram", subscribe_datagram, false);
  nhPriv.param("device_number", device_number, 0);
#endif

  signal(SIGINT, h_sig_sigint); // just a workaround - use two ctrl-c :-(


  sick_scan::SickGenericParser *parser = new sick_scan::SickGenericParser(scannerName);


  double param;
  char colaDialectId = 'A'; // A or B (Ascii or Binary)
#if TODO
  if (nhPriv.getParam("range_min", param))
  {
    parser->set_range_min(param);
  }
  if (nhPriv.getParam("range_max", param))
  {
    parser->set_range_max(param);
  }
  if (nhPriv.getParam("time_increment", param))
  {
    parser->set_time_increment(param);
  }
#endif
  /*
   *  Check, if parameter for protocol type is set
   */
  bool use_binary_protocol = true;
#if TODO
  if (true == nhPriv.getParam("emul_sensor", emulSensor))
  {
    ROS_INFO("Found emul_sensor overwriting default settings. Emulation: %s\n", emulSensor ? "True" : "False");
  }
  if (true == nhPriv.getParam("use_binary_protocol", use_binary_protocol))
  {
    ROS_INFO("Found sopas_protocol_type param overwriting default protocol:");
    if (use_binary_protocol == true)
    {
      ROS_INFO("Binary protocol activated");
    }
    else
    {
      if (parser->getCurrentParamPtr()->getNumberOfLayers() > 4)
      {
        nhPriv.setParam("sopas_protocol_type", true);
        use_binary_protocol = true;
        ROS_WARN("This scanner type does not support ASCII communication.\n"
                         "Binary communication has been activated.\n"
                         "The parameter \"sopas_protocol_type\" has been set to \"True\".");
      } else
      {
        ROS_INFO("ASCII protocol activated");
      }
    }
    parser->getCurrentParamPtr()->setUseBinaryProtocol(use_binary_protocol);
  }
#endif

  if (parser->getCurrentParamPtr()->getUseBinaryProtocol())
  {
    colaDialectId = 'B';
  } else
  {
    colaDialectId = 'A';
  }

  sick_scan::SickScanCommonTcp *s = NULL;

  int result = sick_scan::ExitError;

  sick_scan::SickScanConfig cfg;


  enum NodeRunState { scanner_init, scanner_run, scanner_finalize };

  NodeRunState runState = scanner_init;  //
  while (rclcpp::ok())
  {
    switch(runState)
    {
      case scanner_init:
        RCLCPP_INFO(node->get_logger(),"Start initialising scanner ...");
        // attempt to connect/reconnect
        delete s;  // disconnect scanner
        if (useTCP)
        {
          RCLCPP_INFO(node->get_logger(),"hostname: %s", hostName.c_str());
          RCLCPP_INFO(node->get_logger(),"Port    : %d", port);

          s = new sick_scan::SickScanCommonTcp(hostName, port, timelimit, parser, colaDialectId);
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(),"TCP is not switched on. Probably hostname or port not set. Use roslaunch to start node.");
          exit(-1);
        }


        if (emulSensor)
        {
          s->setEmulSensor(true);
        }

        result = s->init();

        runState = scanner_run; // after initialising switch to run state
        break;

      case scanner_run:
        if (result == sick_scan::ExitSuccess) // OK -> loop again
        {
          rclcpp::spin_some(node);
          result = s->loopOnce();
        }
        else
        {
          runState = scanner_finalize; // interrupt
        }
      case scanner_finalize:
        break; // ExitError or similiar -> interrupt while-Loop
      default:
      RCLCPP_ERROR(node->get_logger(),"Invalid run state in main loop");
        break;
    }
  }

  delete s; // close connnect
  delete parser; // close parser
  return result;

  }
