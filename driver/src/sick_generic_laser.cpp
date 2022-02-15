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

#if defined LDMRS_SUPPORT && LDMRS_SUPPORT > 0
#include <sick_scan/ldmrs/sick_ldmrs_node.h>
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

static std::shared_ptr<rclcpp::Node> mainNode = NULL;

void setMainNode(std::shared_ptr<rclcpp::Node> _tmpNode)
{
  mainNode = _tmpNode;
}

std::shared_ptr<rclcpp::Node> getMainNode(void)
{
  return (mainNode);
}

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
  double min_range = 0.05;
  double max_range = 100.0;
  node->get_parameter("frame_id", frameId);
  node->get_parameter("hostname", hostName);
  node->get_parameter("scanner_name", scannerName);
  node->get_parameter("port", port);
  node->get_parameter("min_ang", min_ang);
  node->get_parameter("max_ang", max_ang);
  node->get_parameter("min_range", min_range);
  node->get_parameter("max_range", max_range);
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

  if(scannerName == "sick_ldmrs")
  {
#if defined LDMRS_SUPPORT && LDMRS_SUPPORT > 0
    RCLCPP_INFO(node->get_logger(), "Initializing LDMRS...");
    sick_scan::SickLdmrsNode ldmrs;
    int result = ldmrs.init(node, hostName, frameId);
    if(result != sick_scan::ExitSuccess)
    {
      RCLCPP_ERROR(node->get_logger(), "LDMRS initialization failed.");
      return sick_scan::ExitError;
    }
    RCLCPP_INFO(node->get_logger(), "LDMRS initialized.");
    rclcpp::spin(node);
    return sick_scan::ExitSuccess;
#else
    RCLCPP_ERROR(node->get_logger(), "LDMRS not supported. Please build sick_scan2 with option LDMRS_SUPPORT");
    return sick_scan::ExitError;
#endif    
  }

  sick_scan::SickGenericParser *parser = 0;
  try
  {
    parser = new sick_scan::SickGenericParser(scannerName);
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "Scanner \"%s\" not supported, exception \"%s\". Please check your sick_scan2 configuration and launch file.", scannerName.c_str(), e.what());
    return sick_scan::ExitError;
  }
  parser->set_range_min(min_range);
  parser->set_range_max(max_range);

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
        if (result == sick_scan::ExitError || result == sick_scan::ExitFatal)
        {
          RCLCPP_ERROR(node->get_logger(),"init failed: %d. shutting down", result);
          return result;
        }
        else
        {
          runState = scanner_run; // after initialising switch to run state
        }
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
