/**
* \file
* \brief Laser Scanner communication main routine
*
* Copyright (C) 2013, Osnabrück University
* Copyright (C) 2017-2019, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017-2019, SICK AG, Waldkirch
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
* Based on the TiM communication example by SICK AG.
*
*/

#ifdef _MSC_VER // compiling simulation for MS-Visual C++ - not defined for linux system
#pragma warning(disable: 4996)
#pragma warning(disable: 4267) // due to conversion warning size_t in the ros header file
#define _WIN32_WINNT 0x0501

#endif

#include <sick_scan/sick_scan_common_nw.h>
#include <sick_scan/sick_scan_common.h>
#include <sick_scan/sick_generic_laser.h>
// #include <sick_scan/sick_generic_radar.h>

#ifdef _MSC_VER
#include "sick_scan/rosconsole_simu.hpp"
#endif

#include "sick_scan/binScanf.hpp"
#include "sick_scan/sick_generic_imu.h"
// if there is a missing RadarScan.h, try to run catkin_make in der workspace-root
// #include <sick_scan/RadarScan.h>

#include <cstdio>
#include <cstring>

#include <math.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#define _USE_MATH_DEFINES

#include <math.h>

#ifndef rad2deg
#define rad2deg(x) ((x) / M_PI * 180.0)
#endif

#define deg2rad_const (0.017453292519943295769236907684886f)

#include "sick_scan/tcp/colaa.hpp"
#include "sick_scan/tcp/colab.hpp"

#include <map>
#include <climits>

/*!
\brief Universal swapping function
\param ptr: Pointer to datablock
\param numBytes : size of variable in bytes
*/
void swap_endian(unsigned char *ptr, int numBytes)
{
  unsigned char *buf = (ptr);
  unsigned char tmpChar;
  for (int i = 0; i < numBytes / 2; i++)
  {
    tmpChar = buf[numBytes - 1 - i];
    buf[numBytes - 1 - i] = buf[i];
    buf[i] = tmpChar;
  }
}

/*!
 * <todo> doku.
\brief Universal swapping function
\param ptr: Pointer to datablock
\param numBytes : size of variable in bytes
*/

std::vector<unsigned char> stringToVector(std::string s)
{
  std::vector<unsigned char> result;
  for (int j = 0; j < s.length(); j++)
  {
    result.push_back(s[j]);
  }
  return result;

}

/*!
\brief return diagnostic error code (small helper function)
     This small helper function was introduced to allow the compiling under Visual c++.
\return Diagnostic error code (value 2)
*/
static int getDiagnosticErrorCode() // workaround due to compiling error under Visual C++
{
  return (2);
#if TODO
#ifdef _MSC_VER
#undef ERROR
  return(2);
#else
  return(diagnostic_msgs::DiagnosticStatus::ERROR);
#endif
#endif
}

/*!
\brief Convert part of unsigned char vector into a std::string
\param replyDummy: Pointer to byte block hold in vector
\param off: Starting Position for copy command
\param len: Number of bytes which should be copied
\return result of copy action as std::string
*/
const std::string binScanfGetStringFromVec(std::vector<unsigned char> *replyDummy, int off, long len)
{
  std::string s;
  s = "";
  for (int i = 0; i < len; i++)
  {
    char ch = (char) ((*replyDummy)[i + off]);
    s += ch;
  }
  return (s);
}

namespace sick_scan
{
  /*!
  \brief calculate crc-code for last byte of binary message
       XOR-calucation is done ONLY over message content (i.e. skipping the first 8 Bytes holding 0x02020202 <Length Information as 4-byte long>)
  \param msgBlock: message content
  \param len: Length of message content in byte
  \return XOR-calucation abount message content (msgBlock[0] ^ msgBlock[1] .... msgBlock[len-1]
  */
  unsigned char sick_crc8(unsigned char *msgBlock, int len)
  {
    unsigned char xorVal = 0x00;
    int off = 0;
    for (int i = off; i < len; i++)
    {

      unsigned char val = msgBlock[i];
      xorVal ^= val;
    }
    return (xorVal);
  }

  /*!
  \brief Converts a SOPAS command to a human readable string
  \param s: ASCII-Sopas command including 0x02 and 0x03
  \return Human readable string 0x02 and 0x02 are converted to "<STX>" and "<ETX>"
  */
  std::string stripControl(std::string s)
  {
    std::string dest;
    for (int i = 0; i < s.length(); i++)
    {

      if (s[i] >= ' ')
      {
        dest += s[i];
      }
      else
      {
        switch (s[i])
        {
          case 0x02:
            dest += "<STX>";
            break;
          case 0x03:
            dest += "<ETX>";
            break;
        }
      }
    }
    return (dest);
  }

  /*!
\brief Converts a SOPAS command to a human readable string
\param s: ASCII-Sopas command including 0x02 and 0x03
\return Human readable string 0x02 and 0x02 are converted to "<STX>" and "<ETX>"
*/
  std::string stripControl(std::vector<unsigned char> s)
  {
    bool isParamBinary = false;
    int spaceCnt = 0x00;
    int cnt0x02 = 0;

    for (size_t i = 0; i < s.size(); i++)
    {
      if (s[i] != 0x02)
      {
        isParamBinary = false;

      }
      else
      {
        cnt0x02++;
      }
      if (i > 4)
      {
        break;
      }
    }
    if (4 == cnt0x02)
    {
      isParamBinary = true;
    }
    std::string dest;
    if (isParamBinary == true)
    {
      int parseState = 0;

      unsigned long lenId = 0x00;
      char szDummy[255] = {0};
      for (size_t i = 0; i < s.size(); i++)
      {
        switch (parseState)
        {
          case 0:
            if (s[i] == 0x02)
            {
              dest += "<STX>";
            }
            else
            {
              dest += "?????";
            }
            if (i == 3)
            {
              parseState = 1;
            }
            break;
          case 1:
            lenId |= s[i] << (8 * (7 - i));
            if (i == 7)
            {
              sprintf(szDummy, "<Len=%04lu>", lenId);
              dest += szDummy;
              parseState = 2;
            }
            break;
          case 2:
          {
            unsigned long dataProcessed = i - 8;
            if (s[i] == ' ')
            {
              spaceCnt++;
            }
            if (spaceCnt == 2)
            {
              parseState = 3;
            }
            dest += s[i];
            if (dataProcessed >= (lenId - 1))
            {
              parseState = 4;
            }

            break;
          }

          case 3:
          {
            char ch = dest[dest.length() - 1];
            if (ch != ' ')
            {
              dest += ' ';
            }
            sprintf(szDummy, "0x%02x", s[i]);
            dest += szDummy;

            unsigned long dataProcessed = i - 8;
            if (dataProcessed >= (lenId - 1))
            {
              parseState = 4;
            }
            break;
          }
          case 4:
          {
            sprintf(szDummy, " CRC:<0x%02x>", s[i]);
            dest += szDummy;
            break;
          }
          default:
            break;
        }
      }
    }
    else
    {
      for (size_t i = 0; i < s.size(); i++)
      {

        if (s[i] >= ' ')
        {
          // <todo> >= 0x80
          dest += s[i];
        }
        else
        {
          switch (s[i])
          {
            case 0x02:
              dest += "<STX>";
              break;
            case 0x03:
              dest += "<ETX>";
              break;
          }
        }
      }
    }

    return (dest);
  }

  /*!
  \brief Construction of SickScanCommon
  \param parser: Corresponding parser holding specific scanner parameter
  */
  SickScanCommon::SickScanCommon(SickGenericParser *parser) :
      parser_(parser)
  // FIXME All Tims have 15Hz
  {

    expectedFrequency_ = this->parser_->getCurrentParamPtr()->getExpectedFrequency();

    setSensorIsRadar(false);
    init_cmdTables();

    // For simulation under MS Visual c++ the update config is switched off
    {
      SickScanConfig cfg;

      auto node = getMainNode();
      node->get_parameter("min_ang", cfg.min_ang);
      node->get_parameter("max_ang", cfg.max_ang);
      node->get_parameter("imu_enable", cfg.imu_enable);
      node->get_parameter("imu_frame_id", cfg.imu_frame_id);
      node->get_parameter("frame_id", cfg.frame_id);
      node->get_parameter("skip", cfg.skip);
      node->get_parameter("use_software_pll", cfg.use_software_pll);
      node->get_parameter("sw_pll_only_publish",cfg.sw_pll_only_publish);
      node->get_parameter("intensity",cfg.intensity);
      update_config(cfg);
    }
#if 0
    // datagram publisher (only for debug)
    ros::NodeHandle pn("~");
    pn.param<bool>("publish_datagram", publish_datagram_, false);
    if (publish_datagram_)
      datagram_pub_ = nh_.advertise<std_msgs::String>("datagram", 1000);


    // Pointcloud2 publisher
    //
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 100);

    // just for debugging, but very helpful for the start
    cloud_radar_rawtarget_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_radar_rawtarget", 100);
    cloud_radar_track_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_radar_track", 100);

    radarScan_pub_ = nh_.advertise<sick_scan::RadarScan>("radar", 100);
    // scan publisher
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1000);
#endif
    pub_ = getMainNode()->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // e.g. https://answers.ros.org/question/312587/generate-and-publish-pointcloud2-in-ros2/
    cloud_pub_ = getMainNode()->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 100);
    imuScan_pub_ = getMainNode()->create_publisher<sensor_msgs::msg::Imu>("imu", 100);
  }

  /*!
  \brief Stops sending scan data
  \return error code
   */
  int SickScanCommon::stop_scanner()
  {
    /*
     * Stop streaming measurements
     */
    const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0"};
    int result = sendSOPASCommand(requestScanData0, NULL);
    if (result != 0)
    {
      // use printf because we cannot use printf from the destructor
      printf("\nSOPAS - Error stopping streaming scan data!\n");
    }
    else
    {
      printf("\nSOPAS - Stopped streaming scan data.\n");
    }

    return result;
  }

  /*!
  \brief Convert little endian to big endian (should be replaced by swap-routine)
  \param *vecArr Pointer to 4 byte block
  \return swapped 4-byte-value as long
  */
  unsigned long SickScanCommon::convertBigEndianCharArrayToUnsignedLong(const unsigned char *vecArr)
  {
    unsigned long val = 0;
    for (int i = 0; i < 4; i++)
    {
      val = val << 8;
      val |= vecArr[i];
    }
    return (val);
  }


  /*!
  \brief Check block for correct framing and starting sequence of a binary message
  \param reply Pointer to datablock
  \return length of message (-1 if message format is not correct)
  */
  int sick_scan::SickScanCommon::checkForBinaryAnswer(const std::vector<unsigned char> *reply)
  {
    int retVal = -1;

    if (reply == NULL)
    {
    }
    else
    {
      if (reply->size() < 8)
      {
        retVal = -1;
      }
      else
      {
        const unsigned char *ptr = &((*reply)[0]);
        unsigned binId = convertBigEndianCharArrayToUnsignedLong(ptr);
        unsigned cmdLen = convertBigEndianCharArrayToUnsignedLong(ptr + 4);
        if (binId == 0x02020202)
        {
          int replyLen = reply->size();
          if (replyLen == 8 + cmdLen + 1)
          {
            retVal = cmdLen;
          }
        }
      }
    }
    return (retVal);

  }


  /*!
  \brief Reboot scanner (todo: this does not work if the scanner is set to binary mode) Fix me!
  \return Result of rebooting attempt
  */
  bool SickScanCommon::rebootScanner()
  {
    /*
     * Set Maintenance access mode to allow reboot to be sent
     */
    std::vector<unsigned char> access_reply;
    // changed from "03" to "3"
    int result = sendSOPASCommand("\x02sMN SetAccessMode 3 F4724744\x03\0", &access_reply);
    if (result != 0)
    {
      printf("SOPAS - Error setting access mode");
      return false;
    }
    std::string access_reply_str = replyToString(access_reply);
    if (access_reply_str != "sAN SetAccessMode 1")
    {
      printf("SOPAS - Error setting access mode, unexpected response : %s", access_reply_str.c_str());
      return false;
    }

    /*
     * Send reboot command
     */
    std::vector<unsigned char> reboot_reply;
    result = sendSOPASCommand("\x02sMN mSCreboot\x03\0", &reboot_reply);
    if (result != 0)
    {
      printf("SOPAS - Error rebooting scanner");
      return false;
    }
    std::string reboot_reply_str = replyToString(reboot_reply);
    if (reboot_reply_str != "sAN mSCreboot")
    {
      printf("SOPAS - Error rebooting scanner, unexpected response : %s ", reboot_reply_str.c_str());
      return false;
    }

    printf("SOPAS - Rebooted scanner");

    // Wait a few seconds after rebooting
    // ros::Duration(15.0).sleep();

    return true;
  }


  /*!
  \brief Destructor of SickScanCommon
  */
  SickScanCommon::~SickScanCommon()
  {
//		delete diagnosticPub_;

    printf("sick_scan driver exiting.\n");
  }


  /*!
  \brief Generate expected answer string from the command string
  \param requestStr command string (either as ASCII or BINARY)
  \return expected answer string
   */
  std::string SickScanCommon::generateExpectedAnswerString(const std::vector<unsigned char> requestStr)
  {
    std::string expectedAnswer = "";
    int i = 0;
    char cntWhiteCharacter = 0;
    int initalTokenCnt = 2; // number of initial token to identify command
    std::map<std::string, int> specialTokenLen;
    char firstToken[1024] = {0};
    specialTokenLen["sRI"] = 1; // for SRi-Command only the first token identify the command
    std::string tmpStr = "";
    int cnt0x02 = 0;
    bool isBinary = false;
    for (int i = 0; i < 4; i++)
    {
      if (i < requestStr.size())
      {
        if (requestStr[i] == 0x02)
        {
          cnt0x02++;
        }

      }
    }

    int iStop = requestStr.size();  // copy string until end of string
    if (cnt0x02 == 4)
    {

      int cmdLen = 0;
      for (int i = 0; i < 4; i++)
      {
        cmdLen |= cmdLen << 8;
        cmdLen |= requestStr[i + 4];
      }
      iStop = cmdLen + 8;
      isBinary = true;

    }
    int iStart = (isBinary == true) ? 8 : 0;
    for (int i = iStart; i < iStop; i++)
    {
      tmpStr += (char) requestStr[i];
    }
    if (isBinary)
    {
      tmpStr = "\x2" + tmpStr;
    }
    if (sscanf(tmpStr.c_str(), "\x2%s", firstToken) == 1)
    {
      if (specialTokenLen.find(firstToken) != specialTokenLen.end())
      {
        initalTokenCnt = specialTokenLen[firstToken];

      }
    }

    for (int i = iStart; i < iStop; i++)
    {
      if ((requestStr[i] == ' ') || (requestStr[i] == '\x3'))
      {
        cntWhiteCharacter++;
      }
      if (cntWhiteCharacter >= initalTokenCnt)
      {
        break;
      }
      if (requestStr[i] == '\x2')
      {
      }
      else
      {
        expectedAnswer += requestStr[i];
      }
    }

    /*!
     * Map that defines expected answer identifiers
     */
    std::map<std::string, std::string> keyWordMap;
    keyWordMap["sWN"] = "sWA";
    keyWordMap["sRN"] = "sRA";
    keyWordMap["sRI"] = "sRA";
    keyWordMap["sMN"] = "sAN";
    keyWordMap["sEN"] = "sEA";

    for (std::map<std::string, std::string>::iterator it = keyWordMap.begin(); it != keyWordMap.end(); it++)
    {

      std::string keyWord = it->first;
      std::string newKeyWord = it->second;

      size_t pos = expectedAnswer.find(keyWord);
      if (pos == std::string::npos)
      {

      }
      else
      {
        if (pos == 0)  // must be 0, if keyword has been found
        {
          expectedAnswer.replace(pos, keyWord.length(), newKeyWord);
        }
        else
        {
          printf("Unexpected position of key identifier.\n");
        }
      }

    }
    return (expectedAnswer);

  }

  /*!
  \brief send command and check answer
  \param requestStr: Sopas-Command
  \param *reply: Antwort-String
  \param cmdId: Command index to derive the correct error message (optional)
  \return error code
  */
  int SickScanCommon::sendSopasAndCheckAnswer(std::string requestStr, std::vector<unsigned char> *reply, int cmdId = -1)
  {
    std::vector<unsigned char> requestStringVec;
    for (int i = 0; i < requestStr.length(); i++)
    {
      requestStringVec.push_back(requestStr[i]);
    }
    int retCode = sendSopasAndCheckAnswer(requestStringVec, reply, cmdId);
    return (retCode);
  }

  /*!
  \brief send command and check answer
  \param requestStr: Sopas-Command given as byte-vector
  \param *reply: Antwort-String
  \param cmdId: Command index to derive the correct error message (optional)
  \return error code
  */
  int SickScanCommon::sendSopasAndCheckAnswer(std::vector<unsigned char> requestStr, std::vector<unsigned char> *reply,
                                              int cmdId = -1)
  {

    auto node = getMainNode();

    std::string cmdStr = "";
    int cmdLen = 0;
    for (int i = 0; i < requestStr.size(); i++)
    {
      cmdLen++;
      cmdStr += (char) requestStr[i];
    }
    int result = -1;

    std::string errString;
    if (cmdId == -1)
    {
      errString = "Error unexpected Sopas Answer for request " + stripControl(cmdStr);
    }
    else
    {
      errString = this->sopasCmdErrMsg[cmdId];
    }

    std::string expectedAnswer = generateExpectedAnswerString(requestStr);

    // send sopas cmd

    std::string reqStr = replyToString(requestStr);

    RCLCPP_INFO(node->get_logger(), "Sending  : %s", stripControl(reqStr).c_str());
    result = sendSOPASCommand(cmdStr.c_str(), reply, cmdLen);
    std::string replyStr = replyToString(*reply);
    replyStr = "<STX>" + replyStr + "<ETX>";
    RCLCPP_INFO(node->get_logger(), "Receiving: %s", stripControl(replyStr).c_str());

    if (result != 0)
    {
      std::string tmpStr = "SOPAS Communication -" + errString;
      RCLCPP_INFO(node->get_logger(), "%s\n", tmpStr.c_str());
      // diagnostics_.broadcast(getDiagnosticErrorCode(), tmpStr);
    }
    else
    {
      std::string answerStr = replyToString(*reply);
      std::string searchPattern = generateExpectedAnswerString(requestStr);

      if (answerStr.find(searchPattern) != std::string::npos)
      {
        result = 0;
      }
      else
      {
        std::string tmpMsg = "Error Sopas answer mismatch " + errString + "Answer= >>>" + answerStr + "<<<";
        RCLCPP_INFO(node->get_logger(), "%s\n", tmpMsg.c_str());
        // diagnostics_.broadcast(getDiagnosticErrorCode(), tmpMsg);
        result = -1;
      }
    }
    return result;

  }

  /*!
  \brief set timeout in milliseconds
  \param timeOutInMs in milliseconds
  \sa getReadTimeOutInMs
  */
  void SickScanCommon::setReadTimeOutInMs(int timeOutInMs)
  {
    readTimeOutInMs = timeOutInMs;
  }

  /*!
  \brief get timeout in milliseconds
  \return timeout in milliseconds
  \sa setReadTimeOutInMs
  */
  int SickScanCommon::getReadTimeOutInMs()
  {
    return (readTimeOutInMs);
  }

  /*!
  \brief get protocol type as enum
  \return enum type of type SopasProtocol
  \sa setProtocolType
  */
  int SickScanCommon::getProtocolType(void)
  {
    return m_protocolId;
  }

  /*!
  \brief set protocol type as enum
  \sa getProtocolType
  */
  void SickScanCommon::setProtocolType(SopasProtocol cola_dialect_id)
  {
    m_protocolId = cola_dialect_id;
  }


  /*!
  \brief init routine of scanner
  \return exit code
  */
  int SickScanCommon::init()
  {
    int result = init_device();
    if (result != 0)
    {
      printf("Failed to init device: %d", result);
      return result;
    }
    result = init_scanner();
    if (result != 0)
    {
      printf("Failed to init scanner Error Code: %d\nWaiting for timeout...\n"
             "If the communication mode set in the scanner memory is different from that used by the driver, the scanner's communication mode is changed.\n"
             "This requires a restart of the TCP-IP connection, which can extend the start time by up to 30 seconds. There are two ways to prevent this:\n"
             "1. [Recommended] Set the communication mode with the SOPAS ET software to binary and save this setting in the scanner's EEPROM.\n"
             "2. Use the parameter \"use_binary_protocol\" to overwrite the default settings of the driver.\n", result);
    }
    return result;
  }


  /*!
  \brief init command tables and define startup sequence
  \return exit code
  */
  int SickScanCommon::init_cmdTables()
  {
    sopasCmdVec.resize(SickScanCommon::CMD_END);
    sopasCmdMaskVec.resize(
        SickScanCommon::CMD_END);  // you for cmd with variable content. sprintf should print into corresponding sopasCmdVec
    sopasCmdErrMsg.resize(
        SickScanCommon::CMD_END);  // you for cmd with variable content. sprintf should print into corresponding sopasCmdVec
    sopasReplyVec.resize(SickScanCommon::CMD_END);
    sopasReplyBinVec.resize(SickScanCommon::CMD_END);
    sopasReplyStrVec.resize(SickScanCommon::CMD_END);

    std::string unknownStr = "Command or Error message not defined";
    for (int i = 0; i < SickScanCommon::CMD_END; i++)
    {
      sopasCmdVec[i] = unknownStr;
      sopasCmdMaskVec[i] = unknownStr;  // for cmd with variable content. sprintf should print into corresponding sopasCmdVec
      sopasCmdErrMsg[i] = unknownStr;
      sopasReplyVec[i] = unknownStr;
      sopasReplyStrVec[i] = unknownStr;
    }

    sopasCmdVec[CMD_DEVICE_IDENT_LEGACY] = "\x02sRI 0\x03\0";
    sopasCmdVec[CMD_DEVICE_IDENT] = "\x02sRN DeviceIdent\x03\0";
    sopasCmdVec[CMD_REBOOT] = "\x02sMN mSCreboot\x03";
    sopasCmdVec[CMD_WRITE_EEPROM] = "\x02sMN mEEwriteall\x03";
    sopasCmdVec[CMD_SERIAL_NUMBER] = "\x02sRN SerialNumber\x03\0";
    sopasCmdVec[CMD_FIRMWARE_VERSION] = "\x02sRN FirmwareVersion\x03\0";
    sopasCmdVec[CMD_DEVICE_STATE] = "\x02sRN SCdevicestate\x03\0";
    sopasCmdVec[CMD_OPERATION_HOURS] = "\x02sRN ODoprh\x03\0";
    sopasCmdVec[CMD_POWER_ON_COUNT] = "\x02sRN ODpwrc\x03\0";
    sopasCmdVec[CMD_LOCATION_NAME] = "\x02sRN LocationName\x03\0";
    sopasCmdVec[CMD_ACTIVATE_STANDBY] = "\x02sMN LMCstandby\x03";
    sopasCmdVec[CMD_SET_ACCESS_MODE_3] = "\x02sMN SetAccessMode 3 F4724744\x03\0";
    sopasCmdVec[CMD_SET_ACCESS_MODE_3_SAFETY_SCANNER] = "\x02sMN SetAccessMode 3 6FD62C05\x03\0";
    sopasCmdVec[CMD_GET_OUTPUT_RANGES] = "\x02sRN LMPoutputRange\x03";
    sopasCmdVec[CMD_RUN] = "\x02sMN Run\x03\0";
    sopasCmdVec[CMD_STOP_SCANDATA] = "\x02sEN LMDscandata 0\x03";
    sopasCmdVec[CMD_START_SCANDATA] = "\x02sEN LMDscandata 1\x03";
    sopasCmdVec[CMD_START_RADARDATA] = "\x02sEN LMDradardata 1\x03";
    sopasCmdVec[CMD_ACTIVATE_NTP_CLIENT] = "\x02sWN TSCRole 1\x03";
    sopasCmdVec[CMD_SET_NTP_INTERFACE_ETH] = "\x02sWN TSCTCInterface 0\x03";

    /*
     * Radar specific commands
     */
    sopasCmdVec[CMD_SET_TRANSMIT_RAWTARGETS_ON] = "\x02sWN TransmitTargets 1\x03";  // transmit raw target for radar
    sopasCmdVec[CMD_SET_TRANSMIT_RAWTARGETS_OFF] = "\x02sWN TransmitTargets 0\x03";  // do not transmit raw target for radar
    sopasCmdVec[CMD_SET_TRANSMIT_OBJECTS_ON] = "\x02sWN TransmitObjects 1\x03";  // transmit objects from radar tracking
    sopasCmdVec[CMD_SET_TRANSMIT_OBJECTS_OFF] = "\x02sWN TransmitObjects 0\x03";  // do not transmit objects from radar tracking
    sopasCmdVec[CMD_SET_TRACKING_MODE_0] = "\x02sWN TCTrackingMode 0\x03";  // set object tracking mode to BASIC
    sopasCmdVec[CMD_SET_TRACKING_MODE_1] = "\x02sWN TCTrackingMode 1\x03";  // set object tracking mode to TRAFFIC


    sopasCmdVec[CMD_LOAD_APPLICATION_DEFAULT] = "\x02sMN mSCloadappdef\x03";  // load application default
    sopasCmdVec[CMD_DEVICE_TYPE] = "\x02sRN DItype\x03";  // ask for radar device type
    sopasCmdVec[CMD_ORDER_NUMBER] = "\x02sRN OrdNum\x03";  // ask for radar order number



    sopasCmdVec[CMD_START_MEASUREMENT] = "\x02sMN LMCstartmeas\x03";
    sopasCmdVec[CMD_STOP_MEASUREMENT] = "\x02sMN LMCstopmeas\x03";
    sopasCmdVec[CMD_APPLICATION_MODE_FIELD_ON] = "\x02sWN SetActiveApplications 1 FEVL 1\x03"; // <STX>sWN{SPC}SetActiveApplications{SPC}1{SPC}FEVL{SPC}1<ETX>
    sopasCmdVec[CMD_APPLICATION_MODE_FIELD_OFF] = "\x02sWN SetActiveApplications 1 FEVL 0\x03"; // <STX>sWN{SPC}SetActiveApplications{SPC}1{SPC}FEVL{SPC}1<ETX>
    sopasCmdVec[CMD_APPLICATION_MODE_RANGING_ON] = "\x02sWN SetActiveApplications 1 RANG 1\x03";
    sopasCmdVec[CMD_SET_TO_COLA_A_PROTOCOL] = "\x02sWN EIHstCola 0\x03";
    sopasCmdVec[CMD_GET_PARTIAL_SCANDATA_CFG] = "\x02sRN LMDscandatacfg\x03";//<STX>sMN{SPC}mLMPsetscancfg{SPC } +5000{SPC}+1{SPC}+5000{SPC}-450000{SPC}+2250000<ETX>
    sopasCmdVec[CMD_GET_PARTIAL_SCAN_CFG] = "\x02sRN LMPscancfg\x03";
    sopasCmdVec[CMD_SET_TO_COLA_B_PROTOCOL] = "\x02sWN EIHstCola 1\x03";

    sopasCmdVec[CMD_STOP_IMU_DATA] = "\x02sEN InertialMeasurementUnit 0\x03";
    sopasCmdVec[CMD_START_IMU_DATA] = "\x02sEN InertialMeasurementUnit 1\x03";
    sopasCmdVec[CMD_SET_ENCODER_MODE_NO] = "\x02sWN LICencset 0\x03";
    sopasCmdVec[CMD_SET_ENCODER_MODE_SI] = "\x02sWN LICencset 1\x03";
    sopasCmdVec[CMD_SET_ENCODER_MODE_DP] = "\x02sWN LICencset 2\x03";
    sopasCmdVec[CMD_SET_ENCODER_MODE_DL] = "\x02sWN LICencset 3\x03";
    sopasCmdVec[CMD_SET_INCREMNTSOURCE_ENC] = "\x02sWN LICsrc 1\x03";

    // defining cmd mask for cmds with variable input
    sopasCmdMaskVec[CMD_SET_PARTIAL_SCAN_CFG] = "\x02sMN mLMPsetscancfg %d 1 %d 0 0\x03";//scanfreq [1/100 Hz],angres [1/10000°],
    sopasCmdMaskVec[CMD_SET_PARTICLE_FILTER] = "\x02sWN LFPparticle %d %d\x03";
    sopasCmdMaskVec[CMD_SET_MEAN_FILTER] = "\x02sWN LFPmeanfilter %d +%d 1\x03";
    sopasCmdMaskVec[CMD_ALIGNMENT_MODE] = "\x02sWN MMAlignmentMode %d\x03";
    sopasCmdMaskVec[CMD_APPLICATION_MODE] = "\x02sWN SetActiveApplications 1 %s %d\x03";
    sopasCmdMaskVec[CMD_SET_OUTPUT_RANGES] = "\x02sWN LMPoutputRange 1 %X %X %X\x03";
    sopasCmdMaskVec[CMD_SET_PARTIAL_SCANDATA_CFG] = "\x02sWN LMDscandatacfg %02d 00 %d %d 0 00 00 0 0 0 1 1\x03";
    sopasCmdMaskVec[CMD_SET_ECHO_FILTER] = "\x02sWN FREchoFilter %d\x03";
    sopasCmdMaskVec[CMD_SET_NTP_UPDATETIME] = "\x02sWN TSCTCupdatetime %d\x03";
    sopasCmdMaskVec[CMD_SET_NTP_TIMEZONE] = "sWN TSCTCtimezone %d";
    sopasCmdMaskVec[CMD_SET_IP_ADDR] = "\x02sWN EIIpAddr %02X %02X %02X %02X\x03";
    sopasCmdMaskVec[CMD_SET_NTP_SERVER_IP_ADDR] = "\x02sWN TSCTCSrvAddr %02X %02X %02X %02X\x03";
    sopasCmdMaskVec[CMD_SET_GATEWAY] = "\x02sWN EIgate %02X %02X %02X %02X\x03";
    //error Messages
    sopasCmdErrMsg[CMD_DEVICE_IDENT_LEGACY] = "Error reading device ident";
    sopasCmdErrMsg[CMD_DEVICE_IDENT] = "Error reading device ident for MRS-family";
    sopasCmdErrMsg[CMD_SERIAL_NUMBER] = "Error reading SerialNumber";
    sopasCmdErrMsg[CMD_FIRMWARE_VERSION] = "Error reading FirmwareVersion";
    sopasCmdErrMsg[CMD_DEVICE_STATE] = "Error reading SCdevicestate";
    sopasCmdErrMsg[CMD_OPERATION_HOURS] = "Error reading operation hours";
    sopasCmdErrMsg[CMD_POWER_ON_COUNT] = "Error reading operation power on counter";
    sopasCmdErrMsg[CMD_LOCATION_NAME] = "Error reading Locationname";
    sopasCmdErrMsg[CMD_ACTIVATE_STANDBY] = "Error acticvating Standby";
    sopasCmdErrMsg[CMD_SET_PARTICLE_FILTER] = "Error setting Particelefilter";
    sopasCmdErrMsg[CMD_SET_MEAN_FILTER] = "Error setting Meanfilter";
    sopasCmdErrMsg[CMD_ALIGNMENT_MODE] = "Error setting Alignmentmode";
    sopasCmdErrMsg[CMD_APPLICATION_MODE] = "Error setting Meanfilter";
    sopasCmdErrMsg[CMD_SET_ACCESS_MODE_3] = "Error Access Mode";
    sopasCmdErrMsg[CMD_SET_ACCESS_MODE_3_SAFETY_SCANNER] = "Error Access Mode";
    sopasCmdErrMsg[CMD_SET_OUTPUT_RANGES] = "Error setting angular ranges";
    sopasCmdErrMsg[CMD_GET_OUTPUT_RANGES] = "Error reading angle range";
    sopasCmdErrMsg[CMD_RUN] = "FATAL ERROR unable to start RUN mode!";
    sopasCmdErrMsg[CMD_SET_PARTIAL_SCANDATA_CFG] = "Error setting Scandataconfig";
    sopasCmdErrMsg[CMD_STOP_SCANDATA] = "Error stopping scandata output";
    sopasCmdErrMsg[CMD_START_SCANDATA] = "Error starting Scandata output";
    sopasCmdErrMsg[CMD_SET_IP_ADDR] = "Error setting IP address";
    sopasCmdErrMsg[CMD_SET_GATEWAY] = "Error setting gateway";
    sopasCmdErrMsg[CMD_REBOOT] = "Error rebooting the device";
    sopasCmdErrMsg[CMD_WRITE_EEPROM] = "Error writing data to EEPRom";
    sopasCmdErrMsg[CMD_ACTIVATE_NTP_CLIENT] = "Error activating NTP client";
    sopasCmdErrMsg[CMD_SET_NTP_INTERFACE_ETH] = "Error setting NTP interface to ETH";
    sopasCmdErrMsg[CMD_SET_NTP_SERVER_IP_ADDR] = "Error setting NTP server Adress";
    sopasCmdErrMsg[CMD_SET_NTP_UPDATETIME] = "Error setting NTP update time";
    sopasCmdErrMsg[CMD_SET_NTP_TIMEZONE] = "Error setting NTP timezone";
    sopasCmdErrMsg[CMD_SET_ENCODER_MODE] = "Error activating encoder in single incremnt mode";
    sopasCmdErrMsg[CMD_SET_INCREMNTSOURCE_ENC] = "Error seting encoder increment source to Encoder";

    // ML: Add hier more useful cmd and mask entries

    // After definition of command, we specify the command sequence for scanner initalisation
    ;
    if (parser_->getCurrentParamPtr()->getUseSafetyPasWD())
    {
      sopasCmdChain.push_back(CMD_SET_ACCESS_MODE_3_SAFETY_SCANNER);
    }
    else
    {
      sopasCmdChain.push_back(CMD_SET_ACCESS_MODE_3);
    }

    if (parser_->getCurrentParamPtr()->getUseBinaryProtocol())
    {
      sopasCmdChain.push_back(CMD_SET_TO_COLA_B_PROTOCOL);
    }
    else
    {
      //for binary Mode Testing
      sopasCmdChain.push_back(CMD_SET_TO_COLA_A_PROTOCOL);
    }
    bool tryToStopMeasurement = true;
    if (parser_->getCurrentParamPtr()->getNumberOfLayers() == 1)
    {
      tryToStopMeasurement = false;
      // do not stop measurement for TiM571 otherwise the scanner would not start after start measurement
      // do not change the application - not supported for TiM5xx
    }
    if (parser_->getCurrentParamPtr()->getDeviceIsRadar() == true)
    {
      sopasCmdChain.push_back(CMD_LOAD_APPLICATION_DEFAULT); // load application default for radar

      tryToStopMeasurement = false;
      // do not stop measurement for RMS320 - the RMS320 does not support the stop command
    }
    if (tryToStopMeasurement)
    {
      sopasCmdChain.push_back(CMD_STOP_MEASUREMENT);
      int numberOfLayers = parser_->getCurrentParamPtr()->getNumberOfLayers();

      switch (numberOfLayers)
      {
        case 4:
          sopasCmdChain.push_back(CMD_APPLICATION_MODE_FIELD_OFF);
          sopasCmdChain.push_back(CMD_APPLICATION_MODE_RANGING_ON);
          sopasCmdChain.push_back(CMD_DEVICE_IDENT);
          sopasCmdChain.push_back(CMD_SERIAL_NUMBER);

          break;
        case 24:
          // just measuring - Application setting not supported
          // "Old" device ident command "SRi 0" not supported
          sopasCmdChain.push_back(CMD_DEVICE_IDENT);
          break;

        default:
          sopasCmdChain.push_back(CMD_APPLICATION_MODE_FIELD_OFF);
          sopasCmdChain.push_back(CMD_APPLICATION_MODE_RANGING_ON);
          sopasCmdChain.push_back(CMD_DEVICE_IDENT_LEGACY);

          sopasCmdChain.push_back(CMD_SERIAL_NUMBER);
          break;
      }

    }
    sopasCmdChain.push_back(CMD_FIRMWARE_VERSION);  // read firmware
    sopasCmdChain.push_back(CMD_DEVICE_STATE); // read device state
    sopasCmdChain.push_back(CMD_OPERATION_HOURS); // read operation hours
    sopasCmdChain.push_back(CMD_POWER_ON_COUNT); // read power on count
    sopasCmdChain.push_back(CMD_LOCATION_NAME); // read location name
    return (0);

  }


  /*!
  \brief initialize scanner
  \return exit code
  */
  int SickScanCommon::init_scanner()
  {

    auto node = getMainNode();

    const int MAX_STR_LEN = 1024;

    int maxNumberOfEchos = 1;


    maxNumberOfEchos = this->parser_->getCurrentParamPtr()->getNumberOfMaximumEchos();  // 1 for TIM 571, 3 for MRS1104, 5 for 6000

    bool rssiFlag = true;
    bool rssiResolutionIs16Bit = true; //True=16 bit Flase=8bit
    int activeEchos = 0;
#if TODO
    ros::NodeHandle pn("~");
    pn.getParam("intensity", rssiFlag);
    pn.getParam("intensity_resolution_16bit", rssiResolutionIs16Bit);
    this->parser_->getCurrentParamPtr()->setIntensityResolutionIs16Bit(rssiResolutionIs16Bit);

    // parse active_echos entry and set flag array
    pn.getParam("active_echos", activeEchos);
#endif
    RCLCPP_INFO(node->get_logger(), "Parameter setting for <active_echo: %d>", activeEchos);
    std::vector<bool> outputChannelFlag;
    outputChannelFlag.resize(maxNumberOfEchos);
    int i;
    int numOfFlags = 0;
    for (i = 0; i < outputChannelFlag.size(); i++)
    {
      /*
      After consultation with the company SICK,
      all flags are set to true because the firmware currently does not support single selection of targets.
      The selection of the echoes takes place via FREchoFilter.
       */
      /* former implementation
      if (activeEchos & (1 << i))
      {
        outputChannelFlag[i] = true;
        numOfFlags++;
      }
      else
      {
        outputChannelFlag[i] = false;
      }
       */
      outputChannelFlag[i] = true; // always true (see comment above)
      numOfFlags++;
    }

    if (numOfFlags == 0) // Fallback-Solution
    {
      outputChannelFlag[0] = true;
      numOfFlags = 1;
      RCLCPP_INFO(node->get_logger(), "Activate at least one echo.");
    }

    int result;
    //================== DEFINE ANGULAR SETTING ==========================
    int angleRes10000th = 0;
    int angleStart10000th = 0;
    int angleEnd10000th = 0;


    // Mainloop for initial SOPAS cmd-Handling
    //
    // To add new commands do the followings:
    // 1. Define new enum-entry in the enumumation "enum SOPAS_CMD" in the file sick_scan_common.h
    // 2. Define new command sequence in the member function init_cmdTables()
    // 3. Add cmd-id in the sopasCmdChain to trigger sending of command.
    // 4. Add handling of reply by using for the pattern "REPLY_HANDLING" in this code and adding a new case instruction.
    // That's all!


    volatile bool useBinaryCmd = false;
    if (this->parser_->getCurrentParamPtr()->getUseBinaryProtocol()) // hard coded for every scanner type
    {
      useBinaryCmd = true;  // shall we talk ascii or binary with the scanner type??
    }

    bool useBinaryCmdNow = false;
    int maxCmdLoop = 2; // try binary and ascii during startup

    const int shortTimeOutInMs = 5000; // during startup phase to check binary or ascii
    const int defaultTimeOutInMs = 20000; // standard time out 20 sec.

    setReadTimeOutInMs(shortTimeOutInMs);

    bool restartDueToProcolChange = false;


    for (int i = 0; i < this->sopasCmdChain.size(); i++)
    {
      // ros::Duration(0.2).sleep();   // could maybe removed

      int cmdId = sopasCmdChain[i]; // get next command
      std::string sopasCmd = sopasCmdVec[cmdId];
      std::vector<unsigned char> replyDummy;
      std::vector<unsigned char> reqBinary;
      RCLCPP_INFO(node->get_logger(), "Command: %s", stripControl(sopasCmd).c_str());

      // switch to either binary or ascii after switching the command mode
      // via ... command
      for (int iLoop = 0; iLoop < maxCmdLoop; iLoop++)
      {
        if (iLoop == 0)
        {
          useBinaryCmdNow = useBinaryCmd; // start with expected value
        }
        else
        {
          useBinaryCmdNow = !useBinaryCmdNow;// try the other option
          useBinaryCmd = useBinaryCmdNow; // and use it from now on as default

        }

        this->setProtocolType(useBinaryCmdNow ? CoLa_B : CoLa_A);

        if (useBinaryCmdNow)
        {
          this->convertAscii2BinaryCmd(sopasCmd.c_str(), &reqBinary);
          result = sendSopasAndCheckAnswer(reqBinary, &replyDummy);
          sopasReplyBinVec[cmdId] = replyDummy;
        }
        else
        {
          result = sendSopasAndCheckAnswer(sopasCmd.c_str(), &replyDummy);
        }
        if (result == 0) // command sent successfully
        {
          // useBinaryCmd holds information about last successful command mode
          break;
        }
      }
      if (result != 0)
      {
        RCLCPP_INFO(node->get_logger(), "%s", sopasCmdErrMsg[cmdId].c_str());
        // diagnostics_.broadcast(getDiagnosticErrorCode(), sopasCmdErrMsg[cmdId]);
      }
      else
      {
        sopasReplyStrVec[cmdId] = replyToString(replyDummy);
      }

      //============================================
      // Handle reply of specific commands here by inserting new case instruction
      // REPLY_HANDLING
      //============================================
      maxCmdLoop = 1;


      switch (cmdId)
      {
        case CMD_SET_TO_COLA_A_PROTOCOL:
        {
          bool protocolCheck = checkForProtocolChangeAndMaybeReconnect(useBinaryCmdNow);
          if (false == protocolCheck)
          {
            restartDueToProcolChange = true;
          }
          useBinaryCmd = useBinaryCmdNow;
          setReadTimeOutInMs(defaultTimeOutInMs);
        }
          break;
        case CMD_SET_TO_COLA_B_PROTOCOL:
        {
          bool protocolCheck = checkForProtocolChangeAndMaybeReconnect(useBinaryCmdNow);
          if (false == protocolCheck)
          {
            restartDueToProcolChange = true;
          }
          useBinaryCmd = useBinaryCmdNow;
          setReadTimeOutInMs(defaultTimeOutInMs);
        }
          break;

          /*
          SERIAL_NUMBER: Device ident must be read before!
          */

        case CMD_DEVICE_IDENT: // FOR MRS6xxx the Device Ident holds all specific information (used instead of CMD_SERIAL_NUMBER)
        {
          int cmdLen = this->checkForBinaryAnswer(&replyDummy);
          if (cmdLen == -1)
          {
            RCLCPP_INFO(node->get_logger(), "BINARY REPLY REQUIRED");
          }
          else
          {
            long dummy0, dummy1, identLen, versionLen;
            dummy0 = 0;
            dummy1 = 0;
            identLen = 0;
            versionLen = 0;

            const char *scanMask0 = "%04y%04ysRA DeviceIdent %02y";
            const char *scanMask1 = "%02y";
            unsigned char *replyPtr = &(replyDummy[0]);
            int scanDataLen0 = binScanfGuessDataLenFromMask(scanMask0);
            int scanDataLen1 = binScanfGuessDataLenFromMask(scanMask1); // should be: 2
            binScanfVec(&replyDummy, scanMask0, &dummy0, &dummy1, &identLen);

            std::string identStr = binScanfGetStringFromVec(&replyDummy, scanDataLen0, identLen);
            int off = scanDataLen0 + identLen; // consuming header + fixed part + ident

            std::vector<unsigned char> restOfReplyDummy = std::vector<unsigned char>(replyDummy.begin() + off,
                                                                                     replyDummy.end());

            versionLen = 0;
            binScanfVec(&restOfReplyDummy, "%02y", &versionLen);
            std::string versionStr = binScanfGetStringFromVec(&restOfReplyDummy, scanDataLen1, versionLen);
            std::string fullIdentVersionInfo = identStr + " V" + versionStr;
            // diagnostics_.setHardwareID(fullIdentVersionInfo);
            if (!isCompatibleDevice(fullIdentVersionInfo))
            {
              return ExitFatal;
            }

          }
          break;
        }
        case CMD_SERIAL_NUMBER:
          // diagnostics_.setHardwareID(sopasReplyStrVec[CMD_DEVICE_IDENT_LEGACY] + " " + sopasReplyStrVec[CMD_SERIAL_NUMBER]);

          if (!isCompatibleDevice(sopasReplyStrVec[CMD_DEVICE_IDENT_LEGACY]))
          {
            return ExitFatal;
          }

          break;
          /*
          DEVICE_STATE
          */
        case CMD_DEVICE_STATE:
        {
          int deviceState = -1;
          /*
          * Process device state, 0=Busy, 1=Ready, 2=Error
          * If configuration parameter is set, try resetting device in error state
          */

          int iRetVal = 0;
          if (useBinaryCmd)
          {
            long dummy0 = 0x00;
            long dummy1 = 0x00;
            deviceState = 0x00; // must be set to zero (because only one byte will be copied)
            iRetVal = binScanfVec(&(sopasReplyBinVec[CMD_DEVICE_STATE]), "%4y%4ysRA SCdevicestate %1y", &dummy0,
                                  &dummy1, &deviceState);
          }
          else
          {
            iRetVal = sscanf(sopasReplyStrVec[CMD_DEVICE_STATE].c_str(), "sRA SCdevicestate %d", &deviceState);
          }
          if (iRetVal > 0)  // 1 or 3 (depending of ASCII or Binary)
          {
            switch (deviceState)
            {
              case 0:
                RCLCPP_INFO(node->get_logger(), "Laser is busy");
                break;
              case 1:
                RCLCPP_INFO(node->get_logger(), "Laser is ready");
                break;
              case 2:
                RCLCPP_INFO(node->get_logger(), "Laser reports error state : %s",
                            sopasReplyStrVec[CMD_DEVICE_STATE].c_str());
                if (config_.auto_reboot)
                {
                  rebootScanner();
                };
                break;
              default:
                RCLCPP_INFO(node->get_logger(), "Laser reports unknown devicestate : %s ",
                            sopasReplyStrVec[CMD_DEVICE_STATE].c_str());
                break;
            }
          }
        }
          break;

        case CMD_OPERATION_HOURS:
        {
          int operationHours = -1;
          int iRetVal = 0;
          /*
          * Process device state, 0=Busy, 1=Ready, 2=Error
          * If configuration parameter is set, try resetting device in error state
          */
          if (useBinaryCmd)
          {
            long dummy0, dummy1;
            dummy0 = 0;
            dummy1 = 0;
            operationHours = 0;
            iRetVal = binScanfVec(&(sopasReplyBinVec[CMD_OPERATION_HOURS]), "%4y%4ysRA ODoprh %4y", &dummy0, &dummy1,
                                  &operationHours);
          }
          else
          {
            operationHours = 0;
            iRetVal = sscanf(sopasReplyStrVec[CMD_OPERATION_HOURS].c_str(), "sRA ODoprh %x", &operationHours);
          }
          if (iRetVal > 0)
          {
            double hours = 0.1 * operationHours;
#if TODO
            pn.setParam("operationHours", hours);
#endif
          }
        }
          break;

        case CMD_POWER_ON_COUNT:
        {
          int powerOnCount = -1;
          int iRetVal = -1;
          if (useBinaryCmd)
          {
            long dummy0, dummy1;
            powerOnCount = 0;
            iRetVal = binScanfVec(&(sopasReplyBinVec[CMD_POWER_ON_COUNT]), "%4y%4ysRA ODpwrc %4y", &dummy0, &dummy1,
                                  &powerOnCount);
          }
          else
          {
            iRetVal = sscanf(sopasReplyStrVec[CMD_POWER_ON_COUNT].c_str(), "sRA ODpwrc %x", &powerOnCount);
          }
          if (iRetVal > 0)
          {
#if TODO
            pn.setParam("powerOnCount", powerOnCount);
#endif
          }

        }
          break;

        case CMD_LOCATION_NAME:
        {
          char szLocationName[MAX_STR_LEN] = {0};
          const char *strPtr = sopasReplyStrVec[CMD_LOCATION_NAME].c_str();
          const char *searchPattern = "sRA LocationName "; // Bug fix (leading space) Jan 2018
          strcpy(szLocationName, "unknown location");
          if (useBinaryCmd)
          {
            int iRetVal = 0;
            long dummy0, dummy1, locationNameLen;
            const char *binLocationNameMask = "%4y%4ysRA LocationName %2y";
            int prefixLen = binScanfGuessDataLenFromMask(binLocationNameMask);
            dummy0 = 0;
            dummy1 = 0;
            locationNameLen = 0;

            iRetVal = binScanfVec(&(sopasReplyBinVec[CMD_LOCATION_NAME]), binLocationNameMask, &dummy0, &dummy1,
                                  &locationNameLen);
            if (iRetVal > 0)
            {
              std::string s;
              std::string locationNameStr = binScanfGetStringFromVec(&(sopasReplyBinVec[CMD_LOCATION_NAME]), prefixLen,
                                                                     locationNameLen);
              strcpy(szLocationName, locationNameStr.c_str());
            }
          }
          else
          {
            if (strstr(strPtr, searchPattern) == strPtr)
            {
              const char *ptr = strPtr + strlen(searchPattern);
              strcpy(szLocationName, ptr);
            }
            else
            {
              RCLCPP_INFO(node->get_logger(), "Location command not supported.\n");
            }
          }
#if TODO
          pn.setParam("locationName", std::string(szLocationName));
#endif
        }
          break;
        case CMD_GET_PARTIAL_SCANDATA_CFG:
        {

          const char *strPtr = sopasReplyStrVec[CMD_LOCATION_NAME].c_str();
          RCLCPP_INFO(node->get_logger(), "Config: %s\n", strPtr);
        }
          break;
          // ML: add here reply handling
      }

      if (restartDueToProcolChange)
      {
        return ExitError;

      }
    }


    if (this->parser_->getCurrentParamPtr()->getDeviceIsRadar())
    {
      //=====================================================
      // Radar specific commands
      //=====================================================
    }
    else
    {
      //-----------------------------------------------------------------
      //
      // This is recommended to decide between TiM551 and TiM561/TiM571 capabilities
      // The TiM551 has an angular resolution of 1.000 [deg]
      // The TiM561 and TiM571 have an angular resolution of 0.333 [deg]
      //-----------------------------------------------------------------

      angleRes10000th = (int) (0.5 + 10000.0 * this->parser_->getCurrentParamPtr()->getAngularDegreeResolution());
      std::vector<unsigned char> askOutputAngularRangeReply;

      if (useBinaryCmd)
      {
        std::vector<unsigned char> reqBinary;
        this->convertAscii2BinaryCmd(sopasCmdVec[CMD_GET_OUTPUT_RANGES].c_str(), &reqBinary);
        result = sendSopasAndCheckAnswer(reqBinary, &sopasReplyBinVec[CMD_GET_OUTPUT_RANGES]);
      }
      else
      {
        result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_GET_OUTPUT_RANGES].c_str(), &askOutputAngularRangeReply);
      }


      if (0 == result)
      {
        int askTmpAngleRes10000th = 0;
        int askTmpAngleStart10000th = 0;
        int askTmpAngleEnd10000th = 0;
        char dummy0[MAX_STR_LEN] = {0};
        char dummy1[MAX_STR_LEN] = {0};
        int dummyInt = 0;

        int iDummy0, iDummy1;
        std::string askOutputAngularRangeStr = replyToString(askOutputAngularRangeReply);
        // Binary-Reply Tab. 63
        // 0x20 Space
        // 0x00 0x01 -
        // 0x00 0x00 0x05 0x14  // Resolution in 1/10000th degree  --> 0.13°
        // 0x00 0x04 0x93 0xE0  // Start Angle 300000    -> 30°
        // 0x00 0x16 0xE3 0x60  // End Angle   1.500.000 -> 150°    // in ROS +/-60°
        // 0x83                 // Checksum

        int numArgs;

        if (useBinaryCmd)
        {
          iDummy0 = 0;
          iDummy1 = 0;
          dummyInt = 0;
          askTmpAngleRes10000th = 0;
          askTmpAngleStart10000th = 0;
          askTmpAngleEnd10000th = 0;

          const char *askOutputAngularRangeBinMask = "%4y%4ysRA LMPoutputRange %2y%4y%4y%4y";
          numArgs = binScanfVec(&sopasReplyBinVec[CMD_GET_OUTPUT_RANGES], askOutputAngularRangeBinMask, &iDummy0,
                                &iDummy1,
                                &dummyInt,
                                &askTmpAngleRes10000th,
                                &askTmpAngleStart10000th,
                                &askTmpAngleEnd10000th);
          //
        }
        else
        {
          numArgs = sscanf(askOutputAngularRangeStr.c_str(), "%s %s %d %X %X %X", dummy0, dummy1,
                           &dummyInt,
                           &askTmpAngleRes10000th,
                           &askTmpAngleStart10000th,
                           &askTmpAngleEnd10000th);
        }
        if (numArgs >= 6)
        {
          double askTmpAngleRes = askTmpAngleRes10000th / 10000.0;
          double askTmpAngleStart = askTmpAngleStart10000th / 10000.0;
          double askTmpAngleEnd = askTmpAngleEnd10000th / 10000.0;

          angleRes10000th = askTmpAngleRes10000th;
          RCLCPP_INFO(node->get_logger(), "Angle resolution of scanner is %0.5lf [deg]  (in 1/10000th deg: 0x%X)",
                      askTmpAngleRes, askTmpAngleRes10000th);

        }
      }
      //-----------------------------------------------------------------
      //
      // Set Min- und Max scanning angle given by config
      //
      //-----------------------------------------------------------------


      RCLCPP_INFO(node->get_logger(), "MIN_ANG: %8.3f [rad] %8.3f [deg]", config_.min_ang,
                  rad2deg(this->config_.min_ang));
      RCLCPP_INFO(node->get_logger(), "MAX_ANG: %8.3f [rad] %8.3f [deg]", config_.max_ang,
                  rad2deg(this->config_.max_ang));

      // convert to 10000th degree
      double minAngSopas = rad2deg(this->config_.min_ang);
      double maxAngSopas = rad2deg(this->config_.max_ang);

      if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_TIM_240_NAME) == 0)
      {
        // the TiM240 operates directly in the ros coordinate system
      }
      else
      {
        //TODO change scanAngleShift for other scanners
        //minAngSopas += 90.0;
        //maxAngSopas += 90.0;
      }
      angleStart10000th = (int) (0.5 + 10000.0 * minAngSopas);
      angleEnd10000th = (int) (0.5 + 10000.0 * maxAngSopas);

      char requestOutputAngularRange[MAX_STR_LEN];
      // special for LMS1000
      if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_LMS_1XXX_NAME) == 0)
      {
        RCLCPP_INFO(node->get_logger(), "Angular settings for LMS 1000 not reliable.\n");
        double askAngleStart = -137.0;
        double askAngleEnd = +137.0;

        this->config_.min_ang = askAngleStart;
        this->config_.max_ang = askAngleEnd;
      }
      else
      {
        std::vector<unsigned char> outputAngularRangeReply;
        const char *pcCmdMask = sopasCmdMaskVec[CMD_SET_OUTPUT_RANGES].c_str();
        sprintf(requestOutputAngularRange, pcCmdMask, angleRes10000th, angleStart10000th, angleEnd10000th);

        if (useBinaryCmd)
        {
          unsigned char tmpBuffer[255] = {0};
          unsigned char sendBuffer[255] = {0};
          UINT16 sendLen;
          std::vector<unsigned char> reqBinary;
          int iStatus = 1;
          //				const char *askOutputAngularRangeBinMask = "%4y%4ysWN LMPoutputRange %2y%4y%4y%4y";
          // int askOutputAngularRangeBinLen = binScanfGuessDataLenFromMask(askOutputAngularRangeBinMask);
          // askOutputAngularRangeBinLen -= 8;  // due to header and length identifier

          strcpy((char *) tmpBuffer, "WN LMPoutputRange ");
          unsigned short orgLen = strlen((char *) tmpBuffer);
          colab::addIntegerToBuffer<UINT16>(tmpBuffer, orgLen, iStatus);
          colab::addIntegerToBuffer<UINT32>(tmpBuffer, orgLen, angleRes10000th);
          colab::addIntegerToBuffer<UINT32>(tmpBuffer, orgLen, angleStart10000th);
          colab::addIntegerToBuffer<UINT32>(tmpBuffer, orgLen, angleEnd10000th);
          sendLen = orgLen;
          colab::addFrameToBuffer(sendBuffer, tmpBuffer, &sendLen);

          // binSprintfVec(&reqBinary, askOutputAngularRangeBinMask, 0x02020202, askOutputAngularRangeBinLen, iStatus, angleRes10000th, angleStart10000th, angleEnd10000th);

          // unsigned char sickCrc = sick_crc8((unsigned char *)(&(reqBinary)[8]), reqBinary.size() - 8);
          // reqBinary.push_back(sickCrc);
          reqBinary = std::vector<unsigned char>(sendBuffer, sendBuffer + sendLen);
          // Here we must build a more complex binaryRequest

          // this->convertAscii2BinaryCmd(requestOutputAngularRange, &reqBinary);
          result = sendSopasAndCheckAnswer(reqBinary, &outputAngularRangeReply);
        }
        else
        {
          result = sendSopasAndCheckAnswer(requestOutputAngularRange, &outputAngularRangeReply);
        }
      }

      //-----------------------------------------------------------------
      //
      // Get Min- und Max scanning angle from the scanner to verify angle setting and correct the config, if something went wrong.
      //
      // IMPORTANT:
      // Axis Orientation in ROS differs from SICK AXIS ORIENTATION!!!
      // In relation to a body the standard is:
      // x forward
      // y left
      // z up
      // see http://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions for more details
      //-----------------------------------------------------------------

      askOutputAngularRangeReply.clear();

      if (useBinaryCmd)
      {
        std::vector<unsigned char> reqBinary;
        this->convertAscii2BinaryCmd(sopasCmdVec[CMD_GET_OUTPUT_RANGES].c_str(), &reqBinary);
        result = sendSopasAndCheckAnswer(reqBinary, &sopasReplyBinVec[CMD_GET_OUTPUT_RANGES]);
      }
      else
      {
        result = sendSopasAndCheckAnswer(sopasCmdVec[CMD_GET_OUTPUT_RANGES].c_str(), &askOutputAngularRangeReply);
      }

      if (result == 0)
      {
        char dummy0[MAX_STR_LEN] = {0};
        char dummy1[MAX_STR_LEN] = {0};
        int dummyInt = 0;
        int askAngleRes10000th = 0;
        int askAngleStart10000th = 0;
        int askAngleEnd10000th = 0;
        int iDummy0, iDummy1;
        iDummy0 = 0;
        iDummy1 = 0;
        std::string askOutputAngularRangeStr = replyToString(askOutputAngularRangeReply);
        // Binary-Reply Tab. 63
        // 0x20 Space
        // 0x00 0x01 -
        // 0x00 0x00 0x05 0x14  // Resolution in 1/10000th degree  --> 0.13°
        // 0x00 0x04 0x93 0xE0  // Start Angle 300000    -> 30°
        // 0x00 0x16 0xE3 0x60  // End Angle   1.500.000 -> 150°    // in ROS +/-60°
        // 0x83                 // Checksum

        int numArgs;

        /*
         *
         *  Initialize variables
         */

        iDummy0 = 0;
        iDummy1 = 0;
        dummyInt = 0;
        askAngleRes10000th = 0;
        askAngleStart10000th = 0;
        askAngleEnd10000th = 0;

        /*
         *   scan values
         *
         */

        if (useBinaryCmd)
        {
          const char *askOutputAngularRangeBinMask = "%4y%4ysRA LMPoutputRange %2y%4y%4y%4y";
          numArgs = binScanfVec(&sopasReplyBinVec[CMD_GET_OUTPUT_RANGES], askOutputAngularRangeBinMask, &iDummy0,
                                &iDummy1,
                                &dummyInt,
                                &askAngleRes10000th,
                                &askAngleStart10000th,
                                &askAngleEnd10000th);
          //
        }
        else
        {
          numArgs = sscanf(askOutputAngularRangeStr.c_str(), "%s %s %d %X %X %X", dummy0, dummy1,
                           &dummyInt,
                           &askAngleRes10000th,
                           &askAngleStart10000th,
                           &askAngleEnd10000th);
        }
        if (numArgs >= 6)
        {
          double askTmpAngleRes = askAngleRes10000th / 10000.0;
          double askTmpAngleStart = askAngleStart10000th / 10000.0;
          double askTmpAngleEnd = askAngleEnd10000th / 10000.0;

          angleRes10000th = askAngleRes10000th;
          RCLCPP_INFO(node->get_logger(), "Angle resolution of scanner is %0.5lf [deg]  (in 1/10000th deg: 0x%X)",
                      askTmpAngleRes, askAngleRes10000th);

        }
        double askAngleRes = askAngleRes10000th / 10000.0;
        double askAngleStart = askAngleStart10000th / 10000.0;
        double askAngleEnd = askAngleEnd10000th / 10000.0;
        double angshift= this->parser_->getCurrentParamPtr()->getScanAngleShift();
        this->config_.min_ang = (askAngleStart / 180.0 * M_PI);
        this->config_.max_ang = (askAngleEnd / 180.0 * M_PI);
        RCLCPP_INFO(node->get_logger(), "MIN_ANG (after command verification in device reference frame): %8.3f [rad] %8.3f [deg]",
                    config_.min_ang, rad2deg(this->config_.min_ang));
        RCLCPP_INFO(node->get_logger(), "MAX_ANG (after command verification in device reference frame): %8.3f [rad] %8.3f [deg]",
                    config_.max_ang, rad2deg(this->config_.max_ang));
      }



      //-----------------------------------------------------------------
      //
      // Configure the data content of scan messing regarding to config.
      //
      //-----------------------------------------------------------------
      /*
      see 4.3.1 Configure the data content for the scan in the
      */
      //                              1    2     3
      // Prepare flag array for echos
      // Except for the LMS5xx scanner here the mask is hard 00 see SICK Telegram listing "Telegram structure: sWN LMDscandatacfg" for details

      outputChannelFlagId = 0x00;
      for (int i = 0; i < outputChannelFlag.size(); i++)
      {
        outputChannelFlagId |= ((outputChannelFlag[i] == true) << i);
      }
      if (outputChannelFlagId < 1)
      {
        outputChannelFlagId = 1;  // at least one channel must be set
      }
      if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_LMS_5XX_NAME) == 0)
      {
        outputChannelFlagId = 1;
        RCLCPP_INFO(node->get_logger(), "LMS 5xx detected overwriting output channel flag ID");
        RCLCPP_INFO(node->get_logger(), "LMS 5xx detected overwriting resolution flag (only 8 bit supported)");
        this->parser_->getCurrentParamPtr()->setIntensityResolutionIs16Bit(false);
        rssiResolutionIs16Bit = this->parser_->getCurrentParamPtr()->getIntensityResolutionIs16Bit();
      }
      if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_MRS_1XXX_NAME) == 0)
      {
        RCLCPP_INFO(node->get_logger(), "MRS 1xxx detected overwriting resolution flag (only 8 bit supported)");
        this->parser_->getCurrentParamPtr()->setIntensityResolutionIs16Bit(false);
        rssiResolutionIs16Bit = this->parser_->getCurrentParamPtr()->getIntensityResolutionIs16Bit();

      }






      // set scanning angle for tim5xx and for mrs1104
      if ((this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 1)
          || (this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 4)
          || (this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 24)
          )
      {
        char requestLMDscandatacfg[MAX_STR_LEN];
        // Uses sprintf-Mask to set bitencoded echos and rssi enable flag
        // CMD_SET_PARTIAL_SCANDATA_CFG = "\x02sWN LMDscandatacfg %02d 00 %d 0 0 00 00 0 0 0 0 1\x03";
        const char *pcCmdMask = sopasCmdMaskVec[CMD_SET_PARTIAL_SCANDATA_CFG].c_str();
        sprintf(requestLMDscandatacfg, pcCmdMask, outputChannelFlagId, rssiFlag ? 1 : 0, rssiResolutionIs16Bit ? 1 : 0);
        if (useBinaryCmd)
        {
          std::vector<unsigned char> reqBinary;
          this->convertAscii2BinaryCmd(requestLMDscandatacfg, &reqBinary);
          // FOR MRS6124 this should be
          // like this:
          // 0000  02 02 02 02 00 00 00 20 73 57 4e 20 4c 4d 44 73   .......sWN LMDs
          // 0010  63 61 6e 64 61 74 61 63 66 67 20 1f 00 01 01 00   candatacfg .....
          // 0020  00 00 00 00 00 00 00 01 5c
          result = sendSopasAndCheckAnswer(reqBinary, &sopasReplyBinVec[CMD_SET_PARTIAL_SCANDATA_CFG]);
        }
        else
        {
          std::vector<unsigned char> lmdScanDataCfgReply;
          result = sendSopasAndCheckAnswer(requestLMDscandatacfg, &lmdScanDataCfgReply);
        }


        // check setting
        char requestLMDscandatacfgRead[MAX_STR_LEN];
        // Uses sprintf-Mask to set bitencoded echos and rssi enable flag

        strcpy(requestLMDscandatacfgRead, sopasCmdVec[CMD_GET_PARTIAL_SCANDATA_CFG].c_str());
        if (useBinaryCmd)
        {
          std::vector<unsigned char> reqBinary;
          this->convertAscii2BinaryCmd(requestLMDscandatacfgRead, &reqBinary);
          result = sendSopasAndCheckAnswer(reqBinary, &sopasReplyBinVec[CMD_GET_PARTIAL_SCANDATA_CFG]);
        }
        else
        {
          std::vector<unsigned char> lmdScanDataCfgReadReply;
          result = sendSopasAndCheckAnswer(requestLMDscandatacfgRead, &lmdScanDataCfgReadReply);
        }


      }

      // CONFIG ECHO-Filter (only for MRS1000 not available for TiM5xx
      if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() >= 4)
      {
        char requestEchoSetting[MAX_STR_LEN];
        int filterEchoSetting = 0;
#if TODO
        pn.getParam("filter_echos", filterEchoSetting); // filter_echos
#endif
        if (filterEchoSetting < 0)
        { filterEchoSetting = 0; }
        if (filterEchoSetting > 2)
        { filterEchoSetting = 2; }
        // Uses sprintf-Mask to set bitencoded echos and rssi enable flag
        const char *pcCmdMask = sopasCmdMaskVec[CMD_SET_ECHO_FILTER].c_str();
        /*
        First echo : 0
        All echos : 1
        Last echo : 2
        */
        sprintf(requestEchoSetting, pcCmdMask, filterEchoSetting);
        std::vector<unsigned char> outputFilterEchoRangeReply;


        if (useBinaryCmd)
        {
          std::vector<unsigned char> reqBinary;
          this->convertAscii2BinaryCmd(requestEchoSetting, &reqBinary);
          result = sendSopasAndCheckAnswer(reqBinary, &sopasReplyBinVec[CMD_SET_ECHO_FILTER]);
        }
        else
        {
          result = sendSopasAndCheckAnswer(requestEchoSetting, &outputFilterEchoRangeReply);
        }

      }


    }
    //////////////////////////////////////////////////////////////////////////////


    //-----------------------------------------------------------------
    //
    // Start sending LMD-Scandata messages
    //
    //-----------------------------------------------------------------
    std::vector<int> startProtocolSequence;
    bool deviceIsRadar = false;
    {
      // initializing sequence for laserscanner
      // is this device a TiM240????
      // The TiM240 can not interpret CMD_START_MEASUREMENT
      if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_TIM_240_NAME) == 0)
      {
        // the TiM240 operates directly in the ros coordinate system
        // do nothing for a TiM240
      }
      else
      {
        startProtocolSequence.push_back(CMD_START_MEASUREMENT);
      }
      startProtocolSequence.push_back(CMD_RUN);  // leave user level
      startProtocolSequence.push_back(CMD_START_SCANDATA);
      if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 4)  // MRS1104 - start IMU-Transfer
      {
        bool imu_enable = config_.imu_enable;
        if (imu_enable)
        {
          if (useBinaryCmdNow == true)
          {
            RCLCPP_INFO(node->get_logger(), "Enable IMU data transfer");
            // TODO Flag to decide between IMU on or off
            startProtocolSequence.push_back(CMD_START_IMU_DATA);
          }
          else
          {
            RCLCPP_INFO(node->get_logger(),
                        "IMU USAGE NOT POSSIBLE IN ASCII COMMUNICATION MODE.\nTo use the IMU the communication with the scanner must be set to binary mode.\n This can be done by inserting the line:\n<param name=\"use_binary_protocol\" type=\"bool\" value=\"True\" />\n into the launchfile.\n See also https://github.com/SICKAG/sick_scan/blob/master/doc/IMU.md");
            exit(0);
          }

        }
      }
    }

    std::vector<int>::iterator it;
    for (it = startProtocolSequence.begin(); it != startProtocolSequence.end(); it++)
    {
      int cmdId = *it;
      std::vector<unsigned char> tmpReply;
      //			sendSopasAndCheckAnswer(sopasCmdVec[cmdId].c_str(), &tmpReply);

      std::string sopasCmd = sopasCmdVec[cmdId];
      std::vector<unsigned char> replyDummy;
      std::vector<unsigned char> reqBinary;
      std::vector<unsigned char> sopasVec;
      sopasVec = stringToVector(sopasCmd);
      RCLCPP_INFO(node->get_logger(), "Command: %s", stripControl(sopasVec).c_str());
      if (useBinaryCmd)
      {
        this->convertAscii2BinaryCmd(sopasCmd.c_str(), &reqBinary);
        result = sendSopasAndCheckAnswer(reqBinary, &replyDummy, cmdId);
        sopasReplyBinVec[cmdId] = replyDummy;

        switch (cmdId)
        {
          case CMD_START_SCANDATA:
            // ROS_DEBUG("Sleeping for a couple of seconds of start measurement\n");
            // ros::Duration(10.0).sleep();
            break;
        }
      }
      else
      {
        result = sendSopasAndCheckAnswer(sopasCmd.c_str(), &replyDummy, cmdId);
      }

      if (result != 0)
      {
        RCLCPP_INFO(node->get_logger(), "%s", sopasCmdErrMsg[cmdId].c_str());
        //diagnostics_.broadcast(getDiagnosticErrorCode(), sopasCmdErrMsg[cmdId]);
      }
      else
      {
        sopasReplyStrVec[cmdId] = replyToString(replyDummy);
      }


      if (cmdId == CMD_START_RADARDATA)
      {
        RCLCPP_INFO(node->get_logger(), "Starting radar data ....\n");
      }


      if (cmdId == CMD_START_SCANDATA)
      {
        RCLCPP_INFO(node->get_logger(), "Starting scan data ....\n");
      }

      if (cmdId == CMD_RUN)
      {
        bool waitForDeviceState = true;
        if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 1)
        {
          waitForDeviceState = false; // do nothing for tim5xx
        }
        if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() == 24)
        {
          waitForDeviceState = false; // do nothing for MRS6xxx
        }

        if (waitForDeviceState)
        {
          int maxWaitForDeviceStateReady = 45;   // max. 45 sec. (see manual)
          bool scannerReady = false;
          for (int i = 0; i < maxWaitForDeviceStateReady; i++)
          {
            double shortSleepTime = 1.0;
            std::string sopasCmd = sopasCmdVec[CMD_DEVICE_STATE];
            std::vector<unsigned char> replyDummy;

            int iRetVal = 0;
            int deviceState = 0;

            std::vector<unsigned char> reqBinary;
            std::vector<unsigned char> sopasVec;
            sopasVec = stringToVector(sopasCmd);
//            RCLCPP_INFO(node->get_logger(), "Command: %s", stripControl(sopasVec).c_str());
            if (useBinaryCmd)
            {
              this->convertAscii2BinaryCmd(sopasCmd.c_str(), &reqBinary);
              result = sendSopasAndCheckAnswer(reqBinary, &replyDummy);
              sopasReplyBinVec[CMD_DEVICE_STATE] = replyDummy;
            }
            else
            {
              result = sendSopasAndCheckAnswer(sopasCmd.c_str(), &replyDummy);
              sopasReplyStrVec[CMD_DEVICE_STATE] = replyToString(replyDummy);
            }


            if (useBinaryCmd)
            {
              long dummy0, dummy1;
              dummy0 = 0;
              dummy1 = 0;
              deviceState = 0;
              iRetVal = binScanfVec(&(sopasReplyBinVec[CMD_DEVICE_STATE]), "%4y%4ysRA SCdevicestate %1y", &dummy0,
                                    &dummy1, &deviceState);
            }
            else
            {
              iRetVal = sscanf(sopasReplyStrVec[CMD_DEVICE_STATE].c_str(), "sRA SCdevicestate %d", &deviceState);
            }
            if (iRetVal > 0)  // 1 or 3 (depending of ASCII or Binary)
            {
              if (deviceState == 1) // scanner is ready
              {
                scannerReady = true; // interrupt waiting for scanner ready
                RCLCPP_INFO(node->get_logger(), "Scanner ready for measurement after %d [sec]", i);
                break;
              }
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for scanner ready state since %d secs", i);
            const std::chrono::nanoseconds nsshortSleepTime((int64_t)(shortSleepTime*1e9));
            rclcpp::sleep_for(nsshortSleepTime);

            if (scannerReady)
            {
              break;
            }
            if(i==(maxWaitForDeviceStateReady-1))
            {
              RCLCPP_INFO(node->get_logger(), "TIMEOUT WHILE STARTING SCANNER (%d [sec])", i);
              return ExitError;
            }
          }
        }
      }
      tmpReply.clear();

    }
    return ExitSuccess;
  }


  /*!
  \brief convert ASCII or binary reply to a human readable string
  \param reply datablock, which should be converted
  \return human readable string (used for debug/monitoring output)
  */
  std::string sick_scan::SickScanCommon::replyToString(const std::vector<unsigned char> &reply)
  {
    std::string reply_str;
    std::vector<unsigned char>::const_iterator it_start, it_end;
    int binLen = this->checkForBinaryAnswer(&reply);
    if (binLen == -1) // ASCII-Cmd
    {
      it_start = reply.begin();
      it_end = reply.end();
    }
    else
    {
      it_start = reply.begin() + 8; // skip header and length id
      it_end = reply.end() - 1; // skip CRC
    }
    bool inHexPrintMode = false;
    for (std::vector<unsigned char>::const_iterator it = it_start; it != it_end; it++)
    {
      // inHexPrintMode means that we should continue printing hex value after we started with hex-Printing
      // That is easier to debug for a human instead of switching between ascii binary and then back to ascii
      if (*it >= 0x20 && (inHexPrintMode == false)) // filter control characters for display
      {
        reply_str.push_back(*it);
      }
      else
      {
        if (binLen != -1) // binary
        {
          char szTmp[255] = {0};
          unsigned char val = *it;
          inHexPrintMode = true;
          sprintf(szTmp, "\\x%02x", val);
          for (int ii = 0; ii < strlen(szTmp); ii++)
          {
            reply_str.push_back(szTmp[ii]);
          }
        }
      }

    }
    return reply_str;
  }

  bool sick_scan::SickScanCommon::dumpDatagramForDebugging(unsigned char *buffer, int bufLen)
  {
    bool ret = true;
    static int cnt = 0;
    char szDumpFileName[300] = {0};
    char szDir[255] = {0};
    if (cnt == 0)
    {
      printf("Attention: verboseLevel is set to 1. Datagrams are stored in the /tmp folder.");
    }
#ifdef _MSC_VER
    strcpy(szDir, "C:\\temp\\");
#else
    strcpy(szDir, "/tmp/");
#endif
    sprintf(szDumpFileName, "%ssick_datagram_%06d.bin", szDir, cnt);
    bool isBinary = this->parser_->getCurrentParamPtr()->getUseBinaryProtocol();
    if (isBinary)
    {
      FILE *ftmp;
      ftmp = fopen(szDumpFileName, "wb");
      if (ftmp != NULL)
      {
        fwrite(buffer, bufLen, 1, ftmp);
        fclose(ftmp);
      }
    }
    cnt++;

    return (true);

  }


  /*!
  \brief check the identification string
  \param identStr string (got from sopas request)
  \return true, if this driver supports the scanner identified by the identification string
  */
  bool sick_scan::SickScanCommon::isCompatibleDevice(const std::string identStr) const
  {
    char device_string[7];
    int version_major = -1;
    int version_minor = -1;

    strcpy(device_string, "???");
    // special for TiM3-Firmware
    if (sscanf(identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string,
               &version_major, &version_minor) == 3
        && strncmp("TiM3", device_string, 4) == 0
        && version_major >= 2 && version_minor >= 50)
    {
      printf("This scanner model/firmware combination does not support ranging output!");
      printf("Supported scanners: TiM5xx: all firmware versions; TiM3xx: firmware versions < V2.50.");
      printf("This is a %s, firmware version %d.%d", device_string, version_major, version_minor);

      return false;
    }

    bool supported = false;

    // DeviceIdent 8 MRS1xxxx 8 1.3.0.0R.
    if (sscanf(identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string, &version_major, &version_minor) == 3)
    {
      std::string devStr = device_string;


      if (devStr.compare(0, 4, "TiM5") == 0)
      {
        supported = true;
      }

      if (supported == true)
      {
        printf("Device %s V%d.%d found and supported by this driver.", identStr.c_str(), version_major, version_minor);
      }

    }

    if ((identStr.find("MRS1xxx") !=
         std::string::npos)   // received pattern contains 4 'x' but we check only for 3 'x' (MRS1104 should be MRS1xxx)
        || (identStr.find("LMS1xxx") != std::string::npos)
        )
    {
      printf("Deviceinfo %s found and supported by this driver.", identStr.c_str());
      supported = true;
    }


    if (identStr.find("MRS6") !=
        std::string::npos)  // received pattern contains 4 'x' but we check only for 3 'x' (MRS1104 should be MRS1xxx)
    {
      printf("Deviceinfo %s found and supported by this driver.", identStr.c_str());
      supported = true;
    }

    if (identStr.find("RMS3xx") !=
        std::string::npos)   // received pattern contains 4 'x' but we check only for 3 'x' (MRS1104 should be MRS1xxx)
    {
      printf("Deviceinfo %s found and supported by this driver.", identStr.c_str());
      supported = true;
    }


    if (supported == false)
    {
      printf("Device %s V%d.%d found and maybe unsupported by this driver.", device_string, version_major,
             version_minor);
      printf("Full SOPAS answer: %s", identStr.c_str());
    }
    return true;
  }


  /*!
  \brief parsing datagram and publishing ros messages
  \return error code
  */
  int SickScanCommon::loopOnce()
  {
    static int cnt = 0;
    int seqId = 0; // seqId replaces the seq-Identifier in a ros1-message-header
    // diagnostics_.update();

    unsigned char receiveBuffer[65536];
    int actual_length = 0;
    static unsigned int iteration_count = 0;
    bool useBinaryProtocol = this->parser_->getCurrentParamPtr()->getUseBinaryProtocol();
    int result = 0;

    builtin_interfaces::msg::Time recvTimeStamp;
#if TODO
    ros::Time recvTimeStamp = ros::Time::now();  // timestamp incoming package, will be overwritten by get_datagram
    // tcp packet
    ros::Time recvTimeStampPush = ros::Time::now();  // timestamp
    result = get_datagram(recvTimeStamp, receiveBuffer, 65536, &actual_length, useBinaryProtocol);
#endif
#if TODO
    ros::Duration dur = recvTimeStampPush - recvTimeStamp;
#endif
    result = get_datagram(recvTimeStamp, receiveBuffer, 65536, &actual_length, useBinaryProtocol);

    if (result != 0)
    {
      printf("Read Error when getting datagram: %i.", result);
      // diagnostics_.broadcast(getDiagnosticErrorCode(), "Read Error when getting datagram.");
      return ExitError; // return failure to exit node
    }
    if (actual_length <= 0)
    {
      return ExitSuccess;
    } // return success to continue looping

    // ----- if requested, skip frames
    if (iteration_count++ % (config_.skip + 1) != 0)
      return ExitSuccess;

    if (publish_datagram_)
    {
    }
#if 0
    /* Dump Binary Protocol */
    ros::NodeHandle tmpParam("~");



    std::string echoForSlam = "";
    bool slamBundle = false;
    tmpParam.getParam("slam_echo", echoForSlam);
    tmpParam.getParam("slam_bundle", slamBundle);
    bool dumpData = false;
    int verboseLevel = 0;
    tmpParam.getParam("verboseLevel", verboseLevel);
    if (verboseLevel > 0)
    {
      dumpDatagramForDebugging(receiveBuffer, actual_length);
    }
#endif

    bool deviceIsRadar = false;

    if (this->parser_->getCurrentParamPtr()->getDeviceIsRadar() == true)
    {
      deviceIsRadar = true;
    }

    if (true == deviceIsRadar)
    {
#if 0
      SickScanRadar radar(this);
    int errorCode = ExitSuccess;
    // parse radar telegram and send pointcloud2-debug messages
    errorCode = radar.parseDatagram(recvTimeStamp, (unsigned char *)receiveBuffer, actual_length, useBinaryProtocol);
    return errorCode; // return success to continue looping
#endif
    }
    static SickScanImu scanImu(this); // todo remove static
    if (scanImu.isImuDatagram((char *) receiveBuffer, actual_length))
    {
      int errorCode = ExitSuccess;
      if (scanImu.isImuAckDatagram((char *) receiveBuffer, actual_length))
      {

      }
      else
      {
        // parse radar telegram and send pointcloud2-debug messages
        errorCode = scanImu.parseDatagram(recvTimeStamp, (unsigned char *) receiveBuffer, actual_length,
                                          useBinaryProtocol);

      }
      return errorCode; // return success to continue looping


    }
    else
    {

      sensor_msgs::msg::LaserScan msg;

      // msg.header.stamp = recvTimeStamp;
      double elevationAngleInRad = 0.0;
      short elevAngleX200 = 0;  // signed short (F5 B2  -> Layer 24
      // F5B2h -> -2638/200= -13.19°
      /*
       * datagrams are enclosed in <STX> (0x02), <ETX> (0x03) pairs
       */
      char *buffer_pos = (char *) receiveBuffer;
      char *dstart, *dend;
      bool dumpDbg = false;
      bool dataToProcess = true;
      std::vector<float> vang_vec;
      vang_vec.clear();
      while (dataToProcess)
      {
        const int maxAllowedEchos = 5;

        int numValidEchos = 0;
        int aiValidEchoIdx[maxAllowedEchos] = {0};
        size_t dlength;
        int success = -1;
        int numEchos = 0;
        int echoMask = 0;
        bool publishPointCloud = true;
        if (useBinaryProtocol)
        {
          // if binary protocol used then parse binary message
          std::vector<unsigned char> receiveBufferVec = std::vector<unsigned char>(receiveBuffer,
                                                                                   receiveBuffer + actual_length);

          if (receiveBufferVec.size() > 8)
          {
            long idVal = 0;
            long lenVal = 0;
            memcpy(&idVal, receiveBuffer + 0, 4);
            memcpy(&lenVal, receiveBuffer + 4, 4);
            swap_endian((unsigned char *) &lenVal, 4);

            if (idVal == 0x02020202)
            {
              // binary message
              if (lenVal < actual_length)
              {
                int scanFrequencyX100 = 0;
                double elevAngle = 0.00;
                double scanFrequency = 0.0;
                long measurementFrequencyDiv100 = 0; // multiply with 100
                int numberOf16BitChannels = 0;
                int numberOf8BitChannels = 0;
                uint32_t SystemCountScan = 0;
                static uint32_t lastSystemCountScan = 0;// this variable is used to ensure that only the first time stamp of an multi layer scann is used for PLL updating
                uint32_t SystemCountTransmit = 0;
                memcpy(&elevAngleX200, receiveBuffer + 50, 2);
                swap_endian((unsigned char *) &elevAngleX200, 2);

                elevationAngleInRad = -elevAngleX200 / 200.0 * deg2rad_const;
                // msg.header.seq = elevAngleX200; // should be multiple of 0.625° starting with -2638 (corresponding to 13.19°)

                memcpy(&scanFrequencyX100, receiveBuffer + 52, 4);
                swap_endian((unsigned char *) &scanFrequencyX100, 4);

                memcpy(&measurementFrequencyDiv100, receiveBuffer + 56, 4);
                swap_endian((unsigned char *) &measurementFrequencyDiv100, 4);


                msg.scan_time = 1.0 / (scanFrequencyX100 / 100.0);
                msg.time_increment = 1.0 / (measurementFrequencyDiv100 * 100.0);

                msg.range_min = parser_->get_range_min();
                msg.range_max = parser_->get_range_max();
                memcpy(&SystemCountScan, receiveBuffer + 0x26, 4);
                swap_endian((unsigned char *) &SystemCountScan, 4);

                memcpy(&SystemCountTransmit, receiveBuffer + 0x2A, 4);
                swap_endian((unsigned char *) &SystemCountTransmit, 4);
                double timestampfloat = recvTimeStamp.sec + recvTimeStamp.nanosec * 1e-9;
                bool bRet;
                if (SystemCountScan !=
                    lastSystemCountScan)//MRS 6000 sends 6 packets with same  SystemCountScan we should only update the pll once with this time stamp since the SystemCountTransmit are different and this will only increase jitter of the pll
                {
                  bRet = SoftwarePLL::instance().updatePLL(recvTimeStamp.sec, recvTimeStamp.nanosec,
                                                           SystemCountTransmit);
                  lastSystemCountScan = SystemCountScan;
                }

                bRet = SoftwarePLL::instance().getCorrectedTimeStamp(&recvTimeStamp,
                                                                     SystemCountScan);
                double timestampfloat_coor = recvTimeStamp.sec + recvTimeStamp.nanosec * 1e-9;
                double DeltaTime = timestampfloat - timestampfloat_coor;
                //ROS_INFO("%F,%F,%u,%u,%F",timestampfloat,timestampfloat_coor,SystemCountTransmit,SystemCountScan,DeltaTime);
                //TODO Handle return values
                if (config_.sw_pll_only_publish == true && bRet == false)
                {
                  int packets_expected_to_drop = SoftwarePLL::instance().fifoSize - 1;
                  SoftwarePLL::instance().packeds_droped++;
                  RCLCPP_INFO(getMainNode()->get_logger(),"%i / %i Packet dropped Software PLL not yet locked.",
                           SoftwarePLL::instance().packeds_droped, packets_expected_to_drop);
                  if (SoftwarePLL::instance().packeds_droped == packets_expected_to_drop)
                  {

                    RCLCPP_INFO(getMainNode()->get_logger(),"Software PLL is expected to be ready now!");
                  }
                  if (SoftwarePLL::instance().packeds_droped > packets_expected_to_drop)
                  {
                    RCLCPP_WARN(getMainNode()->get_logger(),"More packages than expected were dropped!!\n"
                             "Check the network connection.\n"
                             "Check if the system time has been changed in a leap.\n"
                             "If the problems can persist, disable the software PLL with the option sw_pll_only_publish=False  !");
                  }
                  dataToProcess = false;
                  break;
                }

                memcpy(&numberOf16BitChannels, receiveBuffer + 62, 2);
                swap_endian((unsigned char *) &numberOf16BitChannels, 2);

                int parseOff = 64;


                char szChannel[255] = {0};
                float scaleFactor = 1.0;
                float scaleFactorOffset = 0.0;
                int32_t startAngleDiv10000 = 1;
                int32_t sizeOfSingleAngularStepDiv10000 = 1;
                double startAngle = 0.0;
                double sizeOfSingleAngularStep = 0.0;
                short numberOfItems = 0;

                static int cnt = 0;
                cnt++;
                // get number of 8 bit channels
                // we must jump of the 16 bit data blocks including header ...
                for (int i = 0; i < numberOf16BitChannels; i++)
                {
                  int numberOfItems = 0x00;
                  memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                  swap_endian((unsigned char *) &numberOfItems, 2);
                  parseOff += 21; // 21 Byte header followed by data entries
                  parseOff += numberOfItems * 2;
                }

                // now we can read the number of 8-Bit-Channels
                memcpy(&numberOf8BitChannels, receiveBuffer + parseOff, 2);
                swap_endian((unsigned char *) &numberOf8BitChannels, 2);

                parseOff = 64;
                enum datagram_parse_task
                {
                  process_dist,
                  process_vang,
                  process_rssi,
                  process_idle
                };
                for (int processLoop = 0; processLoop < 2; processLoop++)
                {
                  int totalChannelCnt = 0;
                  int distChannelCnt;
                  int rssiCnt;
                  bool bCont = true;
                  int vangleCnt;
                  datagram_parse_task task = process_idle;
                  bool parsePacket = true;
                  parseOff = 64;
                  bool processData = false;

                  if (processLoop == 0)
                  {
                    distChannelCnt = 0;
                    rssiCnt = 0;
                    vangleCnt = 0;
                  }

                  if (processLoop == 1)
                  {
                    processData = true;
                    numEchos = distChannelCnt;
                    msg.ranges.resize(numberOfItems * numEchos);
                    if (rssiCnt > 0)
                    {
                      msg.intensities.resize(numberOfItems * rssiCnt);
                    }
                    else
                    {
                    }
                    if (vangleCnt > 0) // should be 0 or 1
                    {
                      vang_vec.resize(numberOfItems * vangleCnt);
                    }
                    else
                    {
                      vang_vec.clear();
                    }
                    echoMask = (1 << numEchos) - 1;

                    // reset count. We will use the counter for index calculation now.
                    distChannelCnt = 0;
                    rssiCnt = 0;
                    vangleCnt = 0;

                  }

                  szChannel[6] = '\0';
                  scaleFactor = 1.0;
                  scaleFactorOffset = 0.0;
                  startAngleDiv10000 = 1;
                  sizeOfSingleAngularStepDiv10000 = 1;
                  startAngle = 0.0;
                  sizeOfSingleAngularStep = 0.0;
                  numberOfItems = 0;


#if 1 // prepared for multiecho parsing

                  bCont = true;
                  bool doVangVecProc = false;
                  // try to get number of DIST and RSSI from binary data
                  task = process_idle;
                  do
                  {
                    task = process_idle;
                    doVangVecProc = false;
                    int processDataLenValuesInBytes = 2;

                    if (totalChannelCnt == numberOf16BitChannels)
                    {
                      parseOff += 2; // jump of number of 8 bit channels- already parsed above
                    }

                    if (totalChannelCnt >= numberOf16BitChannels)
                    {
                      processDataLenValuesInBytes = 1; // then process 8 bit values ...
                    }
                    bCont = false;
                    strcpy(szChannel, "");

                    if (totalChannelCnt < (numberOf16BitChannels + numberOf8BitChannels))
                    {
                      szChannel[5] = '\0';
                      strncpy(szChannel, (const char *) receiveBuffer + parseOff, 5);
                    }
                    else
                    {
                      // all channels processed (16 bit and 8 bit channels)
                    }

                    if (strstr(szChannel, "DIST") == szChannel)
                    {
                      task = process_dist;
                      distChannelCnt++;
                      bCont = true;
                      numberOfItems = 0;
                      memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                      swap_endian((unsigned char *) &numberOfItems, 2);

                    }
                    if (strstr(szChannel, "VANG") == szChannel)
                    {
                      vangleCnt++;
                      task = process_vang;
                      bCont = true;
                      numberOfItems = 0;
                      memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                      swap_endian((unsigned char *) &numberOfItems, 2);

                      vang_vec.resize(numberOfItems);

                    }
                    if (strstr(szChannel, "RSSI") == szChannel)
                    {
                      task = process_rssi;
                      rssiCnt++;
                      bCont = true;
                      numberOfItems = 0;
                      // copy two byte value (unsigned short to  numberOfItems
                      memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);
                      swap_endian((unsigned char *) &numberOfItems, 2); // swap

                    }
                    if (bCont)
                    {
                      scaleFactor = 0.0;
                      scaleFactorOffset = 0.0;
                      startAngleDiv10000 = 0;
                      sizeOfSingleAngularStepDiv10000 = 0;
                      numberOfItems = 0;

                      memcpy(&scaleFactor, receiveBuffer + parseOff + 5, 4);
                      memcpy(&scaleFactorOffset, receiveBuffer + parseOff + 9, 4);
                      memcpy(&startAngleDiv10000, receiveBuffer + parseOff + 13, 4);
                      memcpy(&sizeOfSingleAngularStepDiv10000, receiveBuffer + parseOff + 17, 2);
                      memcpy(&numberOfItems, receiveBuffer + parseOff + 19, 2);


                      swap_endian((unsigned char *) &scaleFactor, 4);
                      swap_endian((unsigned char *) &scaleFactorOffset, 4);
                      swap_endian((unsigned char *) &startAngleDiv10000, 4);
                      swap_endian((unsigned char *) &sizeOfSingleAngularStepDiv10000, 2);
                      swap_endian((unsigned char *) &numberOfItems, 2);

                      if (processData)
                      {
                        unsigned short *data = (unsigned short *) (receiveBuffer + parseOff + 21);

                        unsigned char *swapPtr = (unsigned char *) data;
                        // copy RSSI-Values +2 for 16-bit values +1 for 8-bit value
                        for (int i = 0;
                             i < numberOfItems * processDataLenValuesInBytes; i += processDataLenValuesInBytes)
                        {
                          if (processDataLenValuesInBytes == 1)
                          {
                          }
                          else
                          {
                            unsigned char tmp;
                            tmp = swapPtr[i + 1];
                            swapPtr[i + 1] = swapPtr[i];
                            swapPtr[i] = tmp;
                          }
                        }
                        int idx = 0;

                        switch (task)
                        {

                          case process_dist:
                          {
                            startAngle = startAngleDiv10000 / 10000.00;
                            sizeOfSingleAngularStep = sizeOfSingleAngularStepDiv10000 / 10000.0;
                            sizeOfSingleAngularStep *= (M_PI / 180.0);
                            //TODO add ange offset argument - M_PI / 2
                            msg.angle_min = (startAngle / 180.0 * M_PI)+
                                            this->parser_->getCurrentParamPtr()->getScanAngleShift(); ;
                            msg.angle_increment = sizeOfSingleAngularStep;
                            msg.angle_max = msg.angle_min + (numberOfItems - 1) * msg.angle_increment;

                            float *rangePtr = NULL;

                            if (numberOfItems > 0)
                            {
                              rangePtr = &msg.ranges[0];
                            }
                            for (int i = 0; i < numberOfItems; i++)
                            {
                              idx = i + numberOfItems * (distChannelCnt - 1);
                              rangePtr[idx] = (float) data[i] * 0.001 * scaleFactor + scaleFactorOffset;
                            }
                          }
                            break;
                          case process_rssi:
                          {
                            // Das muss vom Protokoll abgeleitet werden. !!!

                            float *intensityPtr = NULL;

                            if (numberOfItems > 0)
                            {
                              intensityPtr = &msg.intensities[0];

                            }
                            for (int i = 0; i < numberOfItems; i++)
                            {
                              idx = i + numberOfItems * (rssiCnt - 1);
                              // we must select between 16 bit and 8 bit values
                              float rssiVal = 0.0;
                              if (processDataLenValuesInBytes == 2)
                              {
                                rssiVal = (float) data[i];
                              }
                              else
                              {
                                unsigned char *data8Ptr = (unsigned char *) data;
                                rssiVal = (float) data8Ptr[i];
                              }
                              intensityPtr[idx] = rssiVal * scaleFactor + scaleFactorOffset;
                            }
                          }
                            break;

                          case process_vang:
                            float *vangPtr = NULL;
                            if (numberOfItems > 0)
                            {
                              vangPtr = &vang_vec[0]; // much faster, with vang_vec[i] each time the size will be checked
                            }
                            for (int i = 0; i < numberOfItems; i++)
                            {
                              vangPtr[i] = (float) data[i] * scaleFactor + scaleFactorOffset;
                            }
                            break;
                        }
                      }
                      parseOff += 21 + processDataLenValuesInBytes * numberOfItems;


                    }
                    totalChannelCnt++;
                  } while (bCont);
                }
#endif

                elevAngle = elevAngleX200 / 200.0;
                scanFrequency = scanFrequencyX100 / 100.0;


              }
            }
          }


          parser_->checkScanTiming(msg.time_increment, msg.scan_time, msg.angle_increment, 0.00001f);

          success = ExitSuccess;
          // change Parsing Mode
          dataToProcess = false; // only one package allowed - no chaining
        }
        else // Parsing of Ascii-Ending of datagram
        {
          dstart = strchr(buffer_pos, 0x02);
          if (dstart != NULL)
          {
            dend = strchr(dstart + 1, 0x03);
          }
          if ((dstart != NULL) && (dend != NULL))
          {
            dataToProcess = true; // continue parasing
            dlength = dend - dstart;
            *dend = '\0';
            dstart++;
          }
          else
          {
            dataToProcess = false;
            break; //
          }

          if (dumpDbg)
          {
            {
              static int cnt = 0;
              char szFileName[255];

#ifdef _MSC_VER
              sprintf(szFileName, "c:\\temp\\dump%05d.txt", cnt);
#else
              sprintf(szFileName, "/tmp/dump%05d.txt", cnt);
#endif
              FILE *fout;
              fout = fopen(szFileName, "wb");
              fwrite(dstart, dlength, 1, fout);
              fclose(fout);
              cnt++;
            }
          }

          // HEADER of data followed by DIST1 ... DIST2 ... DIST3 .... RSSI1 ... RSSI2.... RSSI3...

          // <frameid>_<sign>00500_DIST[1|2|3]
          numEchos = 1;
          // numEchos contains number of echos (1..5)
          // _msg holds ALL data of all echos
          success = parser_->parse_datagram(dstart, dlength, config_, msg, numEchos, echoMask);
          publishPointCloud = true; // for MRS1000

          numValidEchos = 0;
          for (int i = 0; i < maxAllowedEchos; i++)
          {
            aiValidEchoIdx[i] = 0;
          }
        }


        int numOfLayers = parser_->getCurrentParamPtr()->getNumberOfLayers();

        if (success == ExitSuccess)
        {
          bool elevationPreCalculated = false;
          double elevationAngleDegree = 0.0;


          std::vector<float> rangeTmp = msg.ranges;  // copy all range value
          std::vector<float> intensityTmp = msg.intensities; // copy all intensity value

          int intensityTmpNum = intensityTmp.size();
          float *intensityTmpPtr = NULL;
          if (intensityTmpNum > 0)
          {
            intensityTmpPtr = &intensityTmp[0];
          }

          // Helpful: https://answers.ros.org/question/191265/pointcloud2-access-data/
          // https://gist.github.com/yuma-m/b5dcce1b515335c93ce8
          // Copy to pointcloud
          int layer = 0;
          int baseLayer = 0;
          bool useGivenElevationAngle = false;
          switch (numOfLayers)
          {
            case 1: // TIM571 etc.
              baseLayer = 0;
              break;
            case 4:// MRS1104
              baseLayer = -1;
              if (elevAngleX200 == 250)
              { layer = -1; }
              else if (elevAngleX200 == 0)
              { layer = 0; }
              else if (elevAngleX200 == -250)
              { layer = 1; }
              else if (elevAngleX200 == -500)
              { layer = 2; }
              elevationAngleDegree = this->parser_->getCurrentParamPtr()->getElevationDegreeResolution();
              elevationAngleDegree = elevationAngleDegree / 180.0 * M_PI;
              // 0.0436332 /*2.5 degrees*/;
              break;

              break;
            case 24: // Preparation for MRS6000
              baseLayer = -1;
#if 0
              layer = (msg.header.seq - (-2638)) / 125;
              layer = (23 - layer) - 1;
              elevationAngleDegree = this->parser_->getCurrentParamPtr()->getElevationDegreeResolution();
              elevationAngleDegree = elevationAngleDegree / 180.0 * M_PI;
#endif

              elevationPreCalculated = true;
              if (vang_vec.size() > 0)
              {
                useGivenElevationAngle = true;
              }
              break;
            default:
              assert(0);
              break; // unsupported

          }





          // XXX  - HIER MEHRERE SCANS publish, falls Mehrzielszenario läuft
          if (numEchos > 5)
          {
            RCLCPP_INFO(getMainNode()->get_logger(),"Too much echos");
          }
          else
          {

            int startOffset = 0;
            int endOffset = 0;
            int echoPartNum = msg.ranges.size() / numEchos;
            for (int i = 0; i < numEchos; i++)
            {

              bool sendMsg = false;
              if ((echoMask & (1 << i)) & outputChannelFlagId)
              {
                aiValidEchoIdx[numValidEchos] = i; // save index
                numValidEchos++;
                sendMsg = true;
              }
              startOffset = i * echoPartNum;
              endOffset = (i + 1) * echoPartNum;

              msg.ranges.clear();
              msg.intensities.clear();
              msg.ranges = std::vector<float>(
                  rangeTmp.begin() + startOffset,
                  rangeTmp.begin() + endOffset);
              // check also for MRS1104
              if (endOffset <= intensityTmp.size() && (intensityTmp.size() > 0))
              {
                msg.intensities = std::vector<float>(
                    intensityTmp.begin() + startOffset,
                    intensityTmp.begin() + endOffset);
              }
              else
              {
                msg.intensities.resize(echoPartNum); // fill with zeros
              }
              // msg.header.frame_id = ""
              {
                // numEchos
                char szTmp[255] = {0};
                if (this->parser_->getCurrentParamPtr()->getNumberOfLayers() > 1)
                {
                  const char *cpFrameId = config_.frame_id.c_str();
#if 0
                  sprintf(szTmp, "%s_%+04d_DIST%d", cpFrameId, msg.header.seq, i + 1);
#else // experimental
                  char szSignName[10] = {0};
#if 0
                  if ((int)msg.header.seq < 0)
                  {
                    strcpy(szSignName, "NEG");
                  }
                  else
                  {
                    strcpy(szSignName, "POS");

                  }
#endif
                  int seqDummy = 0;
                  // abs((int)msg.header.seq)
                  sprintf(szTmp, "%s_%s_%03d_DIST%d", cpFrameId, szSignName, seqDummy, i + 1);
#endif
                }
                else
                {
                  strcpy(szTmp, config_.frame_id.c_str());
                }

                //msg.header.frame_id = std::string(szTmp);
                //TODO clean this up!
                msg.header.frame_id = this->config_.frame_id;
                // Hector slam can only process ONE valid frame id.
                std::string echoForSlam = "";
                bool slamBundle = false;
                if (echoForSlam.length() > 0)
                {
                  if (slamBundle)
                  {
                    // try to map first echos to horizontal layers.
                    if (i == 0)
                    {
                      // first echo
                      msg.header.frame_id = echoForSlam;
                      strcpy(szTmp, echoForSlam.c_str());  //
                      if (elevationAngleInRad != 0.0)
                      {
                        float cosVal = cos(elevationAngleInRad);
                        int rangeNum = msg.ranges.size();
                        for (int j = 0; j < rangeNum; j++)
                        {
                          msg.ranges[j] *= cosVal;
                        }
                      }
                    }
                  }

                  if (echoForSlam.compare(szTmp) == 0)
                  {
                    sendMsg = true;
                  }
                  else
                  {
                    sendMsg = false;
                  }
                }

              }
#ifndef _MSC_VER
              if (numOfLayers > 4)
              {
                sendMsg = false; // too many layers for publish as scan message. Only pointcloud messages will be pub.
              }
              if (sendMsg & outputChannelFlagId)  // publish only configured channels - workaround for cfg-bug MRS1104
              {
                msg.header.stamp = recvTimeStamp;

                //msg.range_min = 0.0f;
                //msg.range_max = 100.0f;
                pub_->publish(msg);
              }
#else
              RCLCPP_INFO(node->get_logger(),"MSG received...");
#endif
            }
          }


          if (publishPointCloud == true)
          {
            const int numChannels = 4; // x y z i (for intensity)


						cloud_.header.stamp = recvTimeStamp;
            cloud_.header.frame_id = config_.frame_id;
//						cloud_.header.seq = 0;
            cloud_.height = numOfLayers * numValidEchos; // due to multi echo multiplied by num. of layers
            cloud_.width = msg.ranges.size();
            cloud_.is_bigendian = false;
            cloud_.is_dense = true;
            cloud_.point_step = numChannels * sizeof(float);
            cloud_.row_step = cloud_.point_step * cloud_.width;
            cloud_.fields.resize(numChannels);
            for (int i = 0; i < numChannels; i++)
            {
              std::string channelId[] = {"x", "y", "z", "intensity"};
              cloud_.fields[i].name = channelId[i];
              cloud_.fields[i].offset = i * sizeof(float);
              cloud_.fields[i].count = 1;
              cloud_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            }

            cloud_.data.resize(cloud_.row_step * cloud_.height);

            unsigned char *cloudDataPtr = &(cloud_.data[0]);

            for (size_t iEcho = 0; iEcho < numValidEchos; iEcho++)
            {
              std::vector<float> cosAlphaTable;
              std::vector<float> sinAlphaTable;
              //TODO add ange offset argument - M_PI / 2
              double angshift= this->parser_->getCurrentParamPtr()->getScanAngleShift();
              float angle = config_.min_ang+angshift;
              int rangeNum = rangeTmp.size() / numEchos;

              cosAlphaTable.resize(rangeNum);
              sinAlphaTable.resize(rangeNum);

              float *cosAlphaTablePtr = &cosAlphaTable[0];
              float *sinAlphaTablePtr = &sinAlphaTable[0];

                float *vangPtr = NULL;
                float *rangeTmpPtr = &rangeTmp[0];
                if (vang_vec.size() > 0)
                {
                    vangPtr = &vang_vec[0];
                }
              for (size_t i = 0; i < rangeNum; i++)
              {
                geometry_msgs::msg::Point32 point;
                float range_meter = rangeTmpPtr[iEcho * rangeNum + i];
                float phi = angle; // azimuth angle
                float alpha = 0.0;  // elevation angle

                if (useGivenElevationAngle) // FOR MRS6124
                {
                  alpha = -vangPtr[i] * deg2rad_const;
                }
                else
                {
                  if (elevationPreCalculated) // FOR MRS6124 without VANGL
                  {
                    alpha = elevationAngleInRad;
                  }
                  else
                  {
                    alpha = layer * elevationAngleDegree; // for MRS1104
                  }
                }

                if (iEcho == 0)
                {
                  cosAlphaTablePtr[i] = cos(alpha);
                  sinAlphaTablePtr[i] = sin(alpha);
                }
                // Thanks to Sebastian Pütz <spuetz@uos.de> for his hint
                point.x = range_meter * cosAlphaTablePtr[i] * cos(phi);
                point.y = range_meter * cosAlphaTablePtr[i] * sin(phi);
                point.z = range_meter * sinAlphaTablePtr[i];

                //	cloud_.points[(layer - baseLayer) * msg.ranges.size() + i] = point;

                long adroff = i * (numChannels * (int) sizeof(float)) + (layer - baseLayer) * cloud_.row_step;
                adroff += iEcho * cloud_.row_step * numOfLayers;
                unsigned char *ptr = cloudDataPtr + adroff;

                float intensity = 0.0;
                if (config_.intensity)
                {
                  int intensityIndex = aiValidEchoIdx[iEcho] * rangeNum + i;
                  // intensity values available??
                  if (intensityIndex < intensityTmpNum)
                  {
                    intensity = intensityTmpPtr[intensityIndex];
                  }
                }
                float dataArr[4];
                dataArr[0] = point.x;
                dataArr[1] = point.y;
                dataArr[2] = point.z;
                dataArr[3] = intensity;
                memcpy(ptr + 0, dataArr, 4 * sizeof(float));

                angle += msg.angle_increment;
              }
              // Publish
              static int cnt = 0;
              int layerOff = (layer - baseLayer);

            }
            // if ( (msg.header.seq == 0) || (layerOff == 0)) // FIXEN!!!!
#if 1
            if ((seqId == 0) || (seqId == 237))
// if ((msg.header.seq == 0) || (msg.header.seq == 237))
            {
              // Following cases are interesting:
              // LMS5xx: seq is always 0 -> publish every scan
              // MRS1104: Every 4th-Layer is 0 -> publish pointcloud every 4th layer message
              // LMS1104: Publish every layer. The timing for the LMS1104 seems to be:
              //          Every 67 ms receiving of a new scan message
              //          Scan message contains 367 measurements
              //          angle increment is 0.75° (yields 274,5° covery -> OK)
              // MRS6124: Publish very 24th layer at the layer = 237 , MRS6124 contains no sequence with seq 0
              ;
#ifndef _MSC_VER
              cloud_pub_->publish(cloud_);
#else
              RCLCPP_INFO(node->get_logger(),"PUBLISH:\n");
#endif
            }
#endif
          }
        }
        // Start Point
        buffer_pos = dend + 1;
      } // end of while loop

      return ExitSuccess; // return success to continue looping
    }
    return ExitSuccess;
  }


  /*!
  \brief check angle setting in the config and adjust the min_ang to the max_ang if min_ang greater than max_ax
  */
  void SickScanCommon::check_angle_range(SickScanConfig &conf)
  {
    if (conf.min_ang > conf.max_ang)
    {
      printf("Maximum angle must be greater than minimum angle. Adjusting >min_ang<.");
      conf.min_ang = conf.max_ang;
    }
  }

  /*!
  \brief updating configuration
  \param new_config: Pointer to new configuration
  \param level (not used - should be removed)
  */
  void SickScanCommon::update_config(sick_scan::SickScanConfig &new_config, uint32_t level)
  {
    check_angle_range(new_config);
    config_ = new_config;
  }

  /*!
  \brief Convert ASCII-message to Binary-message
  \param requestAscii holds ASCII-encoded command
  \param requestBinary hold binary command as vector of unsigned char
  \return success = 0
  */
  int SickScanCommon::convertAscii2BinaryCmd(const char *requestAscii, std::vector<unsigned char> *requestBinary)
  {
    requestBinary->clear();
    if (requestAscii == NULL)
    {
      return (-1);
    }
    int cmdLen = strlen(requestAscii);
    if (cmdLen == 0)
    {
      return (-1);
    }
    if (requestAscii[0] != 0x02)
    {
      return (-1);
    }
    if (requestAscii[cmdLen - 1] != 0x03)
    {
      return (-1);
    }
    // Here we know, that the ascii format is correctly framed with <stx> .. <etx>
    for (int i = 0; i < 4; i++)
    {
      requestBinary->push_back(0x02);
    }

    for (int i = 0; i < 4; i++) // Puffer for Length identifier
    {
      requestBinary->push_back(0x00);
    }

    unsigned long msgLen = cmdLen - 2; // without stx and etx

    // special handling for the following commands
    // due to higher number of command arguments
    std::string keyWord0 = "sMN SetAccessMode";
    std::string keyWord1 = "sWN FREchoFilter";
    std::string keyWord2 = "sEN LMDscandata";
    std::string keyWord3 = "sWN LMDscandatacfg";
    std::string keyWord4 = "sWN SetActiveApplications";
    std::string keyWord5 = "sEN IMUData";
    std::string keyWord6 = "sWN EIIpAddr";
    std::string keyWord7 = "sMN mLMPsetscancfg";
    std::string keyWord8 = "sWN TSCTCupdatetime";
    std::string keyWord9 = "sWN TSCTCSrvAddr";
    std::string keyWord10 = "sWN LICencres";
    std::string keyWord11 = "sWN LFPmeanfilter";

    //BBB

    std::string cmdAscii = requestAscii;


    int copyUntilSpaceCnt = 2;
    int spaceCnt = 0;
    char hexStr[255] = {0};
    int level = 0;
    unsigned char buffer[255];
    int bufferLen = 0;
    if (cmdAscii.find(keyWord0) != std::string::npos) // SetAccessMode
    {
      copyUntilSpaceCnt = 2;
      int keyWord0Len = keyWord0.length();
      sscanf(requestAscii + keyWord0Len + 1, " %d %s", &level, hexStr);
      buffer[0] = (unsigned char) (0xFF & level);
      bufferLen = 1;
      char hexTmp[3] = {0};
      for (int i = 0; i < 4; i++)
      {
        int val;
        hexTmp[0] = hexStr[i * 2];
        hexTmp[1] = hexStr[i * 2 + 1];
        hexTmp[2] = 0x00;
        sscanf(hexTmp, "%x", &val);
        buffer[i + 1] = (unsigned char) (0xFF & val);
        bufferLen++;
      }
    }
    if (cmdAscii.find(keyWord1) != std::string::npos)
    {
      int echoCodeNumber = 0;
      int keyWord1Len = keyWord1.length();
      sscanf(requestAscii + keyWord1Len + 1, " %d", &echoCodeNumber);
      buffer[0] = (unsigned char) (0xFF & echoCodeNumber);
      bufferLen = 1;
    }
    if (cmdAscii.find(keyWord2) != std::string::npos)
    {
      int scanDataStatus = 0;
      int keyWord2Len = keyWord2.length();
      sscanf(requestAscii + keyWord2Len + 1, " %d", &scanDataStatus);
      buffer[0] = (unsigned char) (0xFF & scanDataStatus);
      bufferLen = 1;
    }

    if (cmdAscii.find(keyWord3) != std::string::npos)
    {
      int scanDataStatus = 0;
      int keyWord3Len = keyWord3.length();
      int dummyArr[12] = {0};
      //sWN LMDscandatacfg %02d 00 %d %d 0 0 %02d 0 0 0 1 1\x03"
      int sscanfresult = sscanf(requestAscii + keyWord3Len + 1, " %d %d %d %d %d %d %d %d %d %d %d %d",
                                &dummyArr[0], // Data Channel Idx LSB
                                &dummyArr[1], // Data Channel Idx MSB
                                &dummyArr[2], // Remission
                                &dummyArr[3], // Remission data format
                                &dummyArr[4], // Unit
                                &dummyArr[5], // Encoder Setting LSB
                                &dummyArr[6], // Encoder Setting MSB
                                &dummyArr[7], // Position
                                &dummyArr[8], // Send Name
                                &dummyArr[9], // Send Comment
                                &dummyArr[10], // Time information
                                &dummyArr[11]); // n-th Scan (packed - not sent as single byte sequence) !!!
      if (1 < sscanfresult)
      {

        for (int i = 0; i < 13; i++)
        {
          buffer[i] = 0x00;
        }
        buffer[0] = (unsigned char) (0xFF & dummyArr[0]);  //Data Channel 2 Bytes
        buffer[1] = (unsigned char) (0xFF & dummyArr[1]);; // MSB of Data Channel (here Little Endian!!)
        buffer[2] = (unsigned char) (0xFF & dummyArr[2]);  // Remission
        buffer[3] = (unsigned char) (0xFF & dummyArr[3]);  // Remission data format 0=8 bit 1= 16 bit
        buffer[4] = (unsigned char) (0xFF & dummyArr[4]);  //Unit of remission data
        buffer[5] = (unsigned char) (0xFF & dummyArr[5]);  //encoder Data LSB
        buffer[6] = (unsigned char) (0xFF & dummyArr[6]);  //encoder Data MSB
        buffer[7] = (unsigned char) (0xFF & dummyArr[7]);  // Position
        buffer[8] = (unsigned char) (0xFF & dummyArr[8]);  // Send Scanner Name
        buffer[9] = (unsigned char) (0xFF & dummyArr[9]);  // Comment
        buffer[10] = (unsigned char) (0xFF & dummyArr[10]);  // Time information
        buffer[11] = (unsigned char) (0xFF & (dummyArr[11] >> 8));  // BIG Endian High Byte nth-Scan
        buffer[12] = (unsigned char) (0xFF & (dummyArr[11] >> 0));  // BIG Endian Low Byte nth-Scan
        bufferLen = 13;

      }

    }

    if (cmdAscii.find(keyWord4) != std::string::npos)
    {
      char tmpStr[1024] = {0};
      char szApplStr[255] = {0};
      int keyWord4Len = keyWord4.length();
      int scanDataStatus = 0;
      int dummy0, dummy1;
      strcpy(tmpStr, requestAscii + keyWord4Len + 2);
      sscanf(tmpStr, "%d %s %d", &dummy0, szApplStr, &dummy1);
      // rebuild string
      buffer[0] = 0x00;
      buffer[1] = dummy0 ? 0x01 : 0x00;
      for (int ii = 0; ii < 4; ii++)
      {
        buffer[2 + ii] = szApplStr[ii]; // idx: 1,2,3,4
      }
      buffer[6] = dummy1 ? 0x01 : 0x00;
      bufferLen = 7;
    }

    if (cmdAscii.find(keyWord5) != std::string::npos)
    {
      int imuSetStatus = 0;
      int keyWord5Len = keyWord5.length();
      sscanf(requestAscii + keyWord5Len + 1, " %d", &imuSetStatus);
      buffer[0] = (unsigned char) (0xFF & imuSetStatus);
      bufferLen = 1;
    }

    if (cmdAscii.find(keyWord6) != std::string::npos)
    {
      int adrPartArr[4];
      int imuSetStatus = 0;
      int keyWord6Len = keyWord6.length();
      sscanf(requestAscii + keyWord6Len + 1, " %x %x %x %x", &(adrPartArr[0]), &(adrPartArr[1]), &(adrPartArr[2]),
             &(adrPartArr[3]));
      buffer[0] = (unsigned char) (0xFF & adrPartArr[0]);
      buffer[1] = (unsigned char) (0xFF & adrPartArr[1]);
      buffer[2] = (unsigned char) (0xFF & adrPartArr[2]);
      buffer[3] = (unsigned char) (0xFF & adrPartArr[3]);
      bufferLen = 4;
    }
    //\x02sMN mLMPsetscancfg %d 1 %d 0 0\x03";
    //02 02 02 02 00 00 00 25 73 4D 4E 20 6D 4C 4D 50 73 65 74 73 63 61 6E 63 66 67 20
    // 00 00 13 88 4byte freq
    // 00 01 2 byte sectors always 1
    // 00 00 13 88  ang_res
    // FF F9 22 30 sector start always 0
    // 00 22 55 10 sector stop  always 0
    // 21
    if (cmdAscii.find(keyWord7) != std::string::npos)
    {
      if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_NAV_3XX_NAME) != 0)
      {
        {
          bufferLen = 18;
          for (int i = 0; i < bufferLen; i++)
          {
            unsigned char uch = 0x00;
            switch (i)
            {
              case 5:
                uch = 0x01;
                break;
            }
            buffer[i] = uch;
          }
          char tmpStr[1024] = {0};
          char szApplStr[255] = {0};
          int keyWord7Len = keyWord7.length();
          int scanDataStatus = 0;
          int dummy0, dummy1;
          strcpy(tmpStr, requestAscii + keyWord7Len + 2);
          sscanf(tmpStr, "%d 1 %d", &dummy0, &dummy1);

          buffer[0] = (unsigned char) (0xFF & (dummy0 >> 24));
          buffer[1] = (unsigned char) (0xFF & (dummy0 >> 16));
          buffer[2] = (unsigned char) (0xFF & (dummy0 >> 8));
          buffer[3] = (unsigned char) (0xFF & (dummy0 >> 0));


          buffer[6] = (unsigned char) (0xFF & (dummy1 >> 24));
          buffer[7] = (unsigned char) (0xFF & (dummy1 >> 16));
          buffer[8] = (unsigned char) (0xFF & (dummy1 >> 8));
          buffer[9] = (unsigned char) (0xFF & (dummy1 >> 0));
        }
      }
      else
      {
        int keyWord3Len = keyWord7.length();
        int dummyArr[14] = {0};
        if (14 == sscanf(requestAscii + keyWord3Len + 1, " %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                         &dummyArr[0], &dummyArr[1], &dummyArr[2],
                         &dummyArr[3], &dummyArr[4], &dummyArr[5],
                         &dummyArr[6], &dummyArr[7], &dummyArr[8],
                         &dummyArr[9], &dummyArr[10], &dummyArr[11], &dummyArr[12], &dummyArr[13]))
        {
          for (int i = 0; i < 54; i++)
          {
            buffer[i] = 0x00;
          }
          int targetPosArr[] = {0, 4, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 50, 54};
          int numElem = (sizeof(targetPosArr) / sizeof(targetPosArr[0])) - 1;
          for (int i = 0; i < numElem; i++)
          {
            int lenOfBytesToRead = targetPosArr[i + 1] - targetPosArr[i];
            int adrPos = targetPosArr[i];
            unsigned char *destPtr = buffer + adrPos;
            memcpy(destPtr, &(dummyArr[i]), lenOfBytesToRead);
            swap_endian(destPtr, lenOfBytesToRead);
          }
          bufferLen = targetPosArr[numElem];
          /*
           * 00 00 03 20 00 01
  00 00 09 C4 00 00 00 00 00 36 EE 80 00 00 09 C4 00 00 00 00 00 00 00 00 00 00 09 C4 00 00 00 00 00
  00 00 00 00 00 09 C4 00 00 00 00 00 00 00 00 00 00 09 C4 00 00 00 00 00 00 00 00 E4
           */

        }
      }

    }
    if (cmdAscii.find(keyWord8) != std::string::npos)
    {
      uint32_t updatetime = 0;
      int keyWord8Len = keyWord8.length();
      sscanf(requestAscii + keyWord8Len + 1, " %d", &updatetime);
      buffer[0] = (unsigned char) (0xFF & (updatetime >> 24));
      buffer[1] = (unsigned char) (0xFF & (updatetime >> 16));
      buffer[2] = (unsigned char) (0xFF & (updatetime >> 8));
      buffer[3] = (unsigned char) (0xFF & (updatetime >> 0));
      bufferLen = 4;
    }
    if (cmdAscii.find(keyWord9) != std::string::npos)
    {
      int adrPartArr[4];
      int imuSetStatus = 0;
      int keyWord9Len = keyWord9.length();
      sscanf(requestAscii + keyWord9Len + 1, " %x %x %x %x", &(adrPartArr[0]), &(adrPartArr[1]), &(adrPartArr[2]),
             &(adrPartArr[3]));
      buffer[0] = (unsigned char) (0xFF & adrPartArr[0]);
      buffer[1] = (unsigned char) (0xFF & adrPartArr[1]);
      buffer[2] = (unsigned char) (0xFF & adrPartArr[2]);
      buffer[3] = (unsigned char) (0xFF & adrPartArr[3]);
      bufferLen = 4;
    }
    if (cmdAscii.find(keyWord10) != std::string::npos)
    {
      float EncResolution = 0;
      bufferLen = 4;
      int keyWord10Len = keyWord10.length();
      sscanf(requestAscii + keyWord10Len + 1, " %f", &EncResolution);
      memcpy(buffer, &EncResolution, bufferLen);
      swap_endian(buffer, bufferLen);

    }
    if (cmdAscii.find(keyWord11) != std::string::npos)
    {
      char tmpStr[1024] = {0};
      char szApplStr[255] = {0};
      int keyWord11Len = keyWord11.length();
      int dummy0, dummy1,dummy2;
      strcpy(tmpStr, requestAscii + keyWord11Len + 2);
      sscanf(tmpStr, "%d %d %d", &dummy0, &dummy1, &dummy2);
      // rebuild string
      buffer[0] = dummy0 ? 0x01 : 0x00;
      buffer[1] =dummy1/256;//
      buffer[2] =dummy1%256;//
      buffer[3] =dummy2;
      bufferLen = 4;
    }
    // copy base command string to buffer
    bool switchDoBinaryData = false;
    for (int i = 1; i <= (int) (msgLen); i++)  // STX DATA ETX --> 0 1 2
    {
      char c = requestAscii[i];
      if (switchDoBinaryData == true)
      {
        if (0 == bufferLen)  // no keyword handling before this point
        {
          if (c >= '0' && c <= '9')  // standard data format expected - only one digit
          {
            c -= '0';              // convert ASCII-digit to hex-digit
          }                          // 48dez to 00dez and so on.
        }
        else
        {
          break;
        }
      }
      requestBinary->push_back(c);
      if (requestAscii[i] == 0x20) // space
      {
        spaceCnt++;
        if (spaceCnt >= copyUntilSpaceCnt)
        {
          switchDoBinaryData = true;
        }
      }

    }
    // if there are already process bytes (due to special key word handling)
    // copy these data bytes to the buffer
    for (int i = 0; i < bufferLen; i++) // append variable data
    {
      requestBinary->push_back(buffer[i]);
    }

    msgLen = requestBinary->size();
    msgLen -= 8;
    for (int i = 0; i < 4; i++)
    {
      unsigned char bigEndianLen = msgLen & (0xFF << (3 - i) * 8);
      (*requestBinary)[i + 4] = ((unsigned char) (bigEndianLen)); // HIER WEITERMACHEN!!!!
    }
    unsigned char xorVal = 0x00;
    xorVal = sick_crc8((unsigned char *) (&((*requestBinary)[8])), requestBinary->size() - 8);
    requestBinary->push_back(xorVal);
#if 0
    for (int i = 0; i < requestBinary->size(); i++)
    {
      unsigned char c = (*requestBinary)[i];
      printf("[%c]%02x ", (c < ' ') ? '.' : c, c) ;
    }
    printf("\n");
#endif
    return (0);

  };


  /*!
  \brief checks the current protocol type and gives information about necessary change

  \param useBinaryCmdNow Input/Output: Holds information about current protocol
  \return true, if protocol type is already the right one
  */
  bool SickScanCommon::checkForProtocolChangeAndMaybeReconnect(bool &useBinaryCmdNow)
  {
    bool retValue = true;
    bool shouldUseBinary = this->parser_->getCurrentParamPtr()->getUseBinaryProtocol();
    if (shouldUseBinary == useBinaryCmdNow)
    {
      retValue = true;  // !!!! CHANGE ONLY FOR TEST!!!!!
    }
    else
    {
      /*
            // we must reconnect and set the new protocoltype
            int iRet = this->close_device();
            printf("SOPAS - Close and reconnected to scanner due to protocol change and wait 15 sec. ");
            printf("SOPAS - Changing from %s to %s\n", shouldUseBinary ? "ASCII" : "BINARY", shouldUseBinary ? "BINARY" : "ASCII");
            // Wait a few seconds after rebooting
            ros::Duration(15.0).sleep();

            iRet = this->init_device();
            */
      if (shouldUseBinary == true)
      {
        this->setProtocolType(CoLa_B);
      }
      else
      {
        this->setProtocolType(CoLa_A);
      }

      useBinaryCmdNow = shouldUseBinary;
      retValue = false;
    }
    return (retValue);
  }

  void SickScanCommon::setSensorIsRadar(bool _isRadar)
  {
    sensorIsRadar = _isRadar;
  }

  bool SickScanCommon::getSensorIsRadar(void)
  {
    return (sensorIsRadar);
  }
/*!
\brief Universal swapping function
\param ptr: Pointer to datablock
\param numBytes : size of variable in bytes
*/
  void swap_endian(unsigned char *ptr, int numBytes)
  {
    unsigned char *buf = (ptr);
    unsigned char tmpChar;
    for (int i = 0; i < numBytes / 2; i++)
    {
      tmpChar = buf[numBytes - 1 - i];
      buf[numBytes - 1 - i] = buf[i];
      buf[i] = tmpChar;
    }
  }

  // SopasProtocol m_protocolId;

} /* namespace sick_scan */
