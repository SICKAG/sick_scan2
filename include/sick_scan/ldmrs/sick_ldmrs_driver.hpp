/*
 * Copyright (C) 2015, DFKI GmbH
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
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 *  Created on: 23.11.2015
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *
 *  Modified and ported to ROS2: 02.10.2020 by Ing.-Buero Dr. Michael Lehning, Hildesheim
 */

#ifndef SICK_LDMRS800001S01_H_
#define SICK_LDMRS800001S01_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <cfloat>
#include <chrono>
#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sick_scan/ldmrs/sick_ldmrs_point_type.h>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

// #include <dynamic_reconfigure/server.h>
// #include <sick_ldmrs_driver/SickLDMRSDriverConfig.h>
#include "sick_scan/ldmrs/sick_ldmrs_config.hpp"

#include <sick_ldmrs/manager.hpp>
#include <sick_ldmrs/application/BasicApplication.hpp>
#include <sick_ldmrs/datatypes/Object.hpp>
#include <sick_scan2/msg/sick_ldmrs_object_array.hpp>
//#include <sick_scan/ldmrs/msg/object_array.hpp>

namespace sick_ldmrs_driver
{

typedef pcl::PointCloud<sick_ldmrs_msgs::SICK_LDMRS_Point> PointCloud;

// Adapter to combine publisher for sensor_msgs::msg::PointCloud2 and diagnostic_msgs::msg::DiagnosticArray.
// see https://github.com/ros/diagnostics/blob/eloquent/diagnostic_updater/include/diagnostic_updater/publisher.hpp
// and https://github.com/ros/diagnostics/issues/164
template <class DiagnosedPublisherT> class DiagnosedPublishAdapter : public diagnostic_updater::TopicDiagnostic
{
public:
  template <typename PublisherType> DiagnosedPublishAdapter(PublisherType publisher, diagnostic_updater::Updater & diag,
    const diagnostic_updater::FrequencyStatusParam & freq, const diagnostic_updater::TimeStampStatusParam & stamp)
  : diagnostic_updater::TopicDiagnostic(publisher->get_topic_name(), diag, freq, stamp), publisher_(publisher)
  {
  }
  virtual ~DiagnosedPublishAdapter()
  {
  }
  template <typename MessageType> void publish(const std::shared_ptr<MessageType> & message)
  {
    if(message)
      publish(*message);
  }
  template <typename MessageType> void publish(const MessageType & message)
  {
    tick(message.header.stamp);
    publisher_->publish(message);
  }
protected:
  DiagnosedPublisherT publisher_;
};

class SickLDMRS : public application::BasicApplication
{
public:
  SickLDMRS(rclcpp::Node::SharedPtr nh, Manager* manager, boost::shared_ptr<diagnostic_updater::Updater> diagnostics);
  virtual ~SickLDMRS();
  void init();
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void validate_config(SickLDMRSDriverConfig &conf);
  void update_config(SickLDMRSDriverConfig &new_config, uint32_t level = 0);
  rcl_interfaces::msg::SetParametersResult update_config_cb(const std::vector<rclcpp::Parameter> &parameters);
  void pubObjects(datatypes::ObjectList &objects);

protected:
  boost::shared_ptr<diagnostic_updater::Updater> diagnostics_;
  void setData(BasicData& data);  // Callback for new data from the manager (scans etc.)
  void validate_flexres_resolution(int &res);
  void validate_flexres_start_angle(double &angle1, double &angle2);
  bool isUpsideDown();
  void printFlexResError();
  std::string flexres_err_to_string(const UINT32 code) const;

private:

  // ROS
  rclcpp::Node::SharedPtr nh_;                                                 // ros::NodeHandle nh_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;            // ros::Publisher pub_;
  // rclcpp::Publisher<sick_ldmrs_msgs::msg::ObjectArray>::SharedPtr object_pub_; // ros::Publisher object_pub_;
  rclcpp::Publisher<sick_scan2::msg::SickLdmrsObjectArray>::SharedPtr object_pub_; // ros::Publisher object_pub_;
  // Diagnostics
  DiagnosedPublishAdapter<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>* diagnosticPub_; // diagnostic_updater::DiagnosedPublisher<sensor_msgs::PointCloud2>* diagnosticPub_;

  // Dynamic Reconfigure
  SickLDMRSDriverConfig config_;
  // dynamic_reconfigure::Server<SickLDMRSDriverConfig> dynamic_reconfigure_server_;

  // sick_ldmrs library objects
  Manager* manager_;

  // Expected scan frequency. Must be a member variable for access by diagnostics.
  double expected_frequency_;

  bool initialized_;
};

} /* namespace sick_ldmrs_driver */

#endif /* SICK_LDMRS800001S01_H_ */
