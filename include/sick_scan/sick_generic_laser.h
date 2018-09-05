#ifndef SICK_GENERIC_LASER_H
#define SICK_GENERIC_LASER_H
/*
 *
 */
#ifndef _MSC_VER
#endif
#include <sick_scan/sick_scan_common_tcp.h>

#include "rclcpp/rclcpp.hpp"
void setMainNode(std::shared_ptr<rclcpp::Node> _tmpNode);
std::shared_ptr<rclcpp::Node> getMainNode(void);
int mainGenericLaser(int argc, char **argv, std::string scannerName);
#endif

