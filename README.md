# sick_scan2
This stack provides a ROS2 driver for the MRS1000 SICK lidar sensor
## Table of contents

- [Supported Hardware](#supported-hardware)
- [Start node](#start-node)
- [Bugs and feature requests](#bugs-and-feature-requests)
- [Timestamping](doc/timestamping.md)
- [Creators](#creators)

This stack provides a ROS2 driver for the SICK laser scanners
mentioned in the following list.


## Supported Hardware

This driver should work with just MRS1000

| **device name**    |  **part no.**                                                                                                                | **description**                                | **tested?**     |
|--------------------|------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|:---------------:|
| MRS1104            | [1081208](https://www.sick.com/sg/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs1000/mrs1104c-111011/p/p495044)         | 4 layer max. range: 64 m, ang. resol. 0.25 [deg] hor., 2.50 [deg] ver.                                         | ✔ [development]|
|                    |                                                                                                                                  | Scan-Rate: 50 Hz, 4x12.5 Hz            |                 |


##  Start Node

See quick start for further hints.

## Sopas Mode
This driver supports both COLA-B (binary) and COLA-A (ASCII) communication with the laser scanner. It is necessary to changeto the Binary mode to work with ROS 2. Since this mode generates less network traffic.
If the communication mode set in the scanner memory is different from that used by the driver, the scanner's communication mode is changed. This requires a restart of the TCP-IP connection, which can extend the start time by up to 30 seconds.
There are two ways to prevent this:
1. [Recommended] Set the communication mode with the SOPAS ET software to binary and save this setting in the scanner's EEPROM.
2. Use the parameter "use_binary_protocol" to overwrite the default settings of the driver.
3. Setting "use_binary_protocol" to "False" activates COLA-A and disables COLA-B (default)

## Bugs and feature requests

- Stability issues: Driver is experimental and brand new
- Sopas protocol mapping:
-- All scanners: COLA-B (Binary)
- Software should be further tested and documented

## Troubleshooting

1. Check Scanner IP by using fping or SOPAS ET under Windows
2. Check Ethernet connection to scanner with netcat e.g. ```nc -z -v -w5 $SCANNERIPADDRESS 2112```.
   For further details about setting up the correct ip settings see [IP configuration](doc/ipconfig/ipconfig.md)
3. View node startup output wether the IP connection could be established
4. Check the scanner status using the LEDs on the device. The LED codes are described in the above mentioned operation manuals.
5. Further testing and troubleshooting informations can found in the file test/readme_testplan.txt
6. If you stop the scanner in your debugging IDE or by other hard interruption (like Ctrl-C), you must wait until 60 sec. before
   the scanner is up and running again. During this time the MRS6124 reconnects twice.
   If you do not wait this waiting time you could see one of the following messages:
   * TCP connection error
   * Error-Message 0x0d
7. Amplitude values in rviz: If you see only one color in rviz try the following:
   Set the min/max-Range of intensity display in the range [0...200] and switch on the intensity flag in the lauch file  
8. In case of network problems check your own ip address and the ip address of your laser scanner (by using SOPAS ET).
   * List of own IP-addresses: ifconfig|grep "inet addr"
   * Try to ping scanner ip address (used in launch file)
9. If the driver stops during init phase please stop the driver with ctrl-c and restart (could be caused due to protocol ASCII/Binary cola-dialect).

## Support

* In case of technical support please open a new issue (contact in France: jeffrey.yannou@sick.fr). For optimal support, add the following information to your request:
 1. Scanner model name,
 2. Ros node startup log,
 3. Sopas file of your scanner configuration.
  The instructions at http://sickusablog.com/create-and-download-a-sopas-file/ show how to create the Sopas file.
* In case of application support please use [https://supportportal.sick.com ](https://supportportal.sick.com).
* Issue Handling: Issues, for which no reply was received from the questioner for more than 7 days,						
  are closed by us because we assume that the user has solved the problem.


## Installation

In the following instructions, replace `<rosdistro>` with the name of your ROS distribution (e.g., `foxy`).

### From source

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/sick_scan_ws/src/
cd ~/sick_scan_ws/src/
git clone https://github.com/SICKAG/sick_scan2.git
cd ..
colcon build
source ~/sick_scan_ws/install/setup.bash
```

## Quick Start

```bash
cd ~/sick_scan_ws
source ./install/setup.bash
```
Attention: Replace the ip address for "__hostname" with your scanner ip address.
Default ip address of scanner is 192.168.0.1.
In this example we use the ip address 192.168.11.33, but it is possible to change.

Via the program argument __frame_id the frame of the laserscan messages can be changed. Default is "cloud".

For MRS1104:
```
ros2 launch sick_scan2 sick_mrs_1xxx.launch.py
```

## Unit tests

For a quick unit test after installation without the sensor hardware, a test server is provided to simulate a scanner. 
The test server generates scan data examples and responds to command requests. 
Please note, that this test server does not emulate a Lidar sensor. It just sends some simple scan data and response messages to a tcp client.
It can be used for a quick unit test after build and install.

For a unit test, run the following commands in different terminals:

For other scanners supported by sick_scan2, replace sick_mrs_1xxx.yaml, and run the following commands:

```
cd ~/sick_scan_ws
source ./install/setup.bash
ros2 run sick_scan2 test_server --ros-args --params-file src/sick_scan2/tools/test_server/config/test_server_cola.yaml
ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_mrs_1xxx.yaml -p "hostname:=192.168.11.33" -p "port:=2112" -p "sw_pll_only_publish:=false"
ros2 run rviz2 rviz2 -d ./src/sick_scan2/launch/rviz/sick_cola.rviz
```

For a quick build, install and run test, some bash scripts are provided in folder `src/sick_scan2/tools/scripts`. Run the following commands:

```
cd ~/sick_scan_ws/src/sick_scan2/tools/scripts
./makeall.bash
./run_simu.bash
```

## Developing with Visual Studio Code

Download the debian package code_1.47.3-1595520028_amd64.deb (or any later version) from https://code.visualstudio.com and install Visual Studio Code by

```sudo apt install ./code_1.47.3-1595520028_amd64.deb```

Open Visual Studio Code by running `code` in the console, select `Customize`, `Tools and languages` and install Python, C/C++, ROS, Colcon Tasks and Markdown (and any other usefull extensions you might need).

Open folder `sick_scan_ws` via `File` menu and save a new workspace with `Save Workspace As...`. Open file `c_cpp_properties.json` in the Visual Studio Code Editor and insert compiler settings:
```
"includePath": [ "~/sick_scan_ws/src/sick_scan2/include/**", "~/sick_scan_ws/src/sick_scan2/tools/test_server/include/**", "~/sick_scan_ws/src/sick_ldmrs_laser/sick_ldmrs_driver/include/**", "~/sick_scan_ws/src/sick_ldmrs_laser/sick_ldmrs_msgs/include/**", "~/sick_scan_ws/build/**", "~/sick_scan_ws/install/**", "/opt/ros/eloquent/include", "/usr/include/**" ],
"defines": [ "LDMRS_SUPPORT=1" ],
```

## Creators

**Michael Lehning** <http://www.lehning.de> on behalf of SICK AG <http://www.sick.com>

