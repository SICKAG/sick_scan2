# sick_scan2
This stack provides a ROS2 driver for the SICK lidar sensors mentioned in the following list.

## Remark
During the last years we have developed different repositories within the driver development with the projects sick_scan, sick_scan2 and sick_scan_base, with which many lidars and radars of our company can be used under ROS1, ROS2 and generically under C++. For harmonization, due to customer requests and to increase clarity, we decided to combine these developments in the future. As a result of this work, at the end of 2021 we have created the repository [sick_scan_xd](https://github.com/SICKAG/sick_scan_xd), where we will continue future developments. We recommend using this new repository for development so that you can continue to benefit from our improvements and additions to our drivers.

## Table of contents

- [Supported Hardware](#supported-hardware)
- [Requirements](#requirements)
- [Start node](#start-node)
- [Bugs and feature requests](#bugs-and-feature-requests)
- [Timestamping](doc/timestamping.md)
- [Creators](#creators)

This stack provides a ROS2 driver for the SICK laser scanners
mentioned in the following list.


## Supported Hardware

This driver should work with all of the following products.

ROS Device Driver for SICK lidar sensors - supported scanner types:


| **device name**    |  **part no.**                                                                                                                | **description**                                | **tested?**     |
|--------------------|------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|:---------------:|
| TiM240             | [1104981](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/tim2xx/tim240-2050300/p/p654443)  | 1 layer max. range: 10 m, ang. resol. 1.00 [deg], 240 [deg]| ✔ [prototype]|
|                    |                                                                                                                                  | Scan-Rate: 14.5 Hz   |                 |
| TiM551             | [1060445](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim551-2050001/p/p343045)                 | 1 layer max. range: 10 m, ang. resol. 1.00[deg] | ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM561             | [1071419](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim561-2050101/p/p369446)                 | 1 layer max. range: 10 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM571             | [1079742](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim571-2050101/p/p412444)                 | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM781             | [1096807](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim781-2174101/p/p594148)                 | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| TiM781S            | [1096363](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim781s-2174104/p/p594149)                 | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| LMS511-10100 PRO   | [e.g. 1046135](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms5xx/c/g179651)     | 1 layer max. range: 80 m, ang. resol. 0.167 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 100 Hz   |                 |
| LMS1xx-Family      | [e.g. 1041114](https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/c/g91901) | 1 layer max. range: 28 m, ang. resol. 0.25 [deg]| ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 15 Hz   |                 |
| MRS1104            | [1081208](https://www.sick.com/sg/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs1000/mrs1104c-111011/p/p495044)         | 4 layer max. range: 64 m, ang. resol. 0.25 [deg] hor., 2.50 [deg] ver.                                         | ✔ [development]|
|                    |                                                                                                                                  | Scan-Rate: 50 Hz, 4x12.5 Hz            |                 |
| LMS1104            | [1092445](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1000/c/g387151)                         | 1 layer max. range: 64 m, ang. resol. 0.25 [deg] |  ✔ [stable]|
|                    |                                                                                                                                  | Scan-Rate: 150 Hz, 4x37.5 Hz   | 
| LDMRS |   | 4 or 8 layer, max. range: 50/320 m, ang. resol. 0.025°/.../0.25 [deg] | ✔ [development]|
|       |   | Scan-Rate: 12.5-50 Hz | |
| LRS4000 |   | 1 layer, max. range: 130 m, ang. resol. 0.125/0.25/0.5 [deg]  | ✔ [development]|
|       |   | Scan-Rate: 12.5-25 Hz | |


## Requirements

sick_scan2 has been developed and tested using Linux ROS-2 Foxy. It should be compatible with other ROS-2 versions (but not fully tested).

##  Start Node

See quick start for further hints.

## Sopas Mode
This driver supports both COLA-B (binary) and COLA-A (ASCII) communication with the laser scanner. Binary mode is activated by default. Since this mode generates less network traffic.
If the communication mode set in the scanner memory is different from that used by the driver, the scanner's communication mode is changed. This requires a restart of the TCP-IP connection, which can extend the start time by up to 30 seconds.
There are two ways to prevent this:
1. [Recommended] Set the communication mode with the SOPAS ET software to binary and save this setting in the scanner's EEPROM.
2. Use the parameter "use_binary_protocol" to overwrite the default settings of the driver.
3. Setting "use_binary_protocol" to "False" activates COLA-A and disables COLA-B (default)
### Known issue
If the scanner has not been set to binary Sopas in the EEPROM, the automatic restart of the TCP-IP connection does not work after the protocol change. The driver stops in this state:
![Wrong SOPAS Mode](doc/sopas_mode_start_up.png)
#### Workaround
restart the driver node.

## Bugs and feature requests

- Stability issues: Driver is experimental and brand new
- Sopas protocol mapping:
-- All scanners: COLA-B (Binary)
- Software should be further tested, documented and beautified

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

* In case of technical support please open a new issue. For optimal support, add the following information to your request:
 1. Scanner model name,
 2. Ros node startup log,
 3. Sopas file of your scanner configuration.
  The instructions at http://sickusablog.com/create-and-download-a-sopas-file/ show how to create the Sopas file.
* In case of application support please use [https://supportportal.sick.com ](https://supportportal.sick.com).
* Issue Handling: Issues, for which no reply was received from the questioner for more than 7 days,						
  are closed by us because we assume that the user has solved the problem.


## Installation

In the following instructions, replace `<rosdistro>` with the name of your ROS distro (e.g., `dashing`).

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

To build the sick_scan2 ros driver with LDMRS support, libsick_ldmrs is required and flag BUILD_WITH_LDMRS_SUPPORT must be set:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/sick_scan_ws/src/
cd ~/sick_scan_ws/src/
sudo apt-get install ros-$ROS_DISTRO-diagnostic-updater
git clone https://github.com/SICKAG/libsick_ldmrs.git
git clone https://github.com/SICKAG/sick_scan2.git
cd ..
export BUILD_WITH_LDMRS_SUPPORT=True
colcon build --cmake-args " -DBUILD_WITH_LDMRS_SUPPORT=$BUILD_WITH_LDMRS_SUPPORT"
source ~/sick_scan_ws/install/setup.bash
```

The LDMRS driver is originally taken from sick_lmrs_laser, migrated to ROS2 and integrated into sick_scan2. 
Please see https://github.com/SICKAG/sick_ldmrs_laser and https://github.com/SICKAG/libsick_ldmrs for further details.

## Quick Start

```bash
cd ~/sick_scan_ws
source ./install/setup.bash
```
Attention: Replace the ip address for "__hostname" with your scanner ip address.
Default ip address of scanner is 192.168.0.1.
In this example we use the ip address 192.168.0.71

Via the program argument __frame_id the frame of the laserscan messages can be changed. Default is "cloud".

For TiM240:
```
ros2 launch sick_scan2 sick_tim_240.launch.py
```
For TiM5xx:
```
ros2 launch sick_scan2 sick_tim_5xx.launch.py
```
For LMS511:
```
ros2 launch sick_scan2 sick_lms_5xx.launch.py
```

For TiM781:
```
ros2 launch sick_scan2 sick_tim_7xx.launch.py
```
For TiM781S:
```
ros2 launch sick_scan2 sick_tim_7xxS.launch.py
```

For LMS111:
```
ros2 launch sick_scan2 sick_lms_5xx.launch.py
```

For MRS1104:
```
ros2 launch sick_scan2 sick_mrs_1xxx.launch.py
```

For LMS1104:
```
ros2 launch sick_scan2 sick_lms_1xxx.launch.py
```

For LDMRS:
```
ros2 launch sick_scan2 sick_ldmrs.launch.py
```

For LRS4000:
```
ros2 launch sick_scan2 sick_lrs4000.launch.py
```

Start a second terminal window
```
cd ~/sick_scan_ws
source ./install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world cloud
```
Start a third terminal window
```
cd ~/sick_scan_ws
source ./install/setup.bash
rviz2 ./install/sick_scan2/share/sick_scan2/launch/rviz/tim_5xx.rviz
```

The result shoud look like this:
![rviz2_scan](doc/rviz2_scan.png)

## Unit tests

For a quick unit test after installation without the sensor hardware, a test server is provided to simulate a scanner. 
The test server generates scan data examples and responds to command requests. 
Please note, that this test server does not emulate a Lidar sensor. It just sends some simple scan data and response messages to a tcp client.
It can be used for a quick unit test after build and install.

For a unit test, run the following commands in different terminals:

For LDMRS unit test:

```
cd ~/sick_scan_ws
source ./install/setup.bash
ros2 run sick_scan2 test_server --ros-args --params-file src/sick_scan2/tools/test_server/config/test_server_ldmrs.yaml
ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_ldmrs_flexres.yaml -p "hostname:=127.0.0.1"
ros2 run rviz2 rviz2 -d ./src/sick_scan2/launch/rviz/sick_ldmrs.rviz
```

For other scanners supported by sick_scan2, replace $yaml_file by sick_tim_240.yaml, sick_tim_5xx.yaml, sick_tim_7xx.yaml, sick_tim_7xxS.yaml, 
sick_lms_1xx.yaml, sick_lms_5xx.yaml or sick_mrs_1xxx.yaml, and run the following commands:

```
cd ~/sick_scan_ws
source ./install/setup.bash
ros2 run sick_scan2 test_server --ros-args --params-file src/sick_scan2/tools/test_server/config/test_server_cola.yaml
ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/$yaml_file -p "hostname:=127.0.0.1" -p "port:=2112" -p "sw_pll_only_publish:=false"
ros2 run rviz2 rviz2 -d ./src/sick_scan2/launch/rviz/sick_cola.rviz
```

For a quick build, install and run test, some bash scripts are provided in folder `src/sick_scan2/tools/scripts`. Run the following commands:

```
cd ~/sick_scan_ws/src/sick_scan2/tools/scripts
./makeall.bash
./run_simu.bash
```

## Developing with CLion IDE

* Change to workspace directory, e.g.:
```
cd ~/sick_scan_ws
source ./install/setup.bash
```

* Start clion-Shell-Script with directory containing 'CMakeLists.txt', e.g.
  (see https://groups.google.com/forum/#!topic/ros-sig-ng-ros/UMjVH047nVc, Answer from G. Viola)
```
~/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/201.8743.17/bin/clion.sh ./src/sick_scan2
```
  Comment: Please modify the path to your local installation.

* If the build step generates a message like `Could NOT find FastRTPS (missing: FastRTPS_INCLUDE_DIR FastRTPS_LIBRARIES)`,
export addition path infos by the following command:
```
export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH
```
(see https://answers.ros.org/question/334581/could-not-find-fastrtps-missing-fastrtps_include_dir-fastrtps_libraries/).
Or try to set the path manualy
```
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
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

## Keywords

ROS LiDAR
SICK LiDAR
SICK Laser
SICK Laserscanner
TiM5xx
TiM551
TiM561
TiM571
TiM781
TiM781S
LMS111
LMS511
MRS1104
LMS1104
MRS1xxx
LMS1xxx
LDMRS
LRS4000

## Creators

**Michael Lehning**

- <http://www.lehning.de>

on behalf of SICK AG

- <http://www.sick.com>

------------------------------------------------------------------------

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f1/Logo_SICK_AG_2009.svg/1200px-Logo_SICK_AG_2009.svg.png" width="420">

![Lehning Logo](http://www.lehning.de/style/banner.jpg "LEHNING Logo")
