# ouster_ros


[![Build Status](https://travis-ci.com/fada-catec/ouster_ros.svg?token=TpRTcLMNTdG3Avnojha5&branch=main)](https://travis-ci.com/fada-catec/ouster_ros)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

This package is a simpler ROS client implementation for the Ouster lidar sensors with firmware version >=1.14.0. It includes new functionalities that cannot be found in the original [ouster_example](https://github.com/ouster-lidar/ouster_example) (see [Motivation](#Motivation))

## Table of Contents
* [Motivation / Features](#Motivation%20/%20Features)
* [Requirements](#Requirements)
* [Instructions](#Instructions)

   - [Building](#Building)
   - [PTP Configuration](#PTP%20Configuration)
   - [Usage](#Usage)

* [Launch files](#Launch%20files)
* [Nodes](#Nodes)
* [License](#License)
* [Help](#Help)

## Motivation / Features
The motivation for creating this package is:
* Unify the example libraries provided by Ouster in one place, making it easier to compile
* Avoid the overhead given in the official Ouster example
* Possibility to disable IMU message processing in case of non-use, freeing up CPU
* Possibility to inverse TF publication between /os_sensor and /os_lidar
* Code-level check of the synchronization of published messages when setting up PTP

If you are ok without these functionalities, feel free to use the original [ouster_example](https://github.com/ouster-lidar/ouster_example) from Ouster.

## Requirements
* Tested with [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on Ubuntu 18.04, but it may work with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) on Ubuntu 16.04 (untested) with some extra packages:
   - ros-melodic-pcl-ros
   - ros-melodic-tf2-geometry-msgs
* Ouster sensor, firmware **[>= 1.14.0]**. Tested with OS0-128 and OS1-64, but any other with the proper firmware should work

## Instructions

### Building

Just clone in your ROS workspace and do `catkin_make -DCMAKE_BUILD_TYPE=Release`

### PTP Configuration

1. `sudo apt install linuxptp chrony ethtool`
2. Make sure the ethernet interface supports PTP with `sudo ethtool -T YOUR_INTERFACE`
3. Modify /etc/linuxptp/ptp4l.conf with:
   - clockClass 128
   - boundary_clock_jbod 1
   - [YOUR_INTERFACE] <- append this to the end of the file
4. Create folder `sudo mkdir -p /etc/systemd/system/ptp4l.service.d`
5. Create file `/etc/systemd/system/ptp4l.service.d/override.conf` with:
   ~~~
   [Service]
   ExecStart=
   ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf
   ~~~
6. Create folder: `sudo mkdir -p /etc/systemd/system/phc2sys.service.d`
7. Create file /etc/systemd/system/phc2sys.service.d/override.conf with:
   ~~~
   [Service]
   ExecStart=
   ExecStart=/usr/sbin/phc2sys -w -s CLOCK_REALTIME -c YOUR_INTERFACE
   ~~~
8. Modify /etc/systemd/system/phc2shm.service:
   ~~~
   [Unit]
   Description=Synchronize PTP hardware clock (PHC) to NTP SHM
   Documentation=man:phc2sys
   After=ntpdate.service
   Requires=ptp4l.service
   After=ptp4l.service
   [Service]
   Type=simple
   ExecStart=/usr/sbin/phc2sys -s YOUR_INTERFACE -E ntpshm -w 
   [Install]
   WantedBy=multi-user.target
   ~~~
**NOTE:** Change YOUR_INTERFACE to the ethernet interface used (ex: eno1)

### Usage

1. Start PTP services with `rosrun ouster_ros lidar_ptp.sh`
2. In case lidar is in DHCP mode: `sudo dnsmasq -C /dev/null -kd -F MIN_IP,MAX_IP -i YOUR_INTERFACE  --bind-dynamic` (ex: MIN_IP=10.5.0.1, MAX_IP=10.5.0.255, YOUR_INTERFACE=eno1)
3. `roslaunch ouster_ros ouster.launch`

## Launch files

* **ouster.launch**:

   Arguments:
   
   * **sensor_hostname** (string) - Lidar IP or ID (ex: 10.5.0.10)
   * **udp_dest** (string) - Local PC IP (ex: 10.5.0.1)
   * **lidar_port** (int) - Port to which the sensor should send lidar data (default 7502)
   * **imu_port** (int) - Port to which the sensor should send imu data (default 7503). If 0, IMU is disabled.
   * **lidar_mode** (string) - Lidar resolution and rate [512x10, 512x20, 1024x10, 1024x20, 2048x10]
   * **timestamp_mode** (string) - Method used to timestamp measurements [TIME_FROM_INTERNAL_OSC, TIME_FROM_SYNC_PULSE_IN, TIME_FROM_PTP_1588]
   * **tf_prefix** (string) - Namespace for tf transform (default = "")
   * **tf_inverse** (bool) - Inverse tf os_sensor <-> os_lidar.
   * **stamp_offset** (float) - Seconds to be added to the stamp header before publishing it
   * **max_sync_diff** (float) - Max admissible delay in the stamp header
   * **enable_img_node** (bool) - Whether or not enable the image node
   * **metadata** (string) - Path to metadata file for image node

## Nodes

* **os_node**

   Advertised topics:
   - **~/points** ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - Pointcloud from lidar
   - **~/imu** ([sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)) - Imu topic from sensor
   - **~/lidar_status** ([std_msgs/Header](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html)) - Publish one message for each pointcloud. In case lidar sensor fails (ex: overheating), it stops publishing.
* **os_img_node**

   Advertised topics:
   - **~/range_image** ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
   - **~/noise_image** ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
   - **~/intensity_image** ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))

## License

See [License](LICENSE)

This code is distributed with the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of merchantability or fitness for a particular purpose. 

All the credits go to the [Ouster](https://ouster.com/) team and their original repo [ouster_example](https://github.com/ouster-lidar/ouster_example) for establishing the basis of this project.

## Help / Contribution

* Contact: **Rafael Caballero** (rcaballero@catec.aero)

   ![FADA](./doc/FADA.png)    ![CATEC](./doc/CATEC.png)

* Found a bug? Create an ISSUE!

* Do you want to contribute? Create a PULL-REQUEST!