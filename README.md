# etherdaq_ros
The driver has been tested under Ubuntu 18.04 and ROS Melodic.
To build the sources you should use catkin


Usage - Demo
------------

First, should check if the EtherDAQ is working by OptoForce Ethernet Discovery Tool. The tool will 
show you the address of your EtherDAQ device(s) connected to your network.

To run the demo:
```
roslaunch optoforce_etherdaq_driver demo.launch address:=<ip>
```
where `<ip>` is your EtherDAQ's address provided by the Ethernet Discovery Tool.

If the EtherDAQ does not repsonse, please double check the address in the demo.launch file.

Usage - Real world
------------------


The ROS driver is a simple node providing Force/Torque informations on a ROS topic.



Parameters of the node
----------------------
* --help Shows the help screen and exits.
* --rate (default: 100) (in Hz) The publish speed of the F/T informations. It also sets the EtherDAQ speed to the given value. 
* --filter (default: 4) Set the filtering (Valid parameters: 0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz)
* --address The IP address of the EtherDAQ device.
* --wrench  publish older Wrench message type instead of WrenchStamped
* --frame_id arg (default: "base_link") Frame ID for Wrench data


Example: 
 "./etherdaq_node --address 192.168.100.12 --rate 500 --filter 3"
* This will start the node with 500 Hz publish rate and with 50 Hz filter.


Topics of the node
------------------
The node subscribes to /etdaq_zero where you can zero the force/torque readings at the current loading level.
 * The parameter is std_msgs::Bool, if it's true, then the node zeroes otherwise it unzeroes.

The node publishes to the following topics:
*   /diagnostics where you can check the status of the EtherDAQ (speed, last F/T values, system status, address, etc)
*   /etherdaq_data the topic where F/T values are published either in Wrench or in WrenchStamped format if the force and torque units are given 
*   /etherdaq_data_raw topic where F/T values are published either in Wrench or in WrenchStamped format if the force and torque units are not given



The list and short description of source files
----------------------------------------------

* src/etherdaq_driver.cpp: 	   This is the module which implements the communication between EtherDAQ and ROS.
* src/etherdaq_node.cpp: 	   This is the module which publish F/T values on ROS topics using etherdaq_driver.cpp 
                           services.
* src/etherdaq_subscriber.cpp: An example node which subscribes to the topics of etherdaq_node and displays
                           the actual F/T, speed and elapsed times between two packets. Also this node 
                           does a zero/unzero in every 10 seconds.
* script/force_sensor_interface This script creates the same interface as the driver adding tool and offset calibration capability. This script is intended to run with the simulator. The script subscribes to WrenchStamped topic adds offsets.