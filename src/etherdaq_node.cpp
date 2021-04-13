/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, OptoForce, Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the OptoForce nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** 
 * Simple stand-alone ROS node that takes data from EtherDAQ sensor and
 * Publishes it ROS topic
 */

#include <ros/ros.h>
#include <optoforce_etherdaq_driver/etherdaq_driver.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <iostream>
#include <memory>

#include <std_srvs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>

using namespace optoforce_etherdaq_driver;

class FTNode
{
public:
  FTNode() : nh_("~"), pub_rate_(100.0)
  {
    std::string address;
    double pub_rate_hz;
    int filter_hz;
    nh_.param<double>("Rate", pub_rate_hz, 100.0);
    nh_.param<int>("FilterRate", filter_hz, 4); // set filtering (0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz)
    nh_.param<bool>("Wrench", publish_wrench_, false);
    nh_.param<bool>("UpdateToolOffset", update_tool_offset_, false);
    nh_.param<std::string>("Address", address, "");
    nh_.param<std::string>("FrameId", frame_id_, "ft_frame");
    nh_.param<std::string>("BaseFrameId", base_frame_id_, "base_link");
    nh_.param<std::vector<double>>("Offset", offset_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    nh_.param<std::vector<double>>("ToolOffset", tool_offset_, {0.0, 0.0, 0.0, 0.0});

    std::string topic_name = "data";

    if (filter_hz < 0 || filter_hz > 6) throw std::runtime_error("Invalid filter rate!");

    etherdaq_.reset(new EtherDAQDriver(address, pub_rate_hz, filter_hz));

    if (publish_wrench_)
    {
      pub_ = nh_.advertise<geometry_msgs::Wrench>(topic_name, 100);
    }
    else 
    {
      pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(topic_name, 100);
    }
    pub_rate_ = ros::Rate(pub_rate_hz);

    if (update_tool_offset_)
    {
      if(!listener_.waitForTransform(base_frame_id_, frame_id_, ros::Time(0), ros::Duration(5.0)))
      {
        update_tool_offset_ = false;
        ROS_WARN_STREAM("Could not get reference frames, not updating tool offset.");
      }
    }
    
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 2);
    offset_sub_ = nh_.subscribe("offset", 1, &FTNode::offset_cb, this);

    if (update_tool_offset_)
      tool_offset_sub_ = nh_.subscribe("tool_offset", 1, &FTNode::tool_offset_cb, this);
  }

  void offset_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if(msg->data.size()==6)
    {
      offset_ = msg->data;            
    }
  }

  void tool_offset_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if(msg->data.size()==4)
    {
      tool_offset_ = msg->data;            
    }
  }

  void run()
  {
    geometry_msgs::WrenchStamped data;
    ros::Duration diag_pub_duration(1.0);
    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.status.reserve(1);
    diagnostic_updater::DiagnosticStatusWrapper diag_status;
    ros::Time last_diag_pub_time(ros::Time::now());

    ROS_INFO_STREAM("F/T node started");

    while (ros::ok())
    {
      if (etherdaq_->waitForNewData())
      {
        etherdaq_->getData(data);

        if(update_tool_offset_)
        {
          tf::StampedTransform transform;
          try
          {
            listener_.lookupTransform(base_frame_id_, frame_id_, ros::Time(0), transform);
            tf::transformTFToKDL(transform, tool_pose_);
          }
          catch (tf::TransformException &ex) {}
        }

        data.wrench.force.x -= offset_[0];
        data.wrench.force.y -= offset_[1];
        data.wrench.force.z -= offset_[2];
        data.wrench.torque.x -= offset_[3];
        data.wrench.torque.y -= offset_[4];
        data.wrench.torque.z -= offset_[5];

        KDL::Wrench ret(KDL::Vector(data.wrench.force.x, data.wrench.force.y, data.wrench.force.z), KDL::Vector(data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z));
        ret = ret + tool_pose_.Inverse().M * KDL::Wrench(KDL::Vector(0.0, 0.0, tool_offset_[0] * 9.8),KDL::Vector(0.0, 0.0, 0.0)).RefPoint(tool_pose_.Inverse().M * KDL::Vector(tool_offset_[1], tool_offset_[2], tool_offset_[3]));

        data.wrench.force.x = ret.force[0];
        data.wrench.force.y = ret.force[1];
        data.wrench.force.z = ret.force[2];
        data.wrench.torque.x = ret.torque[0];
        data.wrench.torque.y = ret.torque[1];
        data.wrench.torque.z = ret.torque[2];

        if (publish_wrench_) 
        {
          data.header.frame_id = frame_id_;
          pub_.publish(data.wrench);
        }
        else 
        {
          data.header.frame_id = frame_id_;
          pub_.publish(data);
        }
      }
      
      ros::Time current_time(ros::Time::now());
      if ( (current_time - last_diag_pub_time) > diag_pub_duration )
      {
          diag_array.status.clear();
          etherdaq_->diagnostics(diag_status);
          diag_array.status.push_back(diag_status);
          diag_array.header.stamp = ros::Time::now();
          diag_pub_.publish(diag_array);
          last_diag_pub_time = current_time;
      }
      
      ros::spinOnce();
      pub_rate_.sleep();
    }
  }

protected:
  ros::NodeHandle nh_;
  std::shared_ptr<EtherDAQDriver> etherdaq_;
  ros::Publisher pub_;
  ros::Publisher diag_pub_;
  ros::Subscriber offset_sub_;
  ros::Subscriber tool_offset_sub_;
  tf::TransformListener listener_;

  ros::Rate pub_rate_;
  std::string frame_id_;
  std::string base_frame_id_;
  bool update_tool_offset_;
  std::vector<double> offset_;
  std::vector<double> tool_offset_;
  KDL::Frame tool_pose_;
  bool publish_wrench_;
};

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "etherdaq_node");
  FTNode node;
  node.run();
  return 0;

  // float pub_rate_hz;
  // int filter_hz; 
  // string address;
  // string frame_id;

  // po::options_description desc("Options");
  // desc.add_options()
  //   ("help", "display help")
  //   ("rate", po::value<float>(&pub_rate_hz)->default_value(100.0), "set publish rate and Ethernet DAQ speed (in hertz)")
	// ("filter", po::value<int>(&filter_hz)->default_value(4), "set filtering (0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz)") 
  //   ("wrench", "publish older Wrench message type instead of WrenchStamped")
  //   ("address", po::value<string>(&address), "IP address of EthernetDAQ box")
  //   ("frame_id", po::value<string>(&frame_id)->default_value("base_link"), "Frame ID for Wrench data")  
  //   ;
     
  // po::positional_options_description p;
  // p.add("address",  1);

  // po::variables_map vm;
  // po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  // po::notify(vm);

  // if (vm.count("help"))
  // {
  //   cout << desc << endl;
  //   exit(EXIT_SUCCESS);
  // }      

  // if (!vm.count("address"))
  // {
  //   cout << desc << endl;
  //   cerr << "Please specify address of EthernetDAQ" << endl;
  //   exit(EXIT_FAILURE);
  // }
	
  // if (filter_hz < 0 || filter_hz > 6) {
	//   cout << desc << endl;
	//   cerr<<"Please specify a valid filtering value instead of "<<filter_hz<<endl;
	//   exit(EXIT_FAILURE);
  // }

  // bool publish_wrench = false;
  // if (vm.count("wrench"))
  // {
  //   publish_wrench = true;
  //   ROS_WARN("Publishing EthernetDAQ data as geometry_msgs::Wrench is deprecated");
  // }

  // etherdaq = new optoforce_etherdaq_driver::EtherDAQDriver(address, pub_rate_hz, filter_hz);
	
  // bool isRawData = etherdaq->isRawData();

  // std::string topicName = "ethdaq_data";	
  // if (isRawData) {
	//   topicName += "_raw";
  // }
	
  // ros::Publisher pub;
  // ros::Subscriber sub = nh.subscribe("ethdaq_zero", 1000, zeroFunction);
  // if (publish_wrench)
  // {
  //   pub = nh.advertise<geometry_msgs::Wrench>(topicName, 100);
  // }
  // else 
  // {
  //   pub = nh.advertise<geometry_msgs::WrenchStamped>(topicName, 100);
  // }
  // ros::Rate pub_rate(pub_rate_hz);
  // geometry_msgs::WrenchStamped data;

  // ros::Duration diag_pub_duration(1.0);
  // ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);
  // diagnostic_msgs::DiagnosticArray diag_array;
  // diag_array.status.reserve(1);
  // diagnostic_updater::DiagnosticStatusWrapper diag_status;
  // ros::Time last_diag_pub_time(ros::Time::now());

  // unsigned int packetCount = 0; 
  // ros::Time startTime(ros::Time::now());	
  // while (ros::ok())
  // {
  //   if (etherdaq->waitForNewData())
  //   {
  //     etherdaq->getData(data);
  //     packetCount++; 
  //     if (publish_wrench) 
  //     {
	// data.header.frame_id = frame_id;
  //       pub.publish(data.wrench);
  //     }
  //     else 
  //     {
  //  	data.header.frame_id = frame_id;
  //       pub.publish(data);
  //     }
  //   }
    
  //   ros::Time current_time(ros::Time::now());
  //   if ( (current_time - last_diag_pub_time) > diag_pub_duration )
  //   {
  //     diag_array.status.clear();
  //     etherdaq->diagnostics(diag_status);
  //     diag_array.status.push_back(diag_status);
  //     diag_array.header.stamp = ros::Time::now();
  //     diag_pub.publish(diag_array);
  //     last_diag_pub_time = current_time;
  //   }

  //   ros::spinOnce();
  //   pub_rate.sleep();
  // }
	
  // if (etherdaq != NULL) {
	//   delete etherdaq;
  // }
  
  // return 0;
}
