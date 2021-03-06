/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "rsdriver.h"
#include <rslidar_msgs/rslidarScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_driver
{
rslidarDriver::rslidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh):data_(new rslidar_rawdata::RawData())
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("rslidar"));

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("RS16"));
  double packet_rate;  // packet frequency (Hz)
  std::string model_full_name;

  // product model
  if (config_.model == "RS16")
  {
    packet_rate = 840;
    model_full_name = "RS-LiDAR-16";
  }
  else if (config_.model == "RS32")
  {
    packet_rate = 1690;
    model_full_name = "RS-LiDAR-32";
  }
  else
  {
    ROS_ERROR_STREAM("unknown LIDAR model: " << config_.model);
    packet_rate = 2600.0;
  }
  std::string deviceName(std::string("Robosense ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)

  int npackets = (int)ceil(packet_rate / frequency);
  private_nh.param("npackets", config_.npackets, npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  int msop_udp_port;
  private_nh.param("msop_port", msop_udp_port, (int)MSOP_DATA_PORT_NUMBER);
  int difop_udp_port;
  private_nh.param("difop_port", difop_udp_port, (int)DIFOP_DATA_PORT_NUMBER);

  double cut_angle;
  private_nh.param("cut_angle", cut_angle, -0.01);
  if (cut_angle < 0.0)
  {
    ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
  }
  else if (cut_angle < 360)
  {
    ROS_INFO_STREAM("Cut at specific angle feature activated. "
                    "Cutting rslidar points always at "
                    << cut_angle << " degree.");
  }
  else
  {
    ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is "
                     << "between 0.0 and 360 negative values to deactivate this feature.");
    cut_angle = -0.01;
  }

  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in rslidar packets
  config_.cut_angle = static_cast<int>(cut_angle * 100);

  // Initialize dynamic reconfigure
  // srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig> >(private_nh);
  // dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig>::CallbackType f;
  // f = boost::bind(&rslidarDriver::callback, this, _1, _2);
  // srv_->setCallback(f);  // Set callback function und call initially

  // initialize diagnostics
  // diagnostics_.setHardwareID(deviceName);
  // const double diag_freq = packet_rate / config_.npackets;
  // diag_max_freq_ = diag_freq;
  // diag_min_freq_ = diag_freq;

    // read data from live socket
  msop_input_.reset(new rslidar_driver::InputSocket(private_nh, msop_udp_port));
  difop_input_.reset(new rslidar_driver::InputSocket(private_nh, difop_udp_port));
 

  
  //   using namespace diagnostic_updater;
  // std::string msop_output_packets;
  // private_nh.param("msop_output_packets",msop_output_packets,std::string("rslidar_packets"));
  // std::string diag_topic_name;
  // diag_topic_name=msop_output_packets;
  // diag_topic_.reset(new TopicDiagnostic(diag_topic_name, diagnostics_,
  //                                       FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
  //                                       TimeStampStatusParam()));

  private_nh.param("model", model_, std::string("RS16"));
  data_->loadConfigFile(node, private_nh);
  std::string output_topic_name;
  private_nh.param("output_pointcloud_topic", output_topic_name, std::string("pointcloud"));
  pointcloud_pub_ = node.advertise<sensor_msgs::PointCloud2>(output_topic_name, 1000);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool rslidarDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  rslidar_msgs::rslidarScanPtr scan(new rslidar_msgs::rslidarScan);

  // Since the rslidar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  if (config_.cut_angle >= 0)  // Cut at specific angle feature enabled
  {
    scan->packets.reserve(config_.npackets);
    rslidar_msgs::rslidarPacket tmp_packet;
    while (true)
    {
      while (true)
      {
        int rc = msop_input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
      scan->packets.push_back(tmp_packet);

      static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
      static int last_azimuth = ANGLE_HEAD;

      int azimuth = 256 * tmp_packet.data[44] + tmp_packet.data[45];
      // int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      // Handle overflow 35999->0
      if (azimuth < last_azimuth)
      {
        last_azimuth -= 36000;
      }
      // Check if currently passing cut angle
      if (last_azimuth != ANGLE_HEAD && last_azimuth < config_.cut_angle && azimuth >= config_.cut_angle)
      {
        last_azimuth = azimuth;
        break;  // Cut angle passed, one full revolution collected
      }
      last_azimuth = azimuth;
    }
  }
  else  // standard behaviour
  {
    scan->packets.resize(config_.npackets);
    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(&scan->packets[i], config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
    }
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full rslidar scan.");
  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;

  // rjp
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scan->header).stamp;
  outPoints->header.frame_id = scan->header.frame_id;
  outPoints->clear();
  if (model_ == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scan->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
  else if (model_ == "RS32")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scan->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }

  // process each packet provided by the driver

  data_->block_num = 0;
  for (size_t i = 0; i < scan->packets.size(); ++i)
  {
    data_->unpack(scan->packets[i], outPoints);
  }
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);

  pointcloud_pub_.publish(outMsg);

  // notify diagnostics that a message has been published, updating its status
  // diag_topic_->tick(scan->header.stamp);
  // diagnostics_.update();

  return true;
}

void rslidarDriver::callback(rslidar_driver::rslidarNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
}

} // namespace rslidar_driver
