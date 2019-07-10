/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  RSLIDAR 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw RSLIDAR LIDAR packets into useful
 *  formats.
 *
 */
#include "rawdata.h"

namespace rslidar_rawdata {
    float VERT_ANGLE[32];
    float HORI_ANGLE[32];
    float aIntensityCal[7][32];
    float aIntensityCal_old[1600][32];
    bool Curvesis_new = true;
    int g_ChannelNum[32][51];
    float CurvesRate[32];

    float temper = 31.0;
    int tempPacketNum = 0;
    int numOfLasers = 16;
    int TEMPERATURE_RANGE = 40;
}

namespace rslidar_rawdata
{
RawData::RawData()
{
  this->is_init_angle_ = false;
  this->is_init_curve_ = false;
}

void RawData::loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string anglePath, curvesPath, channelPath, curvesRatePath;
  std::string model;

  private_nh.param("curves_path", curvesPath, std::string(""));
  private_nh.param("angle_path", anglePath, std::string(""));
  private_nh.param("channel_path", channelPath, std::string(""));
  private_nh.param("curves_rate_path", curvesRatePath, std::string(""));

  private_nh.param("model", model, std::string("RS16"));
  if (model == "RS16")
  {
    numOfLasers = 16;
  }
  else if (model == "RS32")
  {
    numOfLasers = 32;
    TEMPERATURE_RANGE = 50;
  }

  /// 读参数文件 2017-02-27
  FILE* f_inten = fopen(curvesPath.c_str(), "r");
  int loopi = 0;
  int loopj = 0;
  int loop_num;
  if (!f_inten)
  {
    ROS_ERROR_STREAM(curvesPath << " does not exist");
  }
  else
  {
    fseek(f_inten, 0, SEEK_END);  //定位到文件末
    int size = ftell(f_inten);    //文件长度
    ROS_INFO_STREAM("size is::::::::::::::::::::::::::::: " << size);
    if (size > 10000)  //老版的curve
    {
      Curvesis_new = false;
      loop_num = 1600;
    }
    else
    {
      Curvesis_new = true;
      loop_num = 7;
    }
    fseek(f_inten, 0, SEEK_SET);
    while (!feof(f_inten))
    {
      float a[32];
      loopi++;

      if (loopi > loop_num)
        break;
      if (numOfLasers == 16)
      {
        int tmp = fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &a[0], &a[1], &a[2], &a[3],
                         &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12], &a[13], &a[14], &a[15]);
      }
      else if (numOfLasers == 32)
      {
        int tmp = fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
                                  "%f,%f,%f,%f\n",
                         &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12],
                         &a[13], &a[14], &a[15], &a[16], &a[17], &a[18], &a[19], &a[20], &a[21], &a[22], &a[23], &a[24],
                         &a[25], &a[26], &a[27], &a[28], &a[29], &a[30], &a[31]);
      }
      if (Curvesis_new)
      {
        for (loopj = 0; loopj < numOfLasers; loopj++)
        {
          aIntensityCal[loopi - 1][loopj] = a[loopj];
        }
      }
      else
      {
        for (loopj = 0; loopj < numOfLasers; loopj++)
        {
          aIntensityCal_old[loopi - 1][loopj] = a[loopj];
        }
      }
      // ROS_INFO_STREAM("new is " << a[0]);
    }
    fclose(f_inten);
  }
  //=============================================================
  FILE* f_angle = fopen(anglePath.c_str(), "r");
  if (!f_angle)
  {
    ROS_ERROR_STREAM(anglePath << " does not exist");
  }
  else
  {
    float b[32], d[32];
    int loopk = 0;
    int loopn = 0;
    while (!feof(f_angle))
    {
      int tmp = fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
      loopk++;
      if (loopk > (numOfLasers - 1))
        break;
    }
    for (loopn = 0; loopn < numOfLasers; loopn++)
    {
      VERT_ANGLE[loopn] = b[loopn] / 180 * M_PI;
      HORI_ANGLE[loopn] = d[loopn] * 100;
    }
    fclose(f_angle);
  }

  //=============================================================
  FILE* f_channel = fopen(channelPath.c_str(), "r");
  if (!f_channel)
  {
    ROS_ERROR_STREAM(channelPath << " does not exist");
  }
  else
  {
    int loopl = 0;
    int loopm = 0;
    int c[51];
    int tempMode = 1;
    while (!feof(f_channel))
    {
      if (numOfLasers == 16)
      {
        int tmp = fscanf(f_channel,
                         "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                         "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                         &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
                         &c[13], &c[14], &c[15], &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24],
                         &c[25], &c[26], &c[27], &c[28], &c[29], &c[30], &c[31], &c[32], &c[33], &c[34], &c[35], &c[36],
                         &c[37], &c[38], &c[39], &c[40]);
      }
      else
      {
        int tmp = fscanf(
            f_channel, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                       "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12], &c[13],
            &c[14], &c[15], &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24], &c[25], &c[26],
            &c[27], &c[28], &c[29], &c[30], &c[31], &c[32], &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39],
            &c[40], &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48], &c[49], &c[50]);
      }
      //                if (c[1] < 100 || c[1] > 3000)
      //                {
      //                    tempMode = 0;
      //                }
      for (loopl = 0; loopl < TEMPERATURE_RANGE + 1; loopl++)
      {
        g_ChannelNum[loopm][loopl] = c[tempMode * loopl];
      }
      loopm++;
      if (loopm > (numOfLasers - 1))
      {
        break;
      }
    }
    fclose(f_channel);
  }

  if (numOfLasers == 32)
  {
    FILE* f_curvesRate = fopen(curvesRatePath.c_str(), "r");
    if (!f_curvesRate)
    {
      ROS_ERROR_STREAM(curvesRatePath << " does not exist");
    }
    else
    {
      int loopk = 0;
      while (!feof(f_curvesRate))
      {
        int tmp = fscanf(f_curvesRate, "%f\n", &CurvesRate[loopk]);
        loopk++;
        if (loopk > (numOfLasers - 1))
          break;
      }
      fclose(f_curvesRate);
    }
  }

  // receive difop data
  // subscribe to difop rslidar packets, if not right correct data in difop, it will not revise the correct data in the
  // VERT_ANGLE, HORI_ANGLE etc.
  // std::string subscribe_difop_packets;
  // private_nh.param("subscribe_difop_packets",subscribe_difop_packets,std::string("rslidar_packets_difop"));
  // difop_sub_ = node.subscribe(subscribe_difop_packets, 10, &RawData::processDifop, (RawData*)this);

  // std::string publish_device_info;
  // private_nh.param("device_info",publish_device_info,std::string("rslidar_device_info"));
  // device_info_pub_ = node.advertise<rslidar_msgs::rslidarDeviceInfo>(publish_device_info,2);
}

void RawData::processDifop(const rslidar_msgs::rslidarPacket::ConstPtr& difop_msg)
{
  // std::cout << "Enter difop callback!" << std::endl;
  const uint8_t* data = &difop_msg->data[0];

  if (!this->is_init_curve_)
  {
    // check header
    if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a)
    {
      bool curve_flag = true;
      // check difop reigon has beed flashed the right data
      if ((data[50] == 0x00 || data[50] == 0xff) && (data[51] == 0x00 || data[51] == 0xff) &&
          (data[52] == 0x00 || data[52] == 0xff) && (data[53] == 0x00 || data[53] == 0xff))
      {
        curve_flag = false;
      }

      // TODO check why rsview here no 32 laser, be more careful the new, old version
      // Init curves
      if (curve_flag)
      {
        unsigned char checkbit;
        int bit1, bit2;
        for (int loopn = 0; loopn < numOfLasers; ++loopn)
        {
          // check the curves' parameter in difop
          checkbit = *(data + 50 + loopn * 15) ^ *(data + 50 + loopn * 15 + 1);
          for (int loopm = 1; loopm < 7; ++loopm)
          {
            checkbit = checkbit ^ (*(data + 50 + loopn * 15 + loopm * 2)) ^ (*(data + 50 + loopn * 15 + loopm * 2 + 1));
          }
          if (checkbit != *(data + 50 + loopn * 15 + 14))
          {
            return;
          }
        }
        for (int loopn = 0; loopn < numOfLasers; ++loopn)
        {
          // calculate curves' parameters
          bit1 = static_cast<int>(*(data + 50 + loopn * 15));
          bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 1));
          aIntensityCal[0][loopn] = (bit1 * 256 + bit2) * 0.001;
          bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 2));
          bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 3));
          aIntensityCal[1][loopn] = (bit1 * 256 + bit2) * 0.001;
          bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 4));
          bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 5));
          aIntensityCal[2][loopn] = (bit1 * 256 + bit2) * 0.001;
          bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 6));
          bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 7));
          aIntensityCal[3][loopn] = (bit1 * 256 + bit2) * 0.001;
          bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 8));
          bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 9));
          aIntensityCal[4][loopn] = (bit1 * 256 + bit2) * 0.00001;
          bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 10));
          bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 11));
          aIntensityCal[5][loopn] = -(bit1 * 256 + bit2) * 0.0001;
          bit1 = static_cast<int>(*(data + 50 + loopn * 15 + 12));
          bit2 = static_cast<int>(*(data + 50 + loopn * 15 + 13));
          aIntensityCal[6][loopn] = (bit1 * 256 + bit2) * 0.001;
          // std::cout << aIntensityCal[0][loopn] << "\t" << aIntensityCal[1][loopn] << "\t" << aIntensityCal[2][loopn]
          //         << "\t" << aIntensityCal[3][loopn] << "\t" << aIntensityCal[4][loopn] << "\t"
          //          << aIntensityCal[5][loopn] << "\t" << aIntensityCal[6][loopn] << std::endl;
          ;
        }
        this->is_init_curve_ = true;
        std::cout << "this->is_init_curve_ = "
                  << "true!" << std::endl;
        Curvesis_new = true;
      }
    }
  }

  if (!this->is_init_angle_)
  {
    // check header
    if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a)
    {
      bool angle_flag = true;
      // check difop reigon has beed flashed the right data
      if ((data[1165] == 0x00 || data[1165] == 0xff) && (data[1166] == 0x00 || data[1166] == 0xff) &&
          (data[1167] == 0x00 || data[1167] == 0xff) && (data[1168] == 0x00 || data[1168] == 0xff))
      {
        angle_flag = false;
      }
      // angle
      if (angle_flag)
      {
        // TODO check the HORI_ANGLE
        int bit1, bit2, bit3, symbolbit;
        for (int loopn = 0; loopn < numOfLasers; ++loopn)
        {
          if (loopn < 8 && numOfLasers == 16)
          {
            symbolbit = -1;
          }
          else
          {
            symbolbit = 1;
          }
          bit1 = static_cast<int>(*(data + 1165 + loopn * 3));
          bit2 = static_cast<int>(*(data + 1165 + loopn * 3 + 1));
          bit3 = static_cast<int>(*(data + 1165 + loopn * 3 + 2));
          VERT_ANGLE[loopn] = (bit1 * 256 * 256 + bit2 * 256 + bit3) * symbolbit * 0.0001f / 180 * M_PI;
          // std::cout << VERT_ANGLE[loopn] << std::endl;
          // TODO
          HORI_ANGLE[loopn] = 0;
        }
        this->is_init_angle_ = true;
        std::cout << "this->is_init_angle_ = "
                  << "true!" << std::endl;
      }
    }
  }

  parseDeviceInfo(data, device_info_);
  device_info_pub_.publish(device_info_);
  
  // std::cout << "DIFOP data! +++++++++++++" << std::endl;
}



void RawData::parseDeviceInfo(const uint8_t* data, rslidar_msgs::rslidarDeviceInfo &device_info) {
  device_info.mot_speed = parseMotSpeed(data);
  device_info.lidar_ip = parseLidarIP(data);
  device_info.dest_pc_ip = parsePCIP(data);
  device_info.lidar_mac_addr = parseLidarMACAddress(data);
  device_info.msop_lidar_send_port = parseMsopLidarSendPort(data);
  device_info.msop_pc_recv_port = parseMsopPCRecvPort(data);
  device_info.difop_lidar_send_port = parseDifopLidarSendport(data);
  device_info.difop_pc_recv_port = parseDifopPCRecvPort(data);
  device_info.mot_phase = parseMotPhase(data);
  device_info.top_frm = parseTopFrm(data);
  device_info.bot_frm = parseBotFrm(data);
  device_info.serial_number = parseSerialNumber(data);
  device_info.utc_time = parseUTCTime(data);
  device_info.ldat1 = parseLdat1(data);
  device_info.ldat2 = parseLdat2(data);
  device_info.vdat_12v = parseVdat12V(data);
  device_info.vdat_12v_m = parseVdat12VM(data);
  device_info.vdat_5v = parseVdat5V(data);
  device_info.vdat_3v3 = parseVdat3V3(data);
  device_info.vdat_2v5 = parseVdat2V5(data);
  device_info.vdat_1v2 = parseVdat1V2(data);
  device_info.cksum_st = parseCksumst(data);
  device_info.manc_err1_per = parseMancErr1Per(data);
  device_info.manc_err2_per = parseMancErr2Per(data);
  device_info.bot_temperature1 = parseBotTemperature1(data);
  device_info.bot_temperature2 = parseBottemperature2(data);
  device_info.top_temperature1 = parseTopTemperature1(data);
  device_info.top_temperature2 = parseTopTemperature2(data);
  device_info.bot_temperature5 = parseBotTemperature5(data);
  device_info.pps_lock = parsePPSLock(data);
  device_info.gprmc_lock = parseGprmcLock(data);
  device_info.utc_lock = parseUTCLock(data);

}

int32_t RawData::parseMotSpeed(const uint8_t* data) {
  // 雷达电机转速  单位 rpm
  if(data[8] == 0x04 && data[9] == 0xB0) {
    return 1200;
  }
  else if(data[8] == 0x02 && data[9] == 0x58) {
    return 600;
  }
  else if(data[8] == 0x01 && data[9] == 0x2C) {
   return 300;
  } 
}

std::string RawData::parseLidarIP(const uint8_t* data) {
  std::string ip = std::to_string(data[10]) + ".";
  ip += std::to_string(data[11]);
  ip += ".";
  ip += std::to_string(data[12]);
  ip += ".";
  ip += std::to_string(data[13]);
  return ip;
}

std::string RawData::parsePCIP(const uint8_t* data) {
  std::string ip = std::to_string(data[14]) + ".";
  ip += std::to_string(data[15]);
  ip += ".";
  ip += std::to_string(data[16]);
  ip += ".";
  ip += std::to_string(data[17]);
  return ip;
}

std::string RawData::parseLidarMACAddress(const uint8_t* data) {
  // MAC Address
  std::string mac_address = ByteToHex(data[18]) + "-";
  mac_address += ByteToHex(data[19]);
  mac_address += "-";
  mac_address += ByteToHex(data[20]);
  mac_address += "-";
  mac_address += ByteToHex(data[21]);
  mac_address += "-";
  mac_address += ByteToHex(data[22]);
  mac_address += "-";
  mac_address += ByteToHex(data[23]);
  return mac_address;
}

std::string RawData::parseMsopLidarSendPort(const uint8_t* data) {
  uint32_t frame_1 = data[24];
  uint8_t frame_2 = data[25];
  frame_1 <<= 8;
  frame_1 |= frame_2;
  return std::to_string(frame_1);
}

std::string RawData::parseMsopPCRecvPort(const uint8_t* data) {
  uint32_t frame_1 = data[26];
  uint8_t frame_2 = data[27];
  frame_1 <<= 8;
  frame_1 |= frame_2;
  return std::to_string(frame_1);
}

std::string RawData::parseDifopLidarSendport(const uint8_t* data) {
  uint32_t frame_1 = data[28];
  uint8_t frame_2 = data[29];
  frame_1 <<= 8;
  frame_1 |= frame_2;
  return std::to_string(frame_1);
}

std::string RawData::parseDifopPCRecvPort(const uint8_t* data) {
  uint32_t frame_1 = data[30];
  uint8_t frame_2 = data[31];
  frame_1 <<= 8;
  frame_1 |= frame_2;
  device_info_.difop_pc_recv_port = std::to_string(frame_1);
}

int32_t RawData::parseMotPhase(const uint8_t* data) {
  uint32_t frame_1 = data[38];
  uint8_t frame_2 = data[39];
  frame_1 <<= 8;  

  frame_1 |= frame_2;

  return frame_1;
}

std::string RawData::parseTopFrm(const uint8_t* data) {
  // 主板固件版本
  std::string top_frm = "0x" + ByteToHex(data[40]);
  top_frm += ByteToHex(data[41]);
  top_frm += ByteToHex(data[42]);
  top_frm += ByteToHex(data[43]);
  top_frm += ByteToHex(data[44]);
  return top_frm;
}

std::string RawData::parseBotFrm(const uint8_t* data) {
  std::string bot_frm = "0x" + ByteToHex(data[40]);
  bot_frm += ByteToHex(data[45]);
  bot_frm += ByteToHex(data[46]);
  bot_frm += ByteToHex(data[47]);
  bot_frm += ByteToHex(data[48]);
  return bot_frm;
}

std::string RawData::parseSerialNumber(const uint8_t* data) {
    // 雷达序列号
  std::string serial_num = ByteToHex(data[292]) + "-";
  serial_num += ByteToHex(data[293]);
  serial_num += "-";
  serial_num += ByteToHex(data[294]);
  serial_num += "-";
  serial_num += ByteToHex(data[295]);
  serial_num += "-";
  serial_num += ByteToHex(data[296]);
  serial_num += "-";
  serial_num += ByteToHex(data[297]);
  return serial_num;
}

std::string RawData::parseUTCTime(const uint8_t* data) {
  std::string utc_time;
  std::string year = std::to_string(data[303] + 2000);
  utc_time = year + "-";

  std::string month = std::to_string(data[304]);
  utc_time += month;
  utc_time += "-";

  std::string day = std::to_string(data[305]);
  utc_time += day;
  utc_time += "  ";

  std::string hour = std::to_string(data[306]);
  utc_time += hour;
  utc_time += ":";

  std::string minute = std::to_string(data[307]);
  utc_time += minute;
  utc_time += ":";

  std::string second = std::to_string(data[308]);
  utc_time += second;
  utc_time += ":";

  uint16_t ms = data[309];
  ms <<= 8;
  ms |= data[310];
  utc_time += std::to_string(ms);
  utc_time += ".";

  uint16_t us = data[311];
  us <<= 8;
  us |= data[312];
  utc_time += std::to_string(us);
  
  return utc_time;
}

float RawData::parseLdat1(const uint8_t* data) {
  uint8_t frame_1 = data[313];
  uint8_t symbol = GetSubByte(frame_1,7,1);
  frame_1 <<= 1;
  frame_1 >>= 1;
  uint32_t ldat = frame_1;
  ldat <<= 8;
  ldat |= data[314];
  
  ldat <<= 8;
  ldat |= data[315];

  if(symbol == 1) {
    return ldat / 1000.0;
  }
  
  return (-1) * ldat / 1000.0;
}

float RawData::parseLdat2(const uint8_t* data) {
  uint8_t frame_1 = data[316];
  uint8_t symbol = GetSubByte(frame_1,7,1);
  frame_1 <<= 1;
  frame_1 >>= 1;
  uint32_t ldat = frame_1;
  ldat <<= 8;
  ldat |= data[317];
  
  ldat <<= 8;
  ldat |= data[318];

  if(symbol == 0) {
    return ldat / 1000.0;
  }
  
  return (-1) * ldat / 1000.0;
}

float RawData::parseVdat12V(const uint8_t* data) {
  uint8_t frame_1 = data[319];
  frame_1 <<= 4;
  frame_1 >>= 4;

  uint32_t vdat = frame_1;
  vdat <<= 8;
  vdat |= data[320];

  return vdat * 2.5 * 12 / 4096.0;
}

float RawData::parseVdat12VM(const uint8_t* data) {
  uint8_t frame_1 = data[321];
  frame_1 <<= 4;
  frame_1 >>= 4;

  uint32_t vdat = frame_1;
  vdat <<= 8;
  vdat |= data[322];

  return vdat * 2.5 * 12 / 4096.0;  
}

float RawData::parseVdat5V(const uint8_t* data) {
  uint8_t frame_1 = data[323];
  frame_1 <<= 4;
  frame_1 >>= 4;

  uint32_t vdat = frame_1;
  vdat <<= 8;
  vdat |= data[324];

  return vdat * 2.5 * 4 / 4096.0;    
}

float RawData::parseVdat3V3(const uint8_t* data) {
  uint8_t frame_1 = data[325];
  frame_1 <<= 4;
  frame_1 >>= 4;

  uint32_t vdat = frame_1;
  vdat <<= 8;
  vdat |= data[326];

  return vdat * 2.5 * 2 / 4096.0;  
}

float RawData::parseVdat2V5(const uint8_t* data) {
  uint8_t frame_1 = data[327];
  frame_1 <<= 4;
  frame_1 >>= 4;

  uint32_t vdat = frame_1;
  vdat <<= 8;
  vdat |= data[328];

  return vdat * 2.5 * 2 / 4096.0; 
}

float RawData::parseVdat1V2(const uint8_t* data) {
  uint8_t frame_1 = data[329];
  frame_1 <<= 4;
  frame_1 >>= 4;

  uint32_t vdat = frame_1;
  vdat <<= 8;
  vdat |= data[330];

  return vdat * 2.5 * 2 / 4096.0; 
}

int32_t RawData::parseCksumst(const uint8_t* data) {
  if(data[352] == 0x00) {
    return 0;
  }
  if(data[352] == 0x01) {
    return 1;
  }
}

float RawData::parseMancErr1Per(const uint8_t* data) {
  uint16_t frame_1 = data[353];
  frame_1 <<= 8;
  frame_1 |= data[354];

  float value = frame_1 * 100.0 / 65536;
  return value;
}

float RawData::parseMancErr2Per(const uint8_t* data) {
  uint16_t frame_1 = data[355];
  frame_1 <<= 8;
  frame_1 |= data[356];

  float value = frame_1 * 100.0 / 65536;
  return value;
}

bool RawData::parsePPSLock(const uint8_t* data) {
  uint8_t frame = data[357];
  uint8_t value = GetSubByte(frame, 0, 1);
  if(value == 1) {
    return true;
  }
  else {
    return false;
  }
}

bool RawData::parseGprmcLock(const uint8_t* data) {
  uint8_t frame = data[357];
  frame >>= 1;
  uint8_t value = GetSubByte(frame, 0, 1);
  if(value == 1) {
    return true;
  }
  else {
    return false;
  }
}

bool RawData::parseUTCLock(const uint8_t* data) {
  uint8_t frame = data[357];
  frame >>= 2;
  uint8_t value = GetSubByte(frame, 0, 1);
  if(value == 1) {
    return true;
  }
  else {
    return false;
  }
}


float RawData::parseBotTemperature1(const uint8_t* data) {
  uint16_t frame_1 = data[358];
  uint8_t frame_2 = data[359];
  frame_2 >>= 3;
  frame_2 <<= 3;
  frame_1 |= frame_2;
  uint8_t symbol = GetSubByte(data[358], 7, 1);
  double value = 0;
  if(symbol == 0) {
    value = frame_1 / 16.0;
  }
  else {
    value = (-1) * (8192 - frame_1) / 16.0;
  }

  return value;
}

float RawData::parseBottemperature2(const uint8_t* data) {
  uint16_t frame_1 = data[360];
  uint8_t frame_2 = data[361];
  frame_2 >>= 3;
  frame_2 <<= 3;
  frame_1 |= frame_2;
  uint8_t symbol = GetSubByte(data[360], 7, 1);
  double value = 0;
  if(symbol == 0) {
    value = frame_1 / 16.0;
  }
  else {
    value = (-1) * (8192 - frame_1) / 16.0;
  }

  return value;
}

float RawData::parseTopTemperature1(const uint8_t* data) {
  uint16_t frame_1 = data[362];
  uint8_t frame_2 = data[363];
  frame_2 >>= 3;
  frame_2 <<= 3;
  frame_1 |= frame_2;
  uint8_t symbol = GetSubByte(data[362], 7, 1);
  double value = 0;
  if(symbol == 0) {
    value = frame_1 / 16.0;
  }
  else {
    value = (-1) * (8192 - frame_1) / 16.0;
  }

  return value;
}

float RawData::parseTopTemperature2(const uint8_t* data) {
  uint16_t frame_1 = data[364];
  uint8_t frame_2 = data[365];
  frame_2 >>= 3;
  frame_2 <<= 3;
  frame_1 |= frame_2;
  uint8_t symbol = GetSubByte(data[364], 7, 1);
  double value = 0;
  if(symbol == 0) {
    value = frame_1 / 16.0;
  }
  else {
    value = (-1) * (8192 - frame_1) / 16.0;
  }

  return value;
}

float RawData::parseBotTemperature5(const uint8_t* data) {
  uint8_t frame_1 = data[366];
  frame_1 <<= 4;
  uint8_t symbol = GetSubByte(frame_1, 7, 1);
  frame_1 >>= 4;
  uint16_t frame = frame_1;
  uint8_t frame_2 = data[367];
  frame |= frame_2;

  float value = 0;
  if(symbol == 0) {
    value = frame / 4.0;
  }
  else {
    value = (-1) * (4096 - frame) / 4.0;
  }

  return value;
}

uint8_t RawData::GetSubByte(const uint8_t& value, const int32_t start_pos, const int32_t length) {
  if (start_pos > BYTE_LENGTH - 1 || start_pos < 0 || length < 1) {
      return 0x00;
  }
  int32_t end_pos = std::min(start_pos + length - 1, BYTE_LENGTH - 1);
  int32_t real_len = end_pos + 1 - start_pos;
  uint8_t result = value >> start_pos;
  result &= RANG_MASK_1_L[real_len - 1];
  return result;
}


std::string RawData::ByteToHex(const uint8_t& value) {
    uint8_t high = value >> 4;
    uint8_t low = value & 0x0F;
    std::string result = "";
    result += HEX[high];
    result += HEX[low];
    return result;
}

std::string RawData::ByteToHex(const uint32_t& value) {
    uint8_t high = (value >> 8) & 0xFF;
    uint8_t low = value & 0xFF;
    std::string result = "";
    result += ByteToHex(high);
    result += ByteToHex(low);
    return result;
}


float RawData::pixelToDistance(int pixelValue, int passageway)
{
  float DistanceValue;
  int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
  if (pixelValue <= g_ChannelNum[passageway][indexTemper])
  {
    DistanceValue = 0.0;
  }
  else
  {
    DistanceValue = (float)(pixelValue - g_ChannelNum[passageway][indexTemper]);
  }
  return DistanceValue;
}

int RawData::correctAzimuth(float azimuth_f, int passageway)
{
  int azimuth;
  if (azimuth_f > 0.0 && azimuth_f < 3000.0)
  {
    azimuth_f = azimuth_f + HORI_ANGLE[passageway] + 36000.0f;
  }
  else
  {
    azimuth_f = azimuth_f + HORI_ANGLE[passageway];
  }
  azimuth = (int)azimuth_f;
  azimuth %= 36000;

  return azimuth;
}

//------------------------------------------------------------
//校准反射强度值
float RawData::calibrateIntensity(float intensity, int calIdx, int distance)
{
  int algDist;
  int sDist;
  int uplimitDist;
  float realPwr;
  float refPwr;
  float tempInten;
  float distance_f;
  float endOfSection1;

  int temp = estimateTemperature(temper);

  realPwr = std::max((float)(intensity / (1 + (temp - TEMPERATURE_MIN) / 24.0f)), 1.0f);
  // realPwr = intensity;

  // transform the one byte intensity value to two byte
  if ((int)realPwr < 126)
    realPwr = realPwr * 4.0f;
  else if ((int)realPwr >= 126 && (int)realPwr < 226)
    realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
  else
    realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;

  int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
  uplimitDist = g_ChannelNum[calIdx][indexTemper] + 20000;
  // limit sDist
  sDist = (distance > g_ChannelNum[calIdx][indexTemper]) ? distance : g_ChannelNum[calIdx][indexTemper];
  sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
  // minus the static offset (this data is For the intensity cal useage only)
  algDist = sDist - g_ChannelNum[calIdx][indexTemper];

  // calculate intensity ref curves
  float refPwr_temp = 0.0f;
  int order = 3;
  endOfSection1 = 500.0f;
  distance_f = (float)algDist;
  if (distance_f <= endOfSection1)
  {
    refPwr_temp =
        aIntensityCal[0][calIdx] * exp(aIntensityCal[1][calIdx] - aIntensityCal[2][calIdx] * distance_f / 100.0f) +
        aIntensityCal[3][calIdx];
    //   printf("a-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
  }
  else
  {
    for (int i = 0; i < order; i++)
    {
      refPwr_temp += aIntensityCal[i + 4][calIdx] * (pow(distance_f / 100.0f, order - 1 - i));
    }
    // printf("b-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
  }

  refPwr = std::max(std::min(refPwr_temp, 500.0f), 4.0f);

  tempInten = (51 * refPwr) / realPwr;
  if (numOfLasers == 32)
  {
    tempInten = tempInten * CurvesRate[calIdx];
  }
  tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
  return tempInten;
}

//------------------------------------------------------------
//校准反射强度值 old
float RawData::calibrateIntensity_old(float intensity, int calIdx, int distance)
{
  int algDist;
  int sDist;
  int uplimitDist;
  float realPwr;
  float refPwr;
  float tempInten;

  int temp = estimateTemperature(temper);
  realPwr = std::max((float)(intensity / (1 + (temp - TEMPERATURE_MIN) / 24.0f)), 1.0f);
  // realPwr = intensity;

  if ((int)realPwr < 126)
    realPwr = realPwr * 4.0f;
  else if ((int)realPwr >= 126 && (int)realPwr < 226)
    realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
  else
    realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;

  int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
  uplimitDist = g_ChannelNum[calIdx][indexTemper] + 1400;
  sDist = (distance > g_ChannelNum[calIdx][indexTemper]) ? distance : g_ChannelNum[calIdx][indexTemper];
  sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
  // minus the static offset (this data is For the intensity cal useage only)
  algDist = sDist - g_ChannelNum[calIdx][indexTemper];
  // algDist = algDist < 1400? algDist : 1399;
  refPwr = aIntensityCal_old[algDist][calIdx];

  tempInten = (51 * refPwr) / realPwr;
  if (numOfLasers == 32)
  {
    tempInten = tempInten * CurvesRate[calIdx];
  }
  tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
  return tempInten;
}

//------------------------------------------------------------
int RawData::isABPacket(int distance)
{
  int ABflag = 0;
  if ((distance & 32768) != 0)
  {
    ABflag = 1;  // B
  }
  else
  {
    ABflag = 0;  // A
  }
  return ABflag;
}

//------------------------------------------------------------
float RawData::computeTemperature(unsigned char bit1, unsigned char bit2)
{
  float Temp;
  float bitneg = bit2 & 128;   // 10000000
  float highbit = bit2 & 127;  // 01111111
  float lowbit = bit1 >> 3;
  if (bitneg == 128)
  {
    Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
  }
  else
  {
    Temp = (highbit * 32 + lowbit) * 0.0625f;
  }

  return Temp;
}

//------------------------------------------------------------
int RawData::estimateTemperature(float Temper)
{
  int temp = (int)floor(Temper + 0.5);
  if (temp < TEMPERATURE_MIN)
  {
    temp = TEMPERATURE_MIN;
  }
  else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE)
  {
    temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
  }

  return temp;
}
//------------------------------------------------------------

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  if (numOfLasers == 32)
  {
    unpack_RS32(pkt, pointcloud);
    return;
  }
  float azimuth;  // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[42];

  for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num++)  // 1 packet:12 data blocks
  {
    if (UPPER_BANK != raw->blocks[block].header)
    {
      ROS_INFO_STREAM_THROTTLE(180, "skipping RSLIDAR DIFOP packet");
      break;
    }

    if (tempPacketNum < 20000 && tempPacketNum > 0)  // update temperature information per 20000 packets
    {
      tempPacketNum++;
    }
    else
    {
      temper = computeTemperature(pkt.data[38], pkt.data[39]);
      // ROS_INFO_STREAM("Temp is: " << temper);
      tempPacketNum = 1;
    }

    azimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

    if (block < (BLOCKS_PER_PACKET - 1))  // 12
    {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block + 1].rotation_1 + raw->blocks[block + 1].rotation_2;
      azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

      // Ingnore the block if the azimuth change abnormal
      if (azimuth_diff <= 0.0 || azimuth_diff > 75.0)
      {
        continue;
      }
    }
    else
    {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

      // Ingnore the block if the azimuth change abnormal
      if (azimuth_diff <= 0.0 || azimuth_diff > 75.0)
      {
        continue;
      }
    }

    for (int firing = 0, k = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++)  // 2
    {
      for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
      {
        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) /
                                         RS16_BLOCK_TDURATION);
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;  // convert to integral value...

        union two_bytes tmp;
        tmp.bytes[1] = raw->blocks[block].data[k];
        tmp.bytes[0] = raw->blocks[block].data[k + 1];
        int distance = tmp.uint;

        // read intensity
        intensity = raw->blocks[block].data[k + 2];
        if (Curvesis_new)
          intensity = calibrateIntensity(intensity, dsr, distance);
        else
          intensity = calibrateIntensity_old(intensity, dsr, distance);

        float distance2 = pixelToDistance(distance, dsr);
        distance2 = distance2 * DISTANCE_RESOLUTION;

        float arg_horiz = (float)azimuth_corrected / 18000.0f * M_PI;
        float arg_vert = VERT_ANGLE[dsr];
        pcl::PointXYZI point;

        if (distance2 > DISTANCE_MAX || distance2 < DISTANCE_MIN)  // invalid data
        {
          point.x = NAN;
          point.y = NAN;
          point.z = NAN;
          point.intensity = 0;
          pointcloud->at(2 * this->block_num + firing, dsr) = point;
        }
        else
        {
          // If you want to fix the rslidar Y aixs to the front side of the cable, please use the two line below
          // point.x = dis * cos(arg_vert) * sin(arg_horiz);
          // point.y = dis * cos(arg_vert) * cos(arg_horiz);

          // If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below
          point.y = -distance2 * cos(arg_vert) * sin(arg_horiz);
          point.x = distance2 * cos(arg_vert) * cos(arg_horiz);
          point.z = distance2 * sin(arg_vert);
          point.intensity = intensity;
          pointcloud->at(2 * this->block_num + firing, dsr) = point;
        }
      }
    }
  }
}

void RawData::unpack_RS32(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  float azimuth;  // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[42];

  for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num++)  // 1 packet:12 data blocks
  {
    if (UPPER_BANK != raw->blocks[block].header)
    {
      ROS_INFO_STREAM_THROTTLE(180, "skipping RSLIDAR DIFOP packet");
      break;
    }

    if (tempPacketNum < 20000 && tempPacketNum > 0)  // update temperature information per 20000 packets
    {
      tempPacketNum++;
    }
    else
    {
      temper = computeTemperature(pkt.data[38], pkt.data[39]);
      // ROS_INFO_STREAM("Temp is: " << temper);
      tempPacketNum = 1;
    }

    azimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

    if (block < (BLOCKS_PER_PACKET - 1))  // 12
    {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block + 1].rotation_1 + raw->blocks[block + 1].rotation_2;
      azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

      // Ingnore the block if the azimuth change abnormal
      if (azimuth_diff <= 0.0 || azimuth_diff > 25.0)
      {
        continue;
      }
    }
    else
    {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

      // Ingnore the block if the azimuth change abnormal
      if (azimuth_diff <= 0.0 || azimuth_diff > 25.0)
      {
        continue;
      }
    }

    // Estimate the type of packet
    union two_bytes tmp_flag;
    tmp_flag.bytes[1] = raw->blocks[block].data[0];
    tmp_flag.bytes[0] = raw->blocks[block].data[1];
    int ABflag = isABPacket(tmp_flag.uint);

    int k = 0;
    int index;
    for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK; dsr++, k += RAW_SCAN_SIZE)  // 16   3
    {
      if (ABflag == 1 && dsr < 16)
      {
        index = k + 48;
      }
      else if (ABflag == 1 && dsr >= 16)
      {
        index = k - 48;
      }
      else
      {
        index = k;
      }

      int dsr_temp;
      if (dsr >= 16)
      {
        dsr_temp = dsr - 16;
      }
      else
      {
        dsr_temp = dsr;
      }
      azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr_temp * RS32_DSR_TOFFSET)) / RS32_BLOCK_TDURATION);
      azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);

      union two_bytes tmp;
      tmp.bytes[1] = raw->blocks[block].data[index];
      tmp.bytes[0] = raw->blocks[block].data[index + 1];
      int ab_flag_in_block = isABPacket(tmp.uint);
      int distance = tmp.uint - ab_flag_in_block * 32768;

      // read intensity
      intensity = (float)raw->blocks[block].data[index + 2];
      if (Curvesis_new)
        intensity = calibrateIntensity(intensity, dsr, distance);
      else
        intensity = calibrateIntensity_old(intensity, dsr, distance);

      float distance2 = pixelToDistance(distance, dsr);
      distance2 = distance2 * DISTANCE_RESOLUTION;

      float arg_horiz = (float)azimuth_corrected / 18000.0f * M_PI;
      float arg_vert = VERT_ANGLE[dsr];
      pcl::PointXYZI point;

      if (distance2 > DISTANCE_MAX || distance2 < DISTANCE_MIN)  // invalid data
      {
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = 0;
        pointcloud->at(this->block_num, dsr) = point;
      }
      else
      {
        // If you want to fix the rslidar Y aixs to the front side of the cable, please use the two line below
        // point.x = dis * cos(arg_vert) * sin(arg_horiz);
        // point.y = dis * cos(arg_vert) * cos(arg_horiz);

        // If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below
        point.y = -distance2 * cos(arg_vert) * sin(arg_horiz);
        point.x = distance2 * cos(arg_vert) * cos(arg_horiz);
        point.z = distance2 * sin(arg_vert);
        point.intensity = intensity;
        pointcloud->at(this->block_num, dsr) = point;
      }
    }
  }
}

}  // namespace rs_pointcloud
