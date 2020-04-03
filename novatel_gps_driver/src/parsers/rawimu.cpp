// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <novatel_gps_driver/parsers/rawimu.h>
#include <novatel_gps_driver/parsers/header.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::RawImuParser::MESSAGE_NAME = "RAWIMU";

uint32_t novatel_gps_driver::RawImuParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::RawImuParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::NovatelRawImuPtr
novatel_gps_driver::RawImuParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected rawimu message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::NovatelRawImuPtr ros_msg = boost::make_shared<novatel_gps_msgs::NovatelRawImu>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = "RAWIMU";

  ros_msg->gps_week_num = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->z_acceleration = ParseInt32(&bin_msg.data_[16]);
  ros_msg->y_acceleration = ParseInt32(&bin_msg.data_[20]);
  ros_msg->x_acceleration = ParseInt32(&bin_msg.data_[24]);
  ros_msg->yaw_rate = ParseInt32(&bin_msg.data_[28]);
  ros_msg->roll_rate = ParseInt32(&bin_msg.data_[32]);
  ros_msg->pitch_rate = ParseInt32(&bin_msg.data_[36]);

  return ros_msg;
}

novatel_gps_msgs::NovatelRawImuPtr
novatel_gps_driver::RawImuParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) noexcept(false)
{
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in RAWIMU log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::NovatelRawImuPtr msg = boost::make_shared<novatel_gps_msgs::NovatelRawImu>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseUInt32(sentence.body[0], msg->gps_week_num);
  valid &= ParseDouble(sentence.body[1], msg->gps_seconds);
  valid &= ParseInt32(sentence.body[3], msg->z_acceleration);
  valid &= ParseInt32(sentence.body[4], msg->y_acceleration);
  valid &= ParseInt32(sentence.body[5], msg->x_acceleration);
  valid &= ParseInt32(sentence.body[6], msg->yaw_rate);
  valid &= ParseInt32(sentence.body[7], msg->roll_rate);
  valid &= ParseInt32(sentence.body[8], msg->pitch_rate);

  if (!valid)
  {
    throw ParseException("Error parsing RAWIMU log.");
  }

  return msg;
}
