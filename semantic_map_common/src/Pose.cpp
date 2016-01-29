/******************************************************************************
 * Copyright (C) 2016 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include <XmlRpcException.h>

#include "semantic_map_common/Pose.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Pose::XmlRpcConversionFailed::XmlRpcConversionFailed(const std::string&
    description) :
  ros::Exception("XML-RPC value conversion of semantic map pose failed: "+
    description) {
}

Pose::Pose(double x, double y, double z, double i, double j, double k,
    double w) {
  position_[0] = x;
  position_[1] = y;
  position_[2] = z;
  
  orientation_.x() = i;
  orientation_.y() = j;
  orientation_.z() = k;
  orientation_.w() = w;
}

Pose::Pose(const Pose& src) :
  position_(src.position_),
  orientation_(src.orientation_) {
}

Pose::~Pose() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Pose::setPosition(const Eigen::Vector3d& position) {
  position_ = position;
}

const Eigen::Vector3d& Pose::getPosition() const {
  return position_;
}

void Pose::setOrientation(const Eigen::Quaterniond& orientation) {
  orientation_ = orientation;
}

const Eigen::Quaterniond& Pose::getOrientation() const {
  return orientation_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Pose::fromXmlRpcValue(const XmlRpc::XmlRpcValue& value) {
  try {
    if (value.hasMember("position")) {
      XmlRpc::XmlRpcValue& position = const_cast<XmlRpc::XmlRpcValue&>(
        value)["position"];
      
      if (position.hasMember("x"))
        position_[0] = position["x"];
      else
        position_[0] = 0.0;
      
      if (position.hasMember("y"))
        position_[1] = position["y"];
      else
        position_[1] = 0.0;
      
      if (position.hasMember("z"))
        position_[2] = position["z"];
      else
        position_[2] = 0.0;
    }
    else
      position_.setZero();
    
    if (value.hasMember("orientation")) {
      XmlRpc::XmlRpcValue& orientation = const_cast<XmlRpc::XmlRpcValue&>(
        value)["orientation"];
      
      if (orientation.hasMember("x"))
        orientation_.x() = orientation["x"];
      else
        orientation_.x() = 0.0;
      
      if (orientation.hasMember("y"))
        orientation_.y() = orientation["y"];
      else
        orientation_.y() = 0.0;
      
      if (orientation.hasMember("z"))
        orientation_.z() = orientation["z"];
      else
        orientation_.z() = 0.0;
      
      if (orientation.hasMember("w"))
        orientation_.w() = orientation["w"];
      else
        orientation_.w() = 1.0;
      
      orientation_.normalize();
    }
    else
      orientation_.setIdentity();
  }
  catch (const XmlRpc::XmlRpcException& exception) {
    throw XmlRpcConversionFailed(exception.getMessage());
  }
}

void Pose::toXmlRpcValue(XmlRpc::XmlRpcValue& value) const {
  try {
    value["position"]["x"] = position_[0];
    value["position"]["y"] = position_[1];
    value["position"]["z"] = position_[2];
    
    value["orientation"]["x"] = orientation_.x();
    value["orientation"]["y"] = orientation_.y();
    value["orientation"]["z"] = orientation_.z();
    value["orientation"]["w"] = orientation_.w();
  }
  catch (const XmlRpc::XmlRpcException& exception) {
    throw XmlRpcConversionFailed(exception.getMessage());
  }
}

void Pose::fromMessage(const geometry_msgs::Pose& message) {
  position_[0] = message.position.x;
  position_[1] = message.position.y;
  position_[2] = message.position.z;
  
  orientation_.x() = message.orientation.x;
  orientation_.y() = message.orientation.y;
  orientation_.z() = message.orientation.z;
  orientation_.w() = message.orientation.w;
}

geometry_msgs::Pose Pose::toMessage() const {
  geometry_msgs::Pose message;
  
  message.position.x = position_[0];
  message.position.y = position_[1];
  message.position.z = position_[2];
  
  message.orientation.x = orientation_.x();
  message.orientation.y = orientation_.y();
  message.orientation.z = orientation_.z();
  message.orientation.w = orientation_.w();
  
  return message;
}

}
