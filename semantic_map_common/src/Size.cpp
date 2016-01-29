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

#include "semantic_map_common/Size.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Size::XmlRpcConversionFailed::XmlRpcConversionFailed(const std::string&
    description) :
  ros::Exception("XML-RPC value conversion of semantic map size failed: "+
    description) {
}

Size::Size(double width, double height, double depth) {
  dimensions_[0] = depth;
  dimensions_[1] = width;
  dimensions_[2] = height;
}

Size::Size(const Size& src) :
  dimensions_(src.dimensions_) {
}

Size::~Size() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Size::setWidth(double width) {
  dimensions_[1] = width;
}

double Size::getWidth() const {
  return dimensions_[1];
}

void Size::setHeight(double height) {
  dimensions_[2] = height;
}

double Size::getHeight() const {
  return dimensions_[2];
}

void Size::setDepth(double depth) {
  dimensions_[0] = depth;
}

double Size::getDepth() const {
  return dimensions_[0];
}

void Size::setDimensions(const Eigen::Vector3d& dimensions) {
  dimensions_ = dimensions;
}

const Eigen::Vector3d& Size::getDimensions() const {
  return dimensions_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Size::fromXmlRpcValue(const XmlRpc::XmlRpcValue& value) {
  try {
    if (value.hasMember("x"))
      dimensions_[0] = const_cast<XmlRpc::XmlRpcValue&>(value)["x"];
    else
      dimensions_[0] = 0.0;
    
    if (value.hasMember("y"))
      dimensions_[1] = const_cast<XmlRpc::XmlRpcValue&>(value)["y"];
    else
      dimensions_[1] = 0.0;
    
    if (value.hasMember("z"))
      dimensions_[2] = const_cast<XmlRpc::XmlRpcValue&>(value)["z"];
    else
      dimensions_[2] = 0.0;
  }
  catch (const XmlRpc::XmlRpcException& exception) {
    throw XmlRpcConversionFailed(exception.getMessage());
  }
}

void Size::toXmlRpcValue(XmlRpc::XmlRpcValue& value) const {
  try {
    value["x"] = dimensions_[0];
    value["y"] = dimensions_[1];
    value["z"] = dimensions_[2];
  }
  catch (const XmlRpc::XmlRpcException& exception) {
    throw XmlRpcConversionFailed(exception.getMessage());
  }
}

void Size::fromMessage(const semantic_map_msgs::Size& message) {
  dimensions_[0] = message.x;
  dimensions_[1] = message.y;
  dimensions_[2] = message.z;
}

semantic_map_msgs::Size Size::toMessage() const {
  semantic_map_msgs::Size message;
  
  message.x = dimensions_[0];
  message.y = dimensions_[1];
  message.z = dimensions_[2];
  
  return message;
}

}
