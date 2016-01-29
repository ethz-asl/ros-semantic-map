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

#include <sstream>

#include <XmlRpcException.h>

#include "semantic_map_common/Address.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Address::XmlRpcConversionFailed::XmlRpcConversionFailed(const std::string&
    description) :
  ros::Exception("XML-RPC value conversion of semantic map address failed: "+
    description) {
}

Address::Address(const std::string& roomNumber, const std::string&
    floorNumber, const std::string& streetNumber, const std::string&
    streetName, const std::string& cityName) :
  roomNumber_(roomNumber), 
  floorNumber_(floorNumber),
  streetNumber_(streetNumber),
  streetName_(streetName),
  cityName_(cityName) {
}

Address::Address(const Address& src) :
  roomNumber_(src.roomNumber_), 
  floorNumber_(src.floorNumber_),
  streetNumber_(src.streetNumber_),
  streetName_(src.streetName_),
  cityName_(src.cityName_) {
}

Address::~Address() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Address::setStreetName(const std::string& name) {
  streetName_ = name;
}

const std::string& Address::getStreetName() const {
  return streetName_;
}

void Address::setCityName(const std::string& name) {
  cityName_ = name;
}

const std::string& Address::getCityName() const {
  return cityName_;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Address::fromXmlRpcValue(const XmlRpc::XmlRpcValue& value) {
  try {
    if (value.hasMember("room_nr")) {
      std::ostringstream stream;
      const_cast<XmlRpc::XmlRpcValue&>(value)["room_nr"].write(stream);
      
      roomNumber_ = stream.str();
    }
    else
      roomNumber_.clear();
    
    if (value.hasMember("floor_nr")) {
      std::ostringstream stream;
      const_cast<XmlRpc::XmlRpcValue&>(value)["floor_nr"].write(stream);
      
      floorNumber_ = stream.str();
    }
    else
      floorNumber_.clear();
    
    if (value.hasMember("street_nr")) {
      std::ostringstream stream;
      const_cast<XmlRpc::XmlRpcValue&>(value)["street_nr"].write(stream);
      
      streetNumber_ = stream.str();
    }
    else
      streetNumber_.clear();
    
    streetName_ = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
      value)["street_name"];
    cityName_ = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
      value)["city_name"];
  }
  catch (const XmlRpc::XmlRpcException& exception) {
    throw XmlRpcConversionFailed(exception.getMessage());
  }
}

void Address::toXmlRpcValue(XmlRpc::XmlRpcValue& value) const {
  try {
    value["room_nr"] = roomNumber_;
    value["floor_nr"] = floorNumber_;
    value["street_nr"] = streetNumber_;
    value["street_name"] = streetName_;
    value["city_name"] = cityName_;
  }
  catch (const XmlRpc::XmlRpcException& exception) {
    throw XmlRpcConversionFailed(exception.getMessage());
  }
}

void Address::fromMessage(const semantic_map_msgs::Address& message) {
  roomNumber_ = message.room_nr;
  floorNumber_ = message.floor_nr;
  streetNumber_ = message.street_nr;
  streetName_ = message.street_name;
  cityName_ = message.city_name;
}

semantic_map_msgs::Address Address::toMessage() const {
  semantic_map_msgs::Address message;
  
  message.room_nr = roomNumber_;
  message.floor_nr = floorNumber_;
  message.street_nr = streetNumber_;
  message.street_name = streetName_;
  message.city_name = cityName_;
  
  return message;
}

}
