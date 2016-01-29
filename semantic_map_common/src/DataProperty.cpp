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

#include <boost/lexical_cast.hpp>

#include <XmlRpcException.h>

#include <semantic_map_common/Map.h>

#include "semantic_map_common/DataProperty.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

DataProperty::XmlRpcConversionFailed::XmlRpcConversionFailed(const
    std::string& description) :
  ros::Exception("XML-RPC value conversion of semantic map data property "
    "failed: "+description) {
}

DataProperty::DataProperty() {
}

DataProperty::DataProperty(const DataProperty& src) :
  Property(src) {
}

DataProperty::DataProperty(const Property& src) :
  Property(src) {
  if (impl_.get())
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl_).get());
}

DataProperty::~DataProperty() {  
}

DataProperty::Impl::Impl(const std::string& identifier, const Entity&
    subject, ValueType valueType, const std::string& value) :
  Property::Impl(identifier, subject),
  valueType_(valueType),
  value_(value) {
  BOOST_ASSERT(valueType != Invalid);
}

DataProperty::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

DataProperty::ValueType DataProperty::getValueType() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->valueType_;
  else
    return Invalid;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void DataProperty::clearValue() {
  if (impl_.get()) {
    boost::static_pointer_cast<Impl>(impl_)->valueType_ = Invalid;
    boost::static_pointer_cast<Impl>(impl_)->value_.clear();
  }
}

void DataProperty::fromXmlRpcValue(const XmlRpc::XmlRpcValue& value, const
    Map& map) {
  try {
    std::string identifier = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
      value)["id"];
    std::string subject = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
      value)["subject"];
      
//     if (impl_.get()) {
//       setIdentifier(identifier);
//       setSubject(map.getEntity(subject));
//     }
//     else
//       impl_.reset(new Impl(identifier, map.getEntity(subject)));
      
    clearValue();
    if (value.hasMember("value")) {
      XmlRpc::XmlRpcValue::Type type = const_cast<XmlRpc::XmlRpcValue&>(
        value)["value"].getType();
      
      if (type == XmlRpc::XmlRpcValue::TypeString)
        setValue<std::string>(const_cast<XmlRpc::XmlRpcValue&>(
          value)["value"]);
      else if (type == XmlRpc::XmlRpcValue::TypeBoolean)
        setValue<bool>(const_cast<XmlRpc::XmlRpcValue&>(value)["value"]);
      else if (type == XmlRpc::XmlRpcValue::TypeDouble)
        setValue<double>(const_cast<XmlRpc::XmlRpcValue&>(value)["value"]);
      else if (type == XmlRpc::XmlRpcValue::TypeInt)
        setValue<int>(const_cast<XmlRpc::XmlRpcValue&>(value)["value"]);
    }
  }
  catch (const XmlRpc::XmlRpcException& exception) {
    throw XmlRpcConversionFailed(exception.getMessage());
  }
}

void DataProperty::toXmlRpcValue(XmlRpc::XmlRpcValue& value) const {
  if (impl_.get()) {
    try {
      value["id"] = getIdentifier();
      value["subject"] = getSubject().getIdentifier();
      
      ValueType valueType = getValueType();
      if (valueType == String)
        value["value"] = getValue<std::string>();
      else if (valueType == Boolean)
        value["value"] = getValue<bool>();
      else if (valueType == Float)
        value["value"] = getValue<double>();
      else if (valueType == Integer)
        value["value"] = getValue<int>();
      else
        value["value"] = XmlRpc::XmlRpcValue();      
    }
    catch (const XmlRpc::XmlRpcException& exception) {
      throw XmlRpcConversionFailed(exception.getMessage());
    }
  }
}

void DataProperty::fromMessage(const semantic_map_msgs::DataProperty&
    message, const Map& map) {
//   if (impl_.get()) {
//     setIdentifier(message.id);
//     setSubject(map.getEntity(message.subject));
//   }
//   else
//     impl_.reset(new Impl(message.id, map.getEntity(message.subject)));
  
  if (message.value_type == semantic_map_msgs::DataProperty::
      VALUE_TYPE_STRING)
    setValue<std::string>(message.value);
  else if (message.value_type == semantic_map_msgs::DataProperty::
      VALUE_TYPE_BOOL)
    setValue<bool>(boost::lexical_cast<bool>(message.value));
  else if (message.value_type == semantic_map_msgs::DataProperty::
      VALUE_TYPE_FLOAT)
    setValue<double>(boost::lexical_cast<double>(message.value));
  else if (message.value_type == semantic_map_msgs::DataProperty::
      VALUE_TYPE_INT)
    setValue<int>(boost::lexical_cast<int>(message.value));
  else
    clearValue();
}

semantic_map_msgs::DataProperty DataProperty::toMessage() const {
  semantic_map_msgs::DataProperty message;
  
  message.id = getIdentifier();
  message.subject = getSubject().getIdentifier();
  
  ValueType valueType = getValueType();
  if (valueType == String)
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_STRING;
  else if (valueType == Boolean)
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_BOOL;
  else if (valueType == Float)
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_FLOAT;
  else if (valueType == Integer)
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_INT;
  else
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_INVALID;
  
  message.value = getValue<std::string>();
  
  return message;
}

}
