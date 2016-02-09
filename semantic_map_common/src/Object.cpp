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

#include <boost/date_time.hpp>

#include <XmlRpcException.h>

#include <semantic_map_common/ActionOnObject.h>

#include "semantic_map_common/Object.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Object::XmlRpcConversionFailed::XmlRpcConversionFailed(const std::string&
    description) :
  ros::Exception("XML-RPC value conversion of semantic map object failed: "+
    description) {
}

Object::Object() {
}

Object::Object(const std::string& identifier, const std::string& type,
    const Entity& parent) {
  impl_.reset(new Impl(identifier, type, parent));
}

Object::Object(const Object& src) :
  Entity(src) {
}

Object::Object(const Entity& src) :
  Entity(src) {
  if (impl_.get())
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl_).get());
}

Object::~Object() {
}

Object::Impl::Impl(const std::string& identifier, const std::string& type,
    const Entity& parent) :
  Entity::Impl(identifier, type, parent),
  stamp_(ros::Time::now()) {
}

Object::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

Object Object::getParentObject() const {
  Entity parent = getParent();
  
  if (parent.isObject())
    return parent;
  else
    return Object();
}

void Object::setFrame(const std::string& frame) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->frame_ = frame;
}

std::string Object::getFrame() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->frame_;
  else
    return std::string();
}

void Object::setStamp(const ros::Time& stamp) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->stamp_ = stamp;
}

ros::Time Object::getStamp() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->stamp_;
  else
    return ros::Time();
}

void Object::setPose(const Pose& pose) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->pose_ = pose;
}

Pose Object::getPose() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->pose_;
  else
    return Pose();
}

void Object::setSize(const Size& size) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->size_ = size;
}

Size Object::getSize() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->size_;
  else
    return Size();
}

size_t Object::getNumParts() const {
  size_t numParts = 0;
  
  if (impl_.get()) {
    numParts = boost::static_pointer_cast<Impl>(impl_)->parts_.size();
      
    for (boost::unordered_map<std::string, Object>::const_iterator
        it = begin(); it != end(); ++it)
      numParts += it->second.getNumParts();    
  }
  
  return numParts;
}

boost::unordered_map<std::string, Object> Object::getParts() const {
  boost::unordered_map<std::string, Object> parts;
  
  if (impl_.get()) {
    parts = boost::static_pointer_cast<Impl>(impl_)->parts_;
    
    for (boost::unordered_map<std::string, Object>::const_iterator
        it = begin(); it != end(); ++it) {
      boost::unordered_map<std::string, Object> subParts = it->second.
        getParts();
      
      for (boost::unordered_map<std::string, Object>::const_iterator
          jt = subParts.begin(); jt != subParts.end(); ++jt)
        subParts.insert(*jt);
    }
  }

  return parts;
}

Object Object::getPart(const std::string& identifier) const {
  if (impl_.get()) {
    boost::unordered_map<std::string, Object>::const_iterator it =
      boost::static_pointer_cast<Impl>(impl_)->parts_.find(identifier);
      
    if (it == boost::static_pointer_cast<Impl>(impl_)->parts_.end()) {
      for (it = begin(); it != end(); ++it) {
        Object part = it->second.getPart(identifier);
        
        if (part.isValid())
          return part;
      }
    }
    else
      return it->second;
  }

  return Object();
}

size_t Object::getNumActions() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->actions_.size();
  else
    return 0;
}

boost::unordered_map<std::string, ActionOnObject> Object::getActions() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->actions_;
  else
    return boost::unordered_map<std::string, ActionOnObject>();
}

ActionOnObject Object::getAction(const std::string& identifier) const {
  if (impl_.get()) {
    boost::unordered_map<std::string, ActionOnObject>::const_iterator it =
      boost::static_pointer_cast<Impl>(impl_)->actions_.find(identifier);
      
    if (it != boost::static_pointer_cast<Impl>(impl_)->actions_.end())
      return it->second;
  }
  
  return ActionOnObject();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

boost::unordered_map<std::string, Object>::iterator Object::begin() {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->parts_.begin();
  else
    return boost::unordered_map<std::string, Object>::iterator();
}

boost::unordered_map<std::string, Object>::const_iterator Object::begin()
    const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->parts_.begin();
  else
    return boost::unordered_map<std::string, Object>::const_iterator();
}

boost::unordered_map<std::string, Object>::iterator Object::end() {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->parts_.end();
  else
    return boost::unordered_map<std::string, Object>::iterator();
}

boost::unordered_map<std::string, Object>::const_iterator Object::end()
    const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->parts_.end();
  else
    return boost::unordered_map<std::string, Object>::const_iterator();
}

Object Object::addPart(const std::string& identifier, const std::string&
    type) {
  if (impl_.get()) {
    Object part(identifier, type, *this);
    
    boost::static_pointer_cast<Impl>(impl_)->parts_.insert(
      std::make_pair(identifier, part));
    
    return part;
  }
  else
    return Object();
}

void Object::clearParts() {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->parts_.clear();
}

ActionOnObject Object::addAction(const std::string& identifier, const
    std::string& type, bool asserted) {
  if (impl_.get()) {
    ActionOnObject action;
    
    action.impl_.reset(new ActionOnObject::Impl(identifier, type,
      asserted, *this));
    boost::static_pointer_cast<Impl>(impl_)->actions_.insert(
      std::make_pair(identifier, action));
    
    return action;
  }
  else
    return ActionOnObject();
}

void Object::clearActions() {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->actions_.clear();
}

// void Object::fromXmlRpcValue(const XmlRpc::XmlRpcValue& value) {
//   try {    
//     std::string identifier = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
//       value)["id"];
//     std::string type = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
//       value)["type"];
//     
//     if (impl_.get()) {
//       BOOST_ASSERT(identifier == getIdentifier());
//       setType(type);
//     }
//     else
//       impl_.reset(new Impl(identifier, type, *this));
//     
//     if (value.hasMember("frame_id"))
//       setFrame(const_cast<XmlRpc::XmlRpcValue&>(value)["frame_id"]);
//     else
//       setFrame(std::string());
//     
//     if (value.hasMember("stamp")) {
//       boost::posix_time::time_input_facet facet(1);
//       std::istringstream stream(const_cast<XmlRpc::XmlRpcValue&>(
//         value)["stamp"]);
//       boost::posix_time::ptime stamp;
//       
//       facet.set_iso_extended_format();
//       stream.imbue(std::locale(std::locale::classic(), &facet));
//       stream >> stamp;
//       
//       setStamp(ros::Time::fromBoost(stamp));
//     }
//     else
//       setStamp(ros::Time::now());
//     
//     Pose pose;
//     if (value.hasMember("pose"))
//       pose.fromXmlRpcValue(const_cast<XmlRpc::XmlRpcValue&>(value)["pose"]);
//     setPose(pose);
//     
//     Size size;
//     if (value.hasMember("size"))
//       size.fromXmlRpcValue(const_cast<XmlRpc::XmlRpcValue&>(value)["size"]);
//     setSize(size);
//     
//     clearParts();
//     if (value.hasMember("parts")) {
//       XmlRpc::XmlRpcValue& partsValue = const_cast<XmlRpc::XmlRpcValue&>(
//         value)["parts"];
//       
//       for (size_t index = 0; index < partsValue.size(); ++index) {
//         Object part(*this);
//         
//         part.fromXmlRpcValue(partsValue[index]);
//         
//         addPart(part);
//       }
//     }
//   }
//   catch (const XmlRpc::XmlRpcException& exception) {
//     throw XmlRpcConversionFailed(exception.getMessage());
//   }
// }

XmlRpc::XmlRpcValue Object::toXmlRpcValue() const {
  XmlRpc::XmlRpcValue value;
  
  if (impl_.get()) {
    try {
      value["id"] = getIdentifier();
      value["type"] = getType();
      
      value["frame_id"] = getFrame();
      value["stamp"] = boost::posix_time::to_iso_extended_string(
        getStamp().toBoost());
      
//       getPose().toXmlRpcValue(value);
//       getSize().toXmlRpcValue(value["size"]);
      
      value["parts"].setSize(getNumParts());
      size_t partIndex = 0;
//       for (boost::unordered_map<std::string, Object>::const_iterator
//           it = begin(); it != end(); ++it, ++partIndex)
//         it->second.toXmlRpcValue(value["parts"][partIndex]);
    }
    catch (const XmlRpc::XmlRpcException& exception) {
      throw XmlRpcConversionFailed(exception.getMessage());
    }
  }
  
  return value;
}

semantic_map_msgs::Object Object::toMessage() const {
  semantic_map_msgs::Object message;
  
  message.header.frame_id = getFrame();
  message.header.stamp = getStamp();
  
  message.id = getIdentifier();
  message.type = getType();
  
  message.pose = getPose().toMessage();
  message.size = getSize().toMessage();
  
//   message.parts.reserve(getNumParts());
//   for (boost::unordered_map<std::string, Object>::const_iterator
//       it = begin(); it != end(); ++it)
//     message.parts.push_back(it->first);
  
  return message;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

Object Object::operator[](const std::string& identifier) const {
  return getPart(identifier);
}

}
