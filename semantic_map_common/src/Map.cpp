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

#include "semantic_map_common/Map.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Map::Map() {
}

Map::Map(const std::string& identifier, const std::string& type, const
    std::string& ns, const std::string& frame, const ros::Time& stamp) {
  impl_.reset(new Impl(identifier, type, ns, frame, stamp));
}

Map::Map(const Map& src) :
  Entity(src) {
}

Map::Map(const Entity& src) :
  Entity(src) {
  if (impl_.get())
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl_).get());
}

Map::~Map() {
}

Map::Impl::Impl(const std::string& identifier, const std::string& type, const
    std::string& ns, const std::string& frame, const ros::Time& stamp) :
  Entity::Impl(identifier, type),
  ontology_(ns),
  frame_(frame),
  stamp_(stamp) {
}

Map::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Map::setOntology(const Ontology& ontology) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->ontology_ = ontology;
}

Ontology Map::getOntology() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->ontology_;
  else
    return Ontology();
}

void Map::setFrame(const std::string& frame) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->frame_ = frame;
}

std::string Map::getFrame() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->frame_;
  else
    return std::string();
}

void Map::setStamp(const ros::Time& stamp) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->stamp_ = stamp;
}

ros::Time Map::getStamp() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->stamp_;
  else
    return ros::Time();
}

void Map::setAddress(const Address& address) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->address_ = address;
}

Address Map::getAddress() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->address_;
  else
    return Address();
}

size_t Map::getNumObjects() const {
  size_t numObjects = 0;
  
  if (impl_.get()) {
    numObjects = boost::static_pointer_cast<Impl>(impl_)->objects_.size();
      
    for (boost::unordered_map<std::string, Object>::const_iterator
        it = begin(); it != end(); ++it)
      numObjects += it->second.getNumParts();    
  }
  
  return numObjects;
}

boost::unordered_map<std::string, Object> Map::getObjects() const {
  boost::unordered_map<std::string, Object> objects;
  
  if (impl_.get()) {
    objects = boost::static_pointer_cast<Impl>(impl_)->objects_;
    
    for (boost::unordered_map<std::string, Object>::const_iterator
        it = begin(); it != end(); ++it) {
      boost::unordered_map<std::string, Object> parts = it->second.getParts();
      
      for (boost::unordered_map<std::string, Object>::const_iterator
          jt = parts.begin(); jt != parts.end(); ++jt)
        objects.insert(*jt);
    }
  }

  return objects;
}

Object Map::getObject(const std::string& identifier) const {
  if (impl_.get()) {
    boost::unordered_map<std::string, Object>::const_iterator it =
      boost::static_pointer_cast<Impl>(impl_)->objects_.find(identifier);
      
    if (it == boost::static_pointer_cast<Impl>(impl_)->objects_.end()) {
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

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

boost::unordered_map<std::string, Object>::iterator Map::begin() {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->objects_.begin();
  else
    return boost::unordered_map<std::string, Object>::iterator();
}

boost::unordered_map<std::string, Object>::const_iterator Map::begin() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->objects_.begin();
  else
    return boost::unordered_map<std::string, Object>::const_iterator();
}

boost::unordered_map<std::string, Object>::iterator Map::end() {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->objects_.end();
  else
    return boost::unordered_map<std::string, Object>::iterator();
}

boost::unordered_map<std::string, Object>::const_iterator Map::end() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->objects_.end();
  else
    return boost::unordered_map<std::string, Object>::const_iterator();
}

Object Map::addObject(const std::string& identifier, const std::string& type) {
  if (impl_.get()) {
    Object object(identifier, type, *this);
    
    boost::static_pointer_cast<Impl>(impl_)->objects_.insert(
      std::make_pair(identifier, object));
    
    return object;
  }
  else
    return Object();
}

void Map::clearObjects() {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->objects_.clear();
}

// void Map::fromXmlRpcValue(const XmlRpc::XmlRpcValue& value) {
//   try {
//     if (value.hasMember("id"))
//       identifier_ = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
//         value)["id"];
//     else
//       identifier_ = "Map";
//     
//     if (value.hasMember("namespace"))
//       ns_ = (std::string)const_cast<XmlRpc::XmlRpcValue&>(value)[
//         "namespace"];
//     else
//       ns_ = "http://ros.org/semantic_map/map.owl";
//     
//     if (value.hasMember("frame_id"))
//       frame_ = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
//         value)["frame_id"];
//     else
//       frame_ = "/map";
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
//       stamp_.fromBoost(stamp);
//     }
//     else
//       stamp_ = ros::Time::now();
//     
//     prefixes_.clear();    
//     if (value.hasMember("prefixes")) {
//       XmlRpc::XmlRpcValue& prefixes = const_cast<XmlRpc::XmlRpcValue&>(
//         value)["prefixes"];
//       
//       for (size_t index = 0; index < prefixes.size(); ++index) {
//         prefixes_.insert(std::make_pair(prefixes[index]["name"],
//           prefixes[index]["prefix"]));
//       }
//     }
//     
//     imports_.clear();
//     if (value.hasMember("imports")) {
//       XmlRpc::XmlRpcValue& imports = const_cast<XmlRpc::XmlRpcValue&>(
//         value)["imports"];
//       
//       for (size_t index = 0; index < imports.size(); ++index) {
//         imports_.push_back(imports[index]);
//       }
//     }
//     
//     if (value.hasMember("address"))
//       address_.fromXmlRpcValue(const_cast<XmlRpc::XmlRpcValue&>(
//         value)["address"]);
//     else
//       address_ = Address();
//     
//     objects_.clear();
//     if (value.hasMember("objects")) {
//       XmlRpc::XmlRpcValue& objects = const_cast<XmlRpc::XmlRpcValue&>(
//         value)["objects"];
//       
//       for (size_t index = 0; index < objects.size(); ++index) {
//         Object object;
//         
//         std::string objectIdentifier = object.fromXmlRpcValue(
//           objects[index]);
//         
//         objects_.insert(std::make_pair(objectIdentifier, object));
//       }
//     }
//   }
//   catch (const XmlRpc::XmlRpcException& exception) {
//     throw XmlRpcConversionFailed(exception.getMessage());
//   }
// }

// void Map::toXmlRpcValue(XmlRpc::XmlRpcValue& value) const {
//   try {
//     value["id"] = identifier_;
//     value["namespace"] = ns_;
//     
//     value["frame_id"] = frame_;
//     value["stamp"] = boost::posix_time::to_iso_extended_string(
//       stamp_.toBoost());
//     
//     size_t prefixIndex = 0;
//     for (Prefixes::const_iterator it = prefixes_.begin();
//         it != prefixes_.end(); ++it, ++prefixIndex) {
//       value["prefixes"][prefixIndex]["name"] = it->first;
//       value["prefixes"][prefixIndex]["prefix"] = it->second;
//     }
//     
//     size_t importIndex = 0;
//     for (Imports::const_iterator it = imports_.begin();
//         it != imports_.end(); ++it, ++importIndex)
//       value["imports"][importIndex] = *it;
//     
//     address_.toXmlRpcValue(value["address"]);
//     
//     size_t objectIndex = 0;    
//     for (Objects::const_iterator it = objects_.begin();
//         it != objects_.end(); ++it, ++objectIndex)
//       it->second.toXmlRpcValue(it->first, value["objects"][objectIndex]);
//   }
//   catch (const XmlRpc::XmlRpcException& exception) {
//     throw XmlRpcConversionFailed(exception.getMessage());
//   }
// }

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

Object Map::operator[](const std::string& identifier) const {
  return getObject(identifier);
}

bool Map::operator==(const Map& map) const {
  if (isValid() && map.isValid()) {
    return true;
  }
  else
    return (isValid() == map.isValid());
}

}
