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

#include <semantic_map_common/Action.h>
#include <semantic_map_common/Map.h>
#include <semantic_map_common/Mission.h>
#include <semantic_map_common/Object.h>
#include <semantic_map_common/ObjectProperty.h>

#include "semantic_map_common/Entity.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Entity::Entity() {
}

Entity::Entity(const Entity& src) :
  impl_(src.impl_) {
}

Entity::~Entity() {  
}

Entity::Impl::Impl(const std::string& identifier, const std::string& type,
    const Entity& parent) :
  identifier_(identifier),
  type_(type),
  parent_(new Entity(parent)) {
  BOOST_ASSERT(!identifier.empty());
  BOOST_ASSERT(!type.empty());
}

// Entity::Impl::Impl(const XmlRpc::XmlRpcValue& value, const Entity& parent) :
//   parent_(new Entity(parent)) {
//   std::string identifier, type;
//   
//   try {
//     identifier = (std::string)const_cast<XmlRpc::XmlRpcValue&>(value)["id"];
//     type = (std::string)const_cast<XmlRpc::XmlRpcValue&>(value)["type"];
//   }
//   catch (const XmlRpc::XmlRpcException& exception) {
//     throw XmlRpcConversionFailed(exception.getMessage());
//   }  
//   
//   BOOST_ASSERT(!identifier.empty());
//   BOOST_ASSERT(!type.empty());
//   
//   const_cast<std::string&>(identifier_) = identifier;
//   const_cast<std::string&>(type_) = type;
// }

Entity::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string Entity::getIdentifier() const {
  if (impl_.get())
    return impl_->identifier_;
  else
    return std::string();
}

void Entity::setType(const std::string& type) {
  BOOST_ASSERT(!type.empty());
  
  if (impl_.get())
    impl_->type_ = type;
}

std::string Entity::getType() const {
  if (impl_.get())
    return impl_->type_;
  else
    return std::string();
}

Entity Entity::getParent() const {
  if (impl_.get())
    return *impl_->parent_;
  else
    return Entity();
}

size_t Entity::getNumProperties() const {
  if (impl_.get())
    return impl_->properties_.size();
  else
    return 0;
}

boost::unordered_multimap<std::string, Property> Entity::getProperties()
    const {
  if (impl_.get())
    return impl_->properties_;
  else
    return boost::unordered_multimap<std::string, Property>();
}

std::list<Property> Entity::getProperties(const std::string& identifier)
    const {
  std::list<Property> properties;
      
  if (impl_.get()) {
    std::pair<boost::unordered_multimap<std::string, Property>::
      const_iterator, boost::unordered_multimap<std::string, Property>::
      const_iterator> range = impl_->properties_.equal_range(identifier);
    
    for (boost::unordered_multimap<std::string, Property>::const_iterator
        jt = range.first; jt != range.second; ++jt)
      properties.push_back(jt->second);
  }
  
  return properties;
}

bool Entity::hasParent() const {
  if (impl_.get())
    return impl_->parent_.get();
  else
    return false;
}

bool Entity::isMap() const {
  if (impl_.get())
    return boost::dynamic_pointer_cast<Map::Impl>(impl_).get();
  else
    return false;
}

bool Entity::isMission() const {
  if (impl_.get())
    return boost::dynamic_pointer_cast<Mission::Impl>(impl_).get();
  else
    return false;
}

bool Entity::isObject() const {
  if (impl_.get())
    return boost::dynamic_pointer_cast<Object::Impl>(impl_).get();
  else
    return false;
}

bool Entity::isAction() const {
  if (impl_.get())
    return boost::dynamic_pointer_cast<Action::Impl>(impl_).get();
  else
    return false;
}

bool Entity::isValid() const {
  return impl_.get();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

ObjectProperty Entity::addProperty(const std::string& identifier, const
    Entity& object) {
  if (impl_.get()) {
    ObjectProperty property(identifier, *this, object);
    
    impl_->properties_.insert(std::make_pair(identifier, property));
    
    return property;
  }
  else
    return ObjectProperty();
}

void Entity::clearProperties() {
  if (impl_.get())
    impl_->properties_.clear();
}

// XmlRpc::XmlRpcValue Entity::toXmlRpcValue() const {
//   XmlRpc::XmlRpcValue value;
//   
//   if (impl_.get()) {
//     try {
//       value["id"] = getIdentifier();
//       value["type"] = getType();
//     }
//     catch (const XmlRpc::XmlRpcException& exception) {
//       throw XmlRpcConversionFailed(exception.getMessage());
//     }
//   }
//   
//   return value;
// }

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

bool Entity::operator==(const Entity& entity) const {
  if (isValid() && entity.isValid()) {
    if ((getIdentifier() == entity.getIdentifier()) && (getType() == src.getType())) ;
  }
  else
    return (isValid() == entity.isValid());
}

}
