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

#include <semantic_map_common/Object.h>

#include "semantic_map_common/ActionOnObject.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ActionOnObject::ActionOnObject() {
}

ActionOnObject::ActionOnObject(const ActionOnObject& src) :
  Action(src) {
}

ActionOnObject::ActionOnObject(const Entity& src) :
  Action(src) {
  if (impl_.get())
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl_).get());
}

ActionOnObject::~ActionOnObject() {  
}

ActionOnObject::Impl::Impl(const std::string& identifier, const std::string&
    type, bool asserted, const Object& objectActedOn) :
  Action::Impl(identifier, type, asserted),
  objectActedOn_(objectActedOn) {
  BOOST_ASSERT(objectActedOn.isValid());
}

// ActionOnObject::Impl::Impl(const XmlRpc::XmlRpcValue& value, const
//     boost::unordered_map<std::string, Object>& objects) :
//   Action::Impl(value) {
//   std::string objectActedOn;
//   
//   try {
//     objectActedOn = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
//       value)["object_acted_on"];
//   }
//   catch (const XmlRpc::XmlRpcException& exception) {
//     throw XmlRpcConversionFailed(exception.getMessage());
//   }  
//   
//   boost::unordered_map<std::string, Object>::const_iterator it = objects.
//     find(objectActedOn);
//       
//   BOOST_ASSERT(it != objects.end());
//   BOOST_ASSERT(it->second.isValid());
//   
//   *parent_ = it->second;
// }

ActionOnObject::Impl::~Impl() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

Object ActionOnObject::getObjectActedOn() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->objectActedOn_;
  else
    return Object();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

// XmlRpc::XmlRpcValue ActionOnObject::toXmlRpcValue() const {
//   XmlRpc::XmlRpcValue value = Action::toXmlRpcValue();
//   
//   if (impl_.get()) {
//     try {
//       value["object_acted_on"] = getObjectActedOn().getIdentifier();
//     }
//     catch (const XmlRpc::XmlRpcException& exception) {
//       throw XmlRpcConversionFailed(exception.getMessage());
//     }
//   }
//   
//   return value;
// }

}
