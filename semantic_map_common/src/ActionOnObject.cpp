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

ActionOnObject::ActionOnObject(const std::string& identifier, const
    std::string& type, const Object& objectActedOn, bool asserted) {
  impl_.reset(new Impl(identifier, type, objectActedOn, asserted));
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
    type, const Object& objectActedOn, bool asserted) :
  Action::Impl(identifier, type, objectActedOn, asserted) {
  BOOST_ASSERT(objectActedOn.isValid());
}

ActionOnObject::Impl::~Impl() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

Object ActionOnObject::getObjectActedOn() const {
  if (impl_.get())
    return getParent();
  else
    return Object();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

semantic_map_msgs::ActionOnObject ActionOnObject::toMessage() const {
  semantic_map_msgs::ActionOnObject message;
  
  message.id = getIdentifier();
  message.type = getType();

  message.asserted = isAsserted();
  
  message.object_acted_on = getObjectActedOn().getIdentifier();
  
  return message;
}

}
