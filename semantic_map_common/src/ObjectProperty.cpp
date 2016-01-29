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

#include "semantic_map_common/ObjectProperty.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ObjectProperty::ObjectProperty() {
}

ObjectProperty::ObjectProperty(const std::string& identifier, const
    Entity& subject, const Entity& object) {
  impl_.reset(new Impl(identifier, subject, object));
}

ObjectProperty::ObjectProperty(const ObjectProperty& src) :
  Property(src) {
}

ObjectProperty::ObjectProperty(const Property& src) :
  Property(src) {
  if (impl_.get())
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl_).get());
}

ObjectProperty::~ObjectProperty() {  
}

ObjectProperty::Impl::Impl(const std::string& identifier, const Entity&
    subject, const Entity& object) :
  Property::Impl(identifier, subject),
  object_(object) {
  BOOST_ASSERT(object.isValid());
}

ObjectProperty::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void ObjectProperty::setObject(const Entity& object) {
  BOOST_ASSERT(object.isValid());
  
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->object_ = object;
}

Entity ObjectProperty::getObject() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->object_;
  else
    return Entity();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

semantic_map_msgs::ObjectProperty ObjectProperty::toMessage() const {
  semantic_map_msgs::ObjectProperty message;
  
  message.id = getIdentifier();
  message.subject = getSubject().getIdentifier();
  message.object = getObject().getIdentifier();
  
  return message;
}

}
