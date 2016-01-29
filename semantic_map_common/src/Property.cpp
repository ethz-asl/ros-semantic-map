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

#include <semantic_map_common/DataProperty.h>
#include <semantic_map_common/Entity.h>
#include <semantic_map_common/ObjectProperty.h>

#include "semantic_map_common/Property.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Property::Property() {
}

Property::Property(const Property& src) :
  impl_(src.impl_) {
}

Property::~Property() {  
}

Property::Impl::Impl(const std::string& identifier, const Entity& subject) :
  identifier_(identifier),
  subject_(new Entity(subject)) {
  BOOST_ASSERT(!identifier.empty());
  BOOST_ASSERT(subject.isValid());
}

Property::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string Property::getIdentifier() const {
  if (impl_.get())
    return impl_->identifier_;
  else
    return std::string();
}

Entity Property::getSubject() const {
  if (impl_.get())
    return *impl_->subject_;
  else
    return Entity();
}

bool Property::isDataProperty() const {
  if (impl_.get())
    return boost::dynamic_pointer_cast<DataProperty::Impl>(impl_).get();
  else
    return false;
}

bool Property::isObjectProperty() const {
  if (impl_.get())
    return boost::dynamic_pointer_cast<ObjectProperty::Impl>(impl_).get();
  else
    return false;
}

bool Property::isValid() const {
  return impl_.get();
}

}
