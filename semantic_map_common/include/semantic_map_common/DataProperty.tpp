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

#include <semantic_map_common/DataPropertyTraits.h>
#include <semantic_map_common/Entity.h>

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T> DataProperty::DataProperty(const std::string&
    identifier, const Entity& subject, const T& value) {
  this->impl_.reset(new Impl(identifier, subject, DataPropertyTraits<T>::
    ValueType, boost::lexical_cast<std::string>(value)));
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> void DataProperty::setValue(const T& value) {
  if (this->impl_.get())
    boost::static_pointer_cast<Impl>(this->impl_)->template
      setValue<T>(value);
}

template <typename T> T DataProperty::getValue() const {
  if (this->impl_.get())
    return boost::static_pointer_cast<Impl>(this->impl_)->template
      getValue<T>();
  else
    return T();
}

template <typename T> void DataProperty::Impl::setValue(const T& value) {
  this->valueType_ = DataPropertyTraits<T>::ValueType;
  this->value_ = boost::lexical_cast<std::string>(value);
}

template <typename T> T DataProperty::Impl::getValue() const {
  return boost::lexical_cast<T>(this->value_);
}

}
