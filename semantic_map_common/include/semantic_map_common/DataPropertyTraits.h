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

/** \file DataPropertyTraits.h
  * \brief Header file providing the DataPropertyTraits class interface
  */

#ifndef ROS_SEMANTIC_MAP_DATA_PROPERTY_TRAITS_H
#define ROS_SEMANTIC_MAP_DATA_PROPERTY_TRAITS_H

#include <string>

#include <boost/type_traits.hpp>

#include <semantic_map_common/DataProperty.h>

namespace semantic_map {
  template <typename T, typename Enable = void> class DataPropertyTraits;

  template <typename T> struct DataPropertyTraits<T, typename boost::
      enable_if<boost::is_base_of<std::string, T> >::type> {
    static const DataProperty::ValueType ValueType = DataProperty::String;
  };
  
  template <typename T> struct DataPropertyTraits<T, typename boost::
      enable_if<boost::is_same<T, bool> >::type> {
    static const DataProperty::ValueType ValueType = DataProperty::Boolean;
  };
  
  template <typename T> struct DataPropertyTraits<T, typename boost::
      enable_if<boost::type_traits::ice_and<boost::is_integral<T>::value,
      boost::type_traits::ice_not<boost::is_same<T, bool>::value>::value> >::
      type> {
    static const DataProperty::ValueType ValueType = DataProperty::Integer;
  };
  
  template <typename T> struct DataPropertyTraits<T, typename boost::
      enable_if<boost::is_float<T> >::type> {
    static const DataProperty::ValueType ValueType = DataProperty::Float;
  };
};

#endif
