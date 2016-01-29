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

/** \file DataProperty.h
  * \brief Header file providing the DataProperty class interface
  */

#ifndef ROS_SEMANTIC_MAP_DATA_PROPERTY_H
#define ROS_SEMANTIC_MAP_DATA_PROPERTY_H

#include <string>

#include <XmlRpcValue.h>

#include <ros/exception.h>

#include <semantic_map_msgs/DataProperty.h>

#include <semantic_map_common/Property.h>

namespace semantic_map {
  class Map;
  
  /** \brief Semantic map data property
    */    
  class DataProperty :
    public Property {
  public:
    /** \brief Definition of the semantic map data property value type
      *   enumerable
      */
    enum ValueType {
      Invalid,
      String,
      Boolean,
      Float,
      Integer
    };
    
    /** \brief Exception thrown in case of a failure to convert a semantic
      *   map data property from/to an XML-RPC value
      */    
    class XmlRpcConversionFailed :
      public ros::Exception {
    public:
      XmlRpcConversionFailed(const std::string& description);
    };
    
    /** \brief Default constructor
      */
    DataProperty();
    
    /** \brief Copy constructor
      */
    DataProperty(const DataProperty& src);
    
    /** \brief Copy constructor (overloaded version taking a property)
      */
    DataProperty(const Property& src);
    
    /** \brief Destructor
      */
    virtual ~DataProperty();
    
    /** \brief Retrieve the value type of this semantic map data property
      */
    ValueType getValueType() const;
    
    /** \brief Set the value of this semantic map data property
      */
    template <typename T> void setValue(const T& value);
    
    /** \brief Retrieve the value of this semantic map data property
      */
    template <typename T> T getValue() const;
        
    /** \brief Clear the value of this semantic map data property
      */
    void clearValue();
    
    /** \brief Convert a message to this semantic map data property
      */
    void fromMessage(const semantic_map_msgs::DataProperty& message,
      const Map& map);
    
    /** \brief Convert this semantic map data property to a message
      */
    semantic_map_msgs::DataProperty toMessage() const;
    
    /** \brief Convert an XML-RPC value to this semantic map data property
      */
    void fromXmlRpcValue(const XmlRpc::XmlRpcValue& value, const Map& map);
    
    /** \brief Convert this semantic map data property to an XML-RPC value
      */
    void toXmlRpcValue(XmlRpc::XmlRpcValue& value) const;
    
  protected:
    friend class Entity;
    friend class Property;
    
    /** \brief Semantic map data property (implementation)
      */
    class Impl :
      public Property::Impl {
    public:
      Impl(const std::string& identifier, const Entity& subject, ValueType
        valueType = Invalid, const std::string& value = std::string());
      virtual ~Impl();
      
      ValueType valueType_;
      std::string value_;
    };
    
    /** \brief Constructor (overloaded version taking an identifier, a
      *   subject entity, an a value)
      */
    template <typename T> DataProperty(const std::string& identifier,
      const Entity& subject, const T& value);
  };
};

#include <semantic_map_common/DataProperty.tpp>

#endif
