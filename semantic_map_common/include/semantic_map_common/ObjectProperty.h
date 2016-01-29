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

/** \file ObjectProperty.h
  * \brief Header file providing the ObjectProperty class interface
  */

#ifndef ROS_SEMANTIC_MAP_OBJECT_PROPERTY_H
#define ROS_SEMANTIC_MAP_OBJECT_PROPERTY_H

#include <semantic_map_msgs/ObjectProperty.h>

#include <semantic_map_common/Entity.h>
#include <semantic_map_common/Property.h>

namespace semantic_map {
  /** \brief Semantic map object property
    */    
  class ObjectProperty :
    public Property {
  public:
    /** \brief Default constructor
      */
    ObjectProperty();
    
    /** \brief Copy constructor
      */
    ObjectProperty(const ObjectProperty& src);
    
    /** \brief Copy constructor (overloaded version taking a property)
      */
    ObjectProperty(const Property& src);
    
    /** \brief Destructor
      */
    virtual ~ObjectProperty();
    
    /** \brief Set the object of this semantic map object property
      */
    void setObject(const Entity& object);
    
    /** \brief Retrieve the subject of this semantic map object property
      */
    Entity getObject() const;
    
    /** \brief Convert this semantic map object property to a message
      */
    semantic_map_msgs::ObjectProperty toMessage() const;
    
  protected:
    friend class Entity;
    friend class Property;
    
    /** \brief Semantic map object property (implementation)
      */
    class Impl :
      public Property::Impl {
    public:
      Impl(const std::string& identifier, const Entity& subject, const
        Entity& object);
      virtual ~Impl();
      
      Entity object_;
    };
    
    /** \brief Constructor (overloaded version taking an identifier, a
      *   subject entity, and an object entity)
      */
    ObjectProperty(const std::string& identifier, const Entity& subject,
      const Entity& object);
  };
};

#endif
