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

/** \file Entity.h
  * \brief Header file providing the Entity class interface
  */

#ifndef ROS_SEMANTIC_MAP_ENTITY_H
#define ROS_SEMANTIC_MAP_ENTITY_H

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

namespace semantic_map {
  class Action;
  class DataProperty;
  class Map;
  class Object;
  class ObjectProperty;
  class Property;
  
  /** \brief Semantic map entity
    */    
  class Entity {
  public:
    /** \brief Default constructor
      */
    Entity();
    
    /** \brief Copy constructor
      */
    Entity(const Entity& src);
    
    /** \brief Destructor
      */
    virtual ~Entity();
    
    /** \brief Retrieve the identifier of this semantic map entity
      */
    std::string getIdentifier() const;
    
    /** \brief Set the type of this semantic map entity
      */
    void setType(const std::string& type);
    
    /** \brief Retrieve the type of this semantic map entity
      */
    std::string getType() const;
        
    /** \brief Retrieve the parent entity of this semantic map entity
      */
    Entity getParent() const;
        
    /** \brief Retrieve the number of properties of this semantic map
      *   entity
      */
    size_t getNumProperties() const;
    
    /** \brief Retrieve the properties of this semantic map entity
      */
    boost::unordered_map<std::string, Property> getProperties() const;
    
    /** \brief Retrieve a property of this semantic map entity
      */
    Property getProperty(const std::string& identifier) const;
    
    /** \brief True, if this semantic map entity has a parent
      */
    bool hasParent() const;
    
    /** \brief True, if this semantic map entity is a semantic map
      */
    bool isMap() const;
    
    /** \brief True, if this semantic map entity is a semantic map object
      */
    bool isObject() const;
        
    /** \brief True, if this semantic map entity is a semantic map action
      */
    bool isAction() const;
        
    /** \brief True, if this semantic map entity is valid
      */
    bool isValid() const;
        
    /** \brief Add a property to this semantic map entity (overloaded
      *   version for adding a data property)
      */
    template <typename T> DataProperty addProperty(const std::string&
      identifier, const T& value);
    
    /** \brief Add a property to this semantic map entity (overloaded
      *   version for adding an object property)
      */
    ObjectProperty addProperty(const std::string& identifier, const
      Entity& object);
    
    /** \brief Clear the properties of this semantic map entity
      */
    void clearProperties();
    
  protected:
    friend class DataProperty;
    friend class ObjectProperty;
    
    /** \brief Semantic map entity (implementation)
      */
    class Impl {
    public:
      Impl(const std::string& identifier, const std::string& type, const
        Entity& parent = Entity());
      virtual ~Impl();
      
      const std::string identifier_;
      std::string type_;
      
      const boost::shared_ptr<Entity> parent_;
      
      boost::unordered_map<std::string, Property> properties_;
    };
    
    /** \brief The semantic map entity's implementation
      */
    boost::shared_ptr<Impl> impl_;
  };
};

#include <semantic_map_common/Entity.tpp>

#endif
