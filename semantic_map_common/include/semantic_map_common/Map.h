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

/** \file Map.h
  * \brief Header file providing the Map class interface
  */

#ifndef ROS_SEMANTIC_MAP_H
#define ROS_SEMANTIC_MAP_H

#include <string>

#include <boost/unordered_map.hpp>

#include <ros/time.h>

#include <semantic_map_common/Address.h>
#include <semantic_map_common/Entity.h>
#include <semantic_map_common/Object.h>
#include <semantic_map_common/Ontology.h>

namespace semantic_map {
  /** \brief Semantic map
    */    
  class Map :
    public Entity {
  public:
    /** \brief Default constructor
      */
    Map();
    
    /** \brief Constructor (overloaded version taking an identifier, a type,
      *   an ontology, a frame identifier, and a timestamp)
      */
    Map(const std::string& identifier, const std::string& type, const
      std::string& ns = "http://ros.org/semantic_map/map.owl", const
      std::string& frame = "/map", const ros::Time& stamp = ros::Time::now());
    
    /** \brief Copy constructor
      */
    Map(const Map& src);
    
    /** \brief Copy constructor (overloaded version taking an entity)
      */
    Map(const Entity& src);
    
    /** \brief Destructor
      */
    virtual ~Map();
    
    /** \brief Set the ontology of this semantic map
      */
    void setOntology(const Ontology& ontology);
    
    /** \brief Retrieve the ontology of this semantic map
      */
    Ontology getOntology() const;
    
    /** \brief Set the frame identifier of this semantic map
      */
    void setFrame(const std::string& frame);
    
    /** \brief Retrieve the frame identifier of this semantic map
      */
    std::string getFrame() const;
    
    /** \brief Set the time stamp of this semantic map
      */
    void setStamp(const ros::Time& stamp);
    
    /** \brief Retrieve the time stamp of this semantic map
      */
    ros::Time getStamp() const;
    
    /** \brief Set the address of this semantic map
      */
    void setAddress(const Address& address);
    
    /** \brief Retrieve the address of this semantic map
      */
    Address getAddress() const;

    /** \brief Retrieve the number of all objects of this semantic map
      */
    size_t getNumObjects() const;
    
    /** \brief Retrieve all objects of this semantic map
      */
    boost::unordered_map<std::string, Object> getObjects() const;
    
    /** \brief Retrieve an object of this semantic map
      */
    Object getObject(const std::string& identifier) const;    
    
    /** \brief Retrieve the begin iterator of this semantic map
      */ 
    boost::unordered_map<std::string, Object>::iterator begin();
    
    /** \brief Retrieve the begin const-iterator of this semantic map
      */ 
    boost::unordered_map<std::string, Object>::const_iterator begin() const;

    /** \brief Retrieve the end iterator of this semantic map
      */ 
    boost::unordered_map<std::string, Object>::iterator end();
    
    /** \brief Retrieve the end const-iterator of this semantic map
      */ 
    boost::unordered_map<std::string, Object>::const_iterator end() const;
    
    /** \brief Add an object to this semantic map
      */
    Object addObject(const std::string& identifier, const std::string& type);
    
    /** \brief Clear the objects of this semantic map
      */
    void clearObjects();

    /** \brief Operator for retrieving an object of this semantic map
      */
    Object operator[](const std::string& identifier) const;
    
    /** \brief Operator for comparing this semantic map to another map
      */
    bool operator==(const Map& map) const;
    
  protected:
    friend class Entity;
    
    /** \brief Semantic map (implementation)
      */
    class Impl :
      public Entity::Impl {
    public:
      Impl(const std::string& identifier, const std::string& type, const
        std::string& ns, const std::string& frame, const ros::Time& stamp);
      virtual ~Impl();
      
      Ontology ontology_;
      
      std::string frame_;
      ros::Time stamp_;

      Address address_;
      
      boost::unordered_map<std::string, Object> objects_;
    };
  };
};

#endif
