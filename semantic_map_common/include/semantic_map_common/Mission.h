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

/** \file Mission.h
  * \brief Header file providing the Mission class interface
  */

#ifndef ROS_SEMANTIC_MAP_MISSION_H
#define ROS_SEMANTIC_MAP_MISSION_H

#include <list>
#include <string>

#include <ros/time.h>

#include <semantic_map_common/Entity.h>
#include <semantic_map_common/Ontology.h>
#include <semantic_map_common/Task.h>

namespace semantic_map {
  /** \brief Semantic map mission
    */    
  class Mission :
    public Entity {
  public:
    /** \brief Default constructor
      */
    Mission();
    
    /** \brief Constructor (overloaded version taking an identifier, a
      *   type, and an ontology)
      */
    Mission(const std::string& identifier, const std::string&
      type, const std::string& map, const std::string& ns =
      "http://ros.org/semantic_map/mission.owl");
    
    /** \brief Copy constructor
      */
    Mission(const Mission& src);
    
    /** \brief Copy constructor (overloaded version taking an entity)
      */
    Mission(const Entity& src);
    
    /** \brief Destructor
      */
    virtual ~Mission();
    
    /** \brief Set the ontology of this semantic map mission
      */
    void setOntology(const Ontology& ontology);
    
    /** \brief Retrieve the ontology of this semantic map mission
      */
    Ontology getOntology() const;
    
    /** \brief Set the map of this semantic map mission
      */
    void setMap(const std::string& map);
    
    /** \brief Retrieve the map of this semantic map mission
      */
    std::string getMap() const;
    
    /** \brief Retrieve the number of tasks of this semantic map mission
      */
    size_t getNumTasks() const;
    
    /** \brief Retrieve the tasks of this semantic map mission
      */
    std::list<Task> getTasks() const;
    
    /** \brief Retrieve the begin iterator of this semantic map mission
      */ 
    std::list<Task>::iterator begin();
    
    /** \brief Retrieve the begin const-iterator of this semantic map mission
      */ 
    std::list<Task>::const_iterator begin() const;

    /** \brief Retrieve the end iterator of this semantic map mission
      */ 
    std::list<Task>::iterator end();
    
    /** \brief Retrieve the end const-iterator of this semantic map mission
      */ 
    std::list<Task>::const_iterator end() const;
    
    /** \brief Add a task to this semantic map mission
      */
    Task addTask(const std::string& identifier, const std::string& type,
      bool asserted = false, Task::Quantification quantification = Task::
      Intersection, bool unordered = false);
    
    /** \brief Clear the tasks of this semantic map mission
      */
    void clearTasks();
    
  protected:
    friend class Entity;
    
    /** \brief Semantic map (implementation)
      */
    class Impl :
      public Entity::Impl {
    public:
      Impl(const std::string& identifier, const std::string& type, const
        std::string& map, const std::string& ns);
      virtual ~Impl();
      
      Ontology ontology_;
      
      std::string map_;
      
      std::list<Task> tasks_;
    };
  };
};

#endif
