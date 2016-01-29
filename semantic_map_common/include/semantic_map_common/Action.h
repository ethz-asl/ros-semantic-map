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

/** \file Action.h
  * \brief Header file providing the Action class interface
  */

#ifndef ROS_SEMANTIC_MAP_ACTION_H
#define ROS_SEMANTIC_MAP_ACTION_H

#include <semantic_map_msgs/Action.h>

#include <semantic_map_common/Entity.h>

namespace semantic_map {
  class Object;
  
  /** \brief Semantic map action
    */    
  class Action :
    public Entity {
  public:
    /** \brief Default constructor
      */
    Action();
    
    /** \brief Copy constructor
      */
    Action(const Action& src);
    
    /** \brief Copy constructor (overloaded version taking an entity)
      */
    Action(const Entity& src);
    
    /** \brief Destructor
      */
    virtual ~Action();
    
    /** \brief Set to true to make this semantic map action an
      *   asserted action
      */
    void setAsserted(bool asserted);
    
    /** \brief True, if this semantic map action is an asserted action
      */
    bool isAsserted() const;
    
    /** \brief True, if this semantic map action is an action on a
      *   semantic map object
      */
    bool isActionOnObject() const;
    
    /** \brief True, if this semantic map action is a task
      */
    bool isTask() const;
    
  protected:
    friend class Entity;
    friend class Task;
    
    /** \brief Semantic map action (implementation)
      */
    class Impl :
      public Entity::Impl {
    public:
      Impl(const std::string& identifier, const std::string& type, const
        Entity& parent, bool asserted);
      virtual ~Impl();
      
      bool asserted_;
    };
    
    /** \brief Constructor (overloaded version taking an identifier, a type,
      *   a parent entity, and an asserted flag)
      */
    Action(const std::string& identifier, const std::string& type, const
      Entity& parent, bool asserted = false);
  };
};

#endif
