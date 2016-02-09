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

/** \file ActionOnObject.h
  * \brief Header file providing the ActionOnObject class interface
  */

#ifndef ROS_SEMANTIC_MAP_ACTION_ON_OBJECT_H
#define ROS_SEMANTIC_MAP_ACTION_ON_OBJECT_H

#include <semantic_map_common/Action.h>
#include <semantic_map_common/Object.h>

namespace semantic_map {
  class Object;
  
  /** \brief Semantic map action on object
    */    
  class ActionOnObject :
    public Action {
  public:
    /** \brief Default constructor
      */
    ActionOnObject();
    
    /** \brief Copy constructor
      */
    ActionOnObject(const ActionOnObject& src);
    
    /** \brief Copy constructor (overloaded version taking an entity)
      */
    ActionOnObject(const Entity& src);
    
    /** \brief Destructor
      */
    virtual ~ActionOnObject();
    
    /** \brief Retrieve the object acted on by this semantic map action
      *   on object
      */
    Object getObjectActedOn() const;
    
  protected:
    friend class Action;
    friend class Object;
    
    /** \brief Semantic map action (implementation)
      */
    class Impl :
      public Action::Impl {
    public:
      Impl(const std::string& identifier, const std::string& type, bool
        asserted, const Object& objectActedOn);
      virtual ~Impl();
      
      const Object objectActedOn_;
    };
  };
};

#endif
