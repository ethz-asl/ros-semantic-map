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

/** \file Task.h
  * \brief Header file providing the Task class interface
  */

#ifndef ROS_SEMANTIC_MAP_TASK_H
#define ROS_SEMANTIC_MAP_TASK_H

#include <list>
#include <string>

#include <semantic_map_common/Action.h>

namespace semantic_map {
  /** \brief Semantic map task
    */    
  class Task :
    public Action {
  public:
    /** \brief Definition of the semantic map task quantification
      *   enumerable
      */
    enum Quantification {
      Intersection,
      Union
    };
    
    /** \brief Default constructor
      */
    Task();
    
    /** \brief Copy constructor
      */
    Task(const Task& src);
    
    /** \brief Copy constructor (overloaded version taking an entity)
      */
    Task(const Entity& src);
    
    /** \brief Destructor
      */
    virtual ~Task();
    
    /** \brief Set the quantification of this semantic map task
      */
    void setQuantification(Quantification quantification);
    
    /** \brief Retrieve the quantification of this semantic map task
      */
    Quantification getQuantification() const;
    
    /** \brief Retrieve the number of actions of this semantic map task 
      */
    size_t getNumActions() const;
    
    /** \brief Retrieve the actions of this semantic map task
      */
    std::list<Action> getActions() const;
    
    /** \brief Set to true to flag the actions of this semantic map
      *   task as unordered
      */
    void setUnordered(bool unordered);
    
    /** \brief True, if the actions of this semantic map task are
      *   unordered
      */
    bool isUnordered() const;
    
    /** \brief Retrieve the begin iterator of this semantic map task
      */ 
    std::list<Action>::iterator begin();
    
    /** \brief Retrieve the begin const-iterator of this semantic map task
      */ 
    std::list<Action>::const_iterator begin() const;

    /** \brief Retrieve the end iterator of this semantic map task
      */ 
    std::list<Action>::iterator end();
    
    /** \brief Retrieve the end const-iterator of this semantic map task
      */ 
    std::list<Action>::const_iterator end() const;
    
    /** \brief Add an action to this semantic map task
      */
    void addAction(const Action& action);
    
    /** \brief Add a subtask to this semantic map task
      */
    Task addTask(const std::string& identifier, const std::string& type,
      bool asserted = false, Quantification quantification = Intersection,
      bool unordered = false);
    
    /** \brief Clear the actions of this semantic map task
      */ 
    void clearActions();
    
  protected:
    friend class Action;
    friend class Mission;
    
    /** \brief Semantic map task (implementation)
      */
    class Impl :
      public Action::Impl {
    public:
      Impl(const std::string& identifier, const std::string& type,
        bool asserted, Quantification quantification, bool unordered);
      virtual ~Impl();
      
      std::list<Action> actions_;
      Quantification quantification_;
      bool unordered_;
    };    
  };
};

#endif
