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

#include "semantic_map_common/Task.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Task::Task() {
}

Task::Task(const std::string& identifier, const std::string& type, const
    Entity& parent, bool asserted) {
  impl_.reset(new Impl(identifier, type, parent, asserted));
}

Task::Task(const Task& src) :
  Action(src) {
}

Task::Task(const Entity& src) :
  Action(src) {
  if (impl_.get())
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl_).get());
}

Task::~Task() {  
}

Task::Impl::Impl(const std::string& identifier, const std::string& type,
    const Entity& parent, bool asserted) :
  Action::Impl(identifier, type, parent, asserted),
  quantification_(Intersection),
  unordered_(false) {
}

Task::Impl::~Impl() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Task::setQuantification(Quantification quantification) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->quantification_ =
      quantification;
}

Task::Quantification Task::getQuantification() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->quantification_;
  else
    return Intersection;
}

size_t Task::getNumActions() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->actions_.size();
  else
    return 0;
}

std::list<Action> Task::getActions() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->actions_;
  else
    return std::list<Action>();
}

void Task::setUnordered(bool unordered) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->unordered_ = unordered;
}

bool Task::isUnordered() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->unordered_;
  else
    return false;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

std::list<Action>::iterator Task::begin() {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->actions_.begin();
  else
    return std::list<Action>::iterator();
}

std::list<Action>::const_iterator Task::begin() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->actions_.begin();
  else
    return std::list<Action>::const_iterator();
}

std::list<Action>::iterator Task::end() {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->actions_.end();
  else
    return std::list<Action>::iterator();
}

std::list<Action>::const_iterator Task::end() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->actions_.end();
  else
    return std::list<Action>::const_iterator();
}

Action Task::addAction(const std::string& identifier, const std::string& type,
    bool asserted) {
  if (impl_.get()) {
    Action action(identifier, type, *this, asserted);
    
    boost::static_pointer_cast<Impl>(impl_)->actions_.push_back(action);
    
    return action;
  }
  else
    return Action();  
}

void Task::clearActions() {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->actions_.clear();
}

semantic_map_msgs::Task Task::toMessage() const {
  semantic_map_msgs::Task message;
  
  message.id = getIdentifier();
  message.type = getType();
  
  message.asserted = isAsserted();
  
  message.actions.reserve(getNumActions());
  for (std::list<Action>::const_iterator it = begin(); it != end(); ++it)
    message.actions.push_back(it->getIdentifier());
    
  if (getQuantification() == Union)
    message.quantification = semantic_map_msgs::Task::UNION_OF;
  else
    message.quantification = semantic_map_msgs::Task::INTERSECTION_OF;
  message.unordered = isUnordered();
  
  return message;
}

}
