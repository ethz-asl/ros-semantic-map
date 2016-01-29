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

#include <semantic_map_common/ActionOnObject.h>
#include <semantic_map_common/Task.h>

#include "semantic_map_common/Action.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Action::Action() {
}

Action::Action(const std::string& identifier, const std::string& type, const
    Entity& parent, bool asserted) {
  impl_.reset(new Impl(identifier, type, parent, asserted));
}

Action::Action(const Action& src) :
  Entity(src) {
}

Action::Action(const Entity& src) :
  Entity(src) {
  if (impl_.get())
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl_).get());
}

Action::~Action() {  
}

Action::Impl::Impl(const std::string& identifier, const std::string& type,
    const Entity& parent, bool asserted) :
  Entity::Impl(identifier, type, parent),
  asserted_(asserted) {
}

Action::Impl::~Impl() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Action::setAsserted(bool asserted) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->asserted_ = asserted;
}

bool Action::isAsserted() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->asserted_;
  else
    return false;
}

bool Action::isActionOnObject() const {
  return boost::dynamic_pointer_cast<ActionOnObject::Impl>(impl_).get();
}

bool Action::isTask() const {
  return boost::dynamic_pointer_cast<Task::Impl>(impl_).get();
}

}
