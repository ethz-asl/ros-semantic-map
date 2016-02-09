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

#include "semantic_map_common/Mission.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Mission::Mission() {
}

Mission::Mission(const std::string& identifier, const std::string& type,
    const std::string& map, const std::string& ns) {
  impl_.reset(new Impl(identifier, type, map, ns));
}

Mission::Mission(const Mission& src) :
  Entity(src) {
}

Mission::Mission(const Entity& src) :
  Entity(src) {
  if (impl_.get())
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl_).get());
}

Mission::~Mission() {
}

Mission::Impl::Impl(const std::string& identifier, const std::string& type,
    const std::string& map, const std::string& ns) :
  Entity::Impl(identifier, type),
  ontology_(ns),
  map_(map) {
}

Mission::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Mission::setOntology(const Ontology& ontology) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->ontology_ = ontology;
}

Ontology Mission::getOntology() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->ontology_;
  else
    return Ontology();
}

void Mission::setMap(const std::string& map) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->map_ = map;
}

std::string Mission::getMap() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->map_;
  else
    return std::string();
}

size_t Mission::getNumTasks() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->tasks_.size();
  else
    return 0;
}

std::list<Task> Mission::getTasks() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->tasks_;
  else
    return std::list<Task>();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

std::list<Task>::iterator Mission::begin() {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->tasks_.begin();
  else
    return std::list<Task>::iterator();
}

std::list<Task>::const_iterator Mission::begin() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->tasks_.begin();
  else
    return std::list<Task>::const_iterator();
}

std::list<Task>::iterator Mission::end() {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->tasks_.end();
  else
    return std::list<Task>::iterator();
}

std::list<Task>::const_iterator Mission::end() const {
  if (impl_.get())
    return boost::static_pointer_cast<Impl>(impl_)->tasks_.end();
  else
    return std::list<Task>::const_iterator();
}

Task Mission::addTask(const std::string& identifier, const std::string&
    type, bool asserted, Task::Quantification quantification, bool
    unordered) {
  if (impl_.get()) {
    Task task;
    
    task.impl_.reset(new Task::Impl(identifier, type, asserted,
      quantification, unordered));
    boost::static_pointer_cast<Impl>(impl_)->tasks_.push_back(task);
    
    return task;
  }
  else
    return Task();
}

void Mission::clearTasks() {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->tasks_.clear();
}

}
