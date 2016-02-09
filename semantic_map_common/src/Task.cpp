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
    bool asserted, Quantification quantification, bool unordered) :
  Action::Impl(identifier, type, asserted),
  quantification_(quantification),
  unordered_(unordered) {
}

// Task::Impl::Impl(const XmlRpc::XmlRpcValue& value, const std::list<Action>&
//     actions, const Task& parentTask) :
//   Action::Impl(value, parentTask) {
//   std::vector<std::string> actionIdentifiers;
//   std::string quantification;
//   bool unordered;
//     
//   try {
//     if (value.hasMember("actions")) {
//       XmlRpc::XmlRpcValue& actionsValue = const_cast<XmlRpc::XmlRpcValue&>(
//         value)["actions"];
//       actionIdentifiers.reserve(actionsValue.size());
//       
//       for (size_t index = 0; index < actionsValue.size(); ++index)
//         actionIdentifiers.push_back(actionsValue[index]);
//     }
//     
//     quantification = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
//       value)["quantification"];
//     unordered = (bool)const_cast<XmlRpc::XmlRpcValue&>(value)["unordered"];
//   }
//   catch (const XmlRpc::XmlRpcException& exception) {
//     throw XmlRpcConversionFailed(exception.getMessage());
//   }
//   
//   for (size_t index = 0; index < actionIdentifiers.size(); ++index) {
//     boost::unordered_map<std::string, Action>::const_iterator it = 
//       actions.find(actionIdentifiers[index]);
//       
//     BOOST_ASSERT(it != actions.end());
//     BOOST_ASSERT(it->second.isValid());
//     
//     actions_.push_back(it->second);
//   }
//   
//   if (quantification == "union")
//     quantification_ = Union;
//   else
//     quantification_ = Intersection;
//   
//   unordered_ = unordered;
// }

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

void Task::addAction(const Action& action) {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->actions_.push_back(action);
}

Task Task::addTask(const std::string& identifier, const std::string& type,
    bool asserted, Quantification quantification, bool unordered) {
  if (impl_.get()) {
    Task task;
    
    task.impl_.reset(new Impl(identifier, type, asserted, quantification,
      unordered));
    boost::static_pointer_cast<Impl>(impl_)->actions_.push_back(task);
    
    return task;
  }
  else
    return Task();
}

void Task::clearActions() {
  if (impl_.get())
    boost::static_pointer_cast<Impl>(impl_)->actions_.clear();
}

// XmlRpc::XmlRpcValue Task::toXmlRpcValue() const {
//   XmlRpc::XmlRpcValue value = Action::toXmlRpcValue();
//   
//   if (impl_.get()) {
//     try {
//       size_t actionIndex = 0;
//       for (std::list<Action>::const_iterator it = begin(); it != end();
//           ++it, ++actionIndex)
//         value["actions"][actionIndex] = it->getIdentifier();
//       
//       if (getQuantification() == Union)
//         value["quantification"] = "union";
//       else
//         value["quantification"] = "intersection";
//       
//       value["unordered"] = isUnordered();
//     }
//     catch (const XmlRpc::XmlRpcException& exception) {
//       throw XmlRpcConversionFailed(exception.getMessage());
//     }
//   }
//   
//   return value;
// }

}
