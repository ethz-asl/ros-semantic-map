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

namespace semantic_map { namespace conversions {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class M> M MessageConversions::actionToMessage(const Action& action)
    const {
  M message = this->template entityToMessage<M>(action);
  
  message.asserted = action.isAsserted();
  
  return message;
}

template <class M> M MessageConversions::entityToMessage(const Entity& entity)
    const {
  M message;
      
  message.id = entity.getIdentifier();
  message.type = entity.getType();
  
  return message;
}

template <class M> M MessageConversions::propertyToMessage(const Property&
    property) const {
  M message;
      
  message.id = property.getIdentifier();
  message.subject = property.getSubject().getIdentifier();
  
  return message;
}

template <class P> Task MessageConversions::taskFromMessage(const
    semantic_map_msgs::Task& message, P& parent) const {
  Task task = parent.addTask(message.id, message.type, message.asserted);

  for (size_t index = 0; index < message.actions.size(); ++index)
    ;
//     task.addAction(message.actions[index]);
  
  if (message.quantification == semantic_map_msgs::Task::QUANTIFICATION_UNION)
    task.setQuantification(Task::Union);
  else
    task.setQuantification(Task::Intersection);
 
  task.setUnordered(message.unordered);
  
  return task;
}

}}
