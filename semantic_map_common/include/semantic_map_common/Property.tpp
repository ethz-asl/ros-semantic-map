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

#include <XmlRpcException.h>

#include <semantic_map_common/Entity.h>

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class M> Property::Impl::Impl(const M& message, const boost::
    unordered_map<std::string, Entity>& entities) :
  identifier_(message.id) {
  BOOST_ASSERT(!message.id.empty());
  
  boost::unordered_map<std::string, Entity>::const_iterator it = entities.
    find(message.subject);

  BOOST_ASSERT(it != entities.end());
  BOOST_ASSERT(it->second.isValid());
  
  const_cast<boost::shared_ptr<Entity>&>(this->subject_).reset(
    new Entity(it->second));
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class M> M Property::toMessage() const {
  M message;
  
  message.id = this->getIdentifier();
  message.subject = this->getSubject().getIdentifier();
  
  return message;
}
   
}
