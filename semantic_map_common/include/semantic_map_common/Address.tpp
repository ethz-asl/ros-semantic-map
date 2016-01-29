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

#include <boost/lexical_cast.hpp>

namespace semantic_map {

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> void Address::setRoomNumber(const T& number) {
  this->roomNumber_ = boost::lexical_cast<std::string>(number);
}

template <typename T> T Address::getRoomNumber() const {
  return boost::lexical_cast<T>(this->roomNumber_);
}

template <typename T> void Address::setFloorNumber(const T& number) {
  this->floorNumber_ = boost::lexical_cast<std::string>(number);
}

template <typename T> T Address::getFloorNumber() const {
  return boost::lexical_cast<T>(this->floorNumber_);
}

template <typename T> void Address::setStreetNumber(const T& number) {
  this->streetNumber_ = boost::lexical_cast<std::string>(number);
}

template <typename T> T Address::getStreetNumber() const {
  return boost::lexical_cast<T>(this->streetNumber_);
}

}
