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

/** \file Exceptions.h
  * \brief Header file providing the Exceptions class interface
  */

#ifndef ROS_SEMANTIC_MAP_EXCEPTIONS_H
#define ROS_SEMANTIC_MAP_EXCEPTIONS_H

#include <ros/exception.h>

namespace semantic_map {
  /** \brief Exception thrown in case of a failure to convert a semantic
    *   map or one of its members from or to an XML-RPC value
    */    
  class XmlRpcConversionFailed :
    public ros::Exception {
  public:
    XmlRpcConversionFailed(const std::string& description);
  };
};

#endif
