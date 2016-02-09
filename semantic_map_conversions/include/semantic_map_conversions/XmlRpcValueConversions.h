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

/** \file XmlRpcValueConversions.h
  * \brief Header file providing the XmlRpcValueConversions class interface
  */

#ifndef ROS_SEMANTIC_MAP_XML_RPC_CONVERSIONS_H
#define ROS_SEMANTIC_MAP_XML_RPC_CONVERSIONS_H

#include <string>

#include <XmlRpcValue.h>

#include <ros/exception.h>

#include <semantic_map_common/Action.h>
#include <semantic_map_common/Entity.h>
#include <semantic_map_common/Property.h>

namespace semantic_map {
  namespace conversions {
    /** \brief Semantic map XML-RPC value conversions
      */
    class XmlRpcValueConversions {
    public:
      /** \brief Exception thrown in case of a failure to convert from
        *   or to an XML-RPC value
        */    
      class ConversionFailed :
        public ros::Exception {
      public:
        ConversionFailed(const std::string& description);
      };
      
      /** \brief Default constructor
        */
      XmlRpcValueConversions();
      
      /** \brief Destructor
        */
      virtual ~XmlRpcValueConversions();
    };
  };
};

#include <semantic_map_conversions/XmlRpcValueConversions.tpp>

#endif
