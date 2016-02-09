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

/** \file MessageConversions.h
  * \brief Header file providing the MessageConversions class interface
  */

#ifndef ROS_SEMANTIC_MAP_MESSAGE_CONVERSIONS_H
#define ROS_SEMANTIC_MAP_MESSAGE_CONVERSIONS_H

#include <string>

#include <boost/unordered_map.hpp>

#include <ros/exception.h>

#include <geometry_msgs/Pose.h>

#include <semantic_map_msgs/ActionOnObject.h>
#include <semantic_map_msgs/Address.h>
#include <semantic_map_msgs/DataProperty.h>
#include <semantic_map_msgs/Map.h>
#include <semantic_map_msgs/Mission.h>
#include <semantic_map_msgs/NamespacePrefix.h>
#include <semantic_map_msgs/Object.h>
#include <semantic_map_msgs/ObjectProperty.h>
#include <semantic_map_msgs/Ontology.h>
#include <semantic_map_msgs/Size.h>
#include <semantic_map_msgs/Task.h>

#include <semantic_map_common/Action.h>
#include <semantic_map_common/ActionOnObject.h>
#include <semantic_map_common/Address.h>
#include <semantic_map_common/DataProperty.h>
#include <semantic_map_common/Entity.h>
#include <semantic_map_common/Map.h>
#include <semantic_map_common/Mission.h>
#include <semantic_map_common/NamespacePrefix.h>
#include <semantic_map_common/Object.h>
#include <semantic_map_common/ObjectProperty.h>
#include <semantic_map_common/Ontology.h>
#include <semantic_map_common/Pose.h>
#include <semantic_map_common/Property.h>
#include <semantic_map_common/Size.h>
#include <semantic_map_common/Task.h>

namespace semantic_map {
  namespace conversions {
    /** \brief Semantic map message conversions
      */
    class MessageConversions {
    public:
      /** \brief Exception thrown in case of a failure to convert from
        *   or to a message
        */    
      class ConversionFailed :
        public ros::Exception {
      public:
        ConversionFailed(const std::string& description);
      };
      
      /** \brief Default constructor
        */
      MessageConversions();
      
      /** \brief Destructor
        */
      virtual ~MessageConversions();
      
      /** \brief Convert a semantic map action to a message
        */
      template <class M> M actionToMessage(const Action& action) const;
    
      /** \brief Convert a message to a semantic map action on object
        */
      ActionOnObject actionOnObjectFromMessage(const semantic_map_msgs::
        ActionOnObject& message, Map& map) const;
    
      /** \brief Convert a semantic map action on object to a message
        */
      semantic_map_msgs::ActionOnObject actionOnObjectToMessage(const
        ActionOnObject& action) const;
    
      /** \brief Convert a message to a semantic map address
        */
      Address addressFromMessage(const semantic_map_msgs::Address& message)
        const;
        
      /** \brief Convert a semantic map address to a message
        */
      semantic_map_msgs::Address addressToMessage(const Address& address)
        const;
      
      /** \brief Convert a message to a semantic map data property
        */
      DataProperty dataPropertyFromMessage(const semantic_map_msgs::
        DataProperty& message, boost::unordered_map<std::string, Entity>&
        entities) const;
    
      /** \brief Convert a semantic map data property to a message
        */
      semantic_map_msgs::DataProperty dataPropertyToMessage(const
        DataProperty& property) const;
    
      /** \brief Convert a semantic map entity to a message
        */
      template <class M> M entityToMessage(const Entity& entity) const;
      
      /** \brief Convert a message to a semantic map
        */
      Map mapFromMessage(const semantic_map_msgs::Map& message) const;
    
      /** \brief Convert a semantic map to a message
        */
      semantic_map_msgs::Map mapToMessage(const Map& map) const;
    
      /** \brief Convert a message to a semantic map mission
        */
      Mission missionFromMessage(const semantic_map_msgs::Mission& message)
        const;
    
      /** \brief Convert a semantic map mission to a message
        */
      semantic_map_msgs::Mission missionToMessage(const Mission& mission)
        const;
    
      /** \brief Convert a message to a semantic map object
        */
      Object objectFromMessage(const semantic_map_msgs::Object& message,
        Map& map) const;
    
      /** \brief Convert a semantic map object to a message
        */
      semantic_map_msgs::Object objectToMessage(const Object& object) const;
    
      /** \brief Convert a message to a semantic map object property
        */
      ObjectProperty objectPropertyFromMessage(const semantic_map_msgs::
        ObjectProperty& message, boost::unordered_map<std::string, Entity>&
        entities) const;
    
      /** \brief Convert a semantic map object property to a message
        */
      semantic_map_msgs::ObjectProperty objectPropertyToMessage(const
        ObjectProperty& property) const;
    
      /** \brief Convert a message to a semantic map ontology
        */
      Ontology ontologyFromMessage(const semantic_map_msgs::Ontology&
        message) const;
      
      /** \brief Convert a semantic map ontology to a message
        */
      semantic_map_msgs::Ontology ontologyToMessage(const Ontology&
        ontology) const;
      
      /** \brief Convert a message to a semantic map pose
        */
      Pose poseFromMessage(const geometry_msgs::Pose& message) const;
        
      /** \brief Convert a semantic map pose to a message
        */
      geometry_msgs::Pose poseToMessage(const Pose& pose) const;
        
      /** \brief Convert a semantic map property to a message
        */
      template <class M> M propertyToMessage(const Property& property) const;
      
      /** \brief Convert a message to a semantic map namespace prefix
        */
      NamespacePrefix prefixFromMessage(const semantic_map_msgs::
        NamespacePrefix& message) const;
        
      /** \brief Convert a semantic map namespace prefix to a message
        */
      semantic_map_msgs::NamespacePrefix prefixToMessage(const
        NamespacePrefix& prefix) const;
    
      /** \brief Convert a message to a semantic map size
        */
      Size sizeFromMessage(const semantic_map_msgs::Size& message) const;
      
      /** \brief Convert a semantic map size to a message
        */
      semantic_map_msgs::Size sizeToMessage(const Size& size) const;
      
      /** \brief Convert a message to a semantic map task
        */
      template <class P> Task taskFromMessage(const semantic_map_msgs::Task&
        message, P& parent) const;
      
      /** \brief Convert a semantic map task to a message
        */
      semantic_map_msgs::Task taskToMessage(const Task& task) const;
    };
  };
};

#include <semantic_map_conversions/MessageConversions.tpp>

#endif
