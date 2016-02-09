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

/** \file Object.h
  * \brief Header file providing the Object class interface
  */

#ifndef ROS_SEMANTIC_MAP_OBJECT_H
#define ROS_SEMANTIC_MAP_OBJECT_H

#include <boost/unordered_map.hpp>

#include <XmlRpcValue.h>

#include <ros/exception.h>
#include <ros/time.h>

#include <semantic_map_msgs/Object.h>

#include <semantic_map_common/Entity.h>
#include <semantic_map_common/Pose.h>
#include <semantic_map_common/Size.h>

namespace semantic_map {
  class ActionOnObject;
  
  /** \brief Semantic map object
    */    
  class Object :
    public Entity {
  public:
    /** \brief Exception thrown in case of a failure to convert a semantic
      *   map object from/to an XML-RPC value
      */    
    class XmlRpcConversionFailed :
      public ros::Exception {
    public:
      XmlRpcConversionFailed(const std::string& description);
    };
    
    /** \brief Default constructor
      */
    Object();
    
    /** \brief Copy constructor
      */
    Object(const Object& src);
    
    /** \brief Copy constructor (overloaded version taking an entity)
      */
    Object(const Entity& src);
    
    /** \brief Destructor
      */
    virtual ~Object();
    
    /** \brief Retrieve the parent object of this semantic map object
      */
    Object getParentObject() const;
    
    /** \brief Set the frame identifier of this semantic map object
      */
    void setFrame(const std::string& frame);
    
    /** \brief Retrieve the frame identifier of this semantic map object
      */
    std::string getFrame() const;
    
    /** \brief Set the time stamp of this semantic map object
      */
    void setStamp(const ros::Time& stamp);
    
    /** \brief Retrieve the time stamp of this semantic map object
      */
    ros::Time getStamp() const;
    
    /** \brief Set the pose of this semantic map object
      */
    void setPose(const Pose& pose);
    
    /** \brief Retrieve the pose of this semantic map object
      */
    Pose getPose() const;
    
    /** \brief Set the size of this semantic map object
      */
    void setSize(const Size& size);
    
    /** \brief Retrieve the size of this semantic map object
      */
    Size getSize() const;
    
    /** \brief Retrieve the number of all parts of this semantic map object
      */
    size_t getNumParts() const;
    
    /** \brief Retrieve all parts of this semantic map object
      */
    boost::unordered_map<std::string, Object> getParts() const;
    
    /** \brief Retrieve a part of this semantic map object
      */
    Object getPart(const std::string& identifier) const;    
    
    /** \brief Retrieve the number of actions of this semantic map object
      */
    size_t getNumActions() const;
    
    /** \brief Retrieve the actions of this semantic map object
      */
    boost::unordered_map<std::string, ActionOnObject> getActions() const;
    
    /** \brief Retrieve an action of this semantic map object
      */
    ActionOnObject getAction(const std::string& identifier) const;    
    
    /** \brief Retrieve the begin iterator of this semantic map object
      */ 
    boost::unordered_map<std::string, Object>::iterator begin();
    
    /** \brief Retrieve the begin const-iterator of this semantic map
      *   object
      */ 
    boost::unordered_map<std::string, Object>::const_iterator begin() const;

    /** \brief Retrieve the end iterator of this semantic map object
      */ 
    boost::unordered_map<std::string, Object>::iterator end();
    
    /** \brief Retrieve the end const-iterator of this semantic map object
      */ 
    boost::unordered_map<std::string, Object>::const_iterator end() const;
    
    /** \brief Add a part to this semantic map object
      */ 
    Object addPart(const std::string& identifier, const std::string& type);
    
    /** \brief Clear the parts of this semantic map object
      */ 
    void clearParts();
    
    /** \brief Add an action to this semantic map object
      */ 
    ActionOnObject addAction(const std::string& identifier, const
      std::string& type, bool asserted = false);
    
    /** \brief Clear the actions of this semantic map object
      */ 
    void clearActions();
    
    /** \brief Convert this semantic map object to an XML-RPC value
      */
    XmlRpc::XmlRpcValue toXmlRpcValue() const;
    
    /** \brief Convert this semantic map object to a message
      */
    semantic_map_msgs::Object toMessage() const;
    
    /** \brief Operator for retrieving a part of this semantic map object
      */
    Object operator[](const std::string& identifier) const;
    
  protected:
    friend class Entity;
    friend class Map;
    
    /** \brief Semantic map action (implementation)
      */
    class Impl :
      public Entity::Impl {
    public:
      Impl(const std::string& identifier, const std::string& type, const
        Entity& parent);
      virtual ~Impl();
      
      std::string frame_;
      ros::Time stamp_;
      
      Pose pose_;
      Size size_;
      
      boost::unordered_map<std::string, Object> parts_;
      boost::unordered_map<std::string, ActionOnObject> actions_;
    };    
    
    /** \brief Constructor (overloaded version taking an identifier,
      *   a type, and a parent entity)
      */
    Object(const std::string& identifier, const std::string& type,
      const Entity& parent);
  };
};

#endif
