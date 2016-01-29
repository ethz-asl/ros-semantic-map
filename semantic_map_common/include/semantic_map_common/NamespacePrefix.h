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

/** \file NamespacePrefix.h
  * \brief Header file providing the NamespacePrefix class interface
  */

#ifndef ROS_SEMANTIC_MAP_NAMESPACE_PREFIX_H
#define ROS_SEMANTIC_MAP_NAMESPACE_PREFIX_H

#include <string>

#include <boost/shared_ptr.hpp>

#include <XmlRpcValue.h>

#include <ros/exception.h>

#include <semantic_map_msgs/NamespacePrefix.h>

namespace semantic_map {
  class Action;
  class Object;
  
  /** \brief Semantic map namespace prefix
    */    
  class NamespacePrefix {
  public:
    /** \brief Exception thrown in case of a failure to convert a semantic
      *   map namespace prefix from/to an XML-RPC value
      */    
    class XmlRpcConversionFailed :
      public ros::Exception {
    public:
      XmlRpcConversionFailed(const std::string& description);
    };
    
    /** \brief Default constructor
      */
    NamespacePrefix();
    
    /** \brief Constructor (overloaded version taking a prefix and a
      *   namespace)
      */
    NamespacePrefix(const std::string& prefix, const std::string& ns);
    
    /** \brief Copy constructor
      */
    NamespacePrefix(const NamespacePrefix& src);
    
    /** \brief Destructor
      */
    virtual ~NamespacePrefix();
    
    /** \brief Retrieve the prefix of this semantic map namespace prefix
      */
    std::string getPrefix() const;
    
    /** \brief Set the namespace of this semantic map namespace prefix
      */
    void setNamespace(const std::string& ns);
    
    /** \brief Retrieve the namespace of this semantic map namespace prefix
      */
    std::string getNamespace() const;
        
    /** \brief True, if this semantic map namespace prefix is valid
      */
    bool isValid() const;
        
    /** \brief Convert a message to this semantic map namespace prefix
      */
    void fromMessage(const semantic_map_msgs::NamespacePrefix& message);
    
    /** \brief Convert this semantic map namespace prefix to a message
      */
    semantic_map_msgs::NamespacePrefix toMessage() const;
    
    /** \brief Convert an XML-RPC value to this semantic map namespace prefix
      */
    void fromXmlRpcValue(const XmlRpc::XmlRpcValue& value);
    
    /** \brief Convert this semantic map namespace prefix to an XML-RPC value
      */
    void toXmlRpcValue(XmlRpc::XmlRpcValue& value) const;
    
  protected:
    /** \brief Semantic map namespace prefix (implementation)
      */
    class Impl {
    public:
      Impl(const std::string& prefix, const std::string& ns);
      virtual ~Impl();
      
      const std::string prefix_;
      std::string ns_;
    };
    
    /** \brief The semantic map namespace prefix's implementation
      */
    boost::shared_ptr<Impl> impl_;
  };
};

#endif
