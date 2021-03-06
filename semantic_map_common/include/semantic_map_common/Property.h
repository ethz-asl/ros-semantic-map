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

/** \file Property.h
  * \brief Header file providing the Property class interface
  */

#ifndef ROS_SEMANTIC_MAP_PROPERTY_H
#define ROS_SEMANTIC_MAP_PROPERTY_H

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <XmlRpcValue.h>

#include <ros/exception.h>

namespace semantic_map {
  class DataProperty;
  class Entity;
  class ObjectProperty;
    
  /** \brief Semantic map property
    */    
  class Property {
  public:
    /** \brief Exception thrown in case of a failure to convert a semantic
      *   map property from/to an XML-RPC value
      */    
    class XmlRpcConversionFailed :
      public ros::Exception {
    public:
      XmlRpcConversionFailed(const std::string& description);
    };
    
    /** \brief Default constructor
      */
    Property();
    
    /** \brief Copy constructor
      */
    Property(const Property& src);
    
    /** \brief Destructor
      */
    virtual ~Property();
    
    /** \brief Retrieve the identifier of this semantic map property
      */
    std::string getIdentifier() const;
        
    /** \brief Retrieve the subject of this semantic map property
      */
    Entity getSubject() const;
    
    /** \brief True, if this semantic map property is a data property
      */
    bool isDataProperty() const;
        
    /** \brief True, if this semantic map property is an object property
      */
    bool isObjectProperty() const;
        
    /** \brief True, if this semantic map property is valid
      */
    bool isValid() const;
    
    /** \brief Convert this semantic map property to an XML-RPC value
      */
    virtual XmlRpc::XmlRpcValue toXmlRpcValue() const;
    
    /** \brief Convert this semantic map property to a message
      */
    template <class M> M toMessage() const;
    
  protected:
    friend class Entity;
    
    /** \brief Semantic map property (implementation)
      */
    class Impl {
    public:
      Impl(const std::string& identifier, const Entity& subject);
      Impl(const XmlRpc::XmlRpcValue& value, const boost::unordered_map<
        std::string, Entity>& entities);
      template <class M> Impl(const M& message, const boost::unordered_map<
        std::string, Entity>& entities);
      virtual ~Impl();
      
      const std::string identifier_;
      const boost::shared_ptr<Entity> subject_;
    };
    
    /** \brief The semantic map property's implementation
      */
    boost::shared_ptr<Impl> impl_;    
  };
};

#include <semantic_map_common/Property.tpp>

#endif
