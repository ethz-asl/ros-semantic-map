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

/** \file Address.h
  * \brief Header file providing the Address class interface
  */

#ifndef ROS_SEMANTIC_MAP_ADDRESS_H
#define ROS_SEMANTIC_MAP_ADDRESS_H

#include <string>

#include <XmlRpcValue.h>

#include <ros/exception.h>

#include <semantic_map_msgs/Address.h>

namespace semantic_map {
  /** \brief Semantic map address
    */    
  class Address {
  public:
    /** \brief Exception thrown in case of a failure to convert a semantic
      *   map address from/to an XML-RPC value
      */    
    class XmlRpcConversionFailed :
      public ros::Exception {
    public:
      XmlRpcConversionFailed(const std::string& description);
    };
    
    /** \brief Default constructor
      */
    Address(const std::string& roomNumber = std::string(), const
      std::string& floorNumber = std::string(), const std::string&
      streetNumber = std::string(), const std::string& streetName =
      std::string(), const std::string& cityName = std::string());
    
    /** \brief Copy constructor
      */
    Address(const Address& src);
    
    /** \brief Destructor
      */
    virtual ~Address();
    
    /** \brief Set the room number of this semantic map address
      */
    template <typename T> void setRoomNumber(const T& number);
    
    /** \brief Retrieve the room number of this semantic map address
      */
    template <typename T> T getRoomNumber() const;
    
    /** \brief Set the floor number of this semantic map address
      */
    template <typename T> void setFloorNumber(const T& number);
    
    /** \brief Retrieve the floor number of this semantic map address
      */
    template <typename T> T getFloorNumber() const;
    
    /** \brief Set the street number of this semantic map address
      */
    template <typename T> void setStreetNumber(const T& number);
    
    /** \brief Retrieve the street number of this semantic map address
      */
    template <typename T> T getStreetNumber() const;
    
    /** \brief Set the street name of this semantic map address
      */
    void setStreetName(const std::string& name);
    
    /** \brief Retrieve the street name of this semantic map address
      */
    const std::string& getStreetName() const;
    
    /** \brief Set the city name of this semantic map address
      */
    void setCityName(const std::string& name);
    
    /** \brief Retrieve the city name of this semantic map address
      */
    const std::string& getCityName() const;
    
    /** \brief Convert an XML-RPC value to this semantic map address
      */
    void fromXmlRpcValue(const XmlRpc::XmlRpcValue& value);
    
    /** \brief Convert this semantic map address to an XML-RPC value
      */
    void toXmlRpcValue(XmlRpc::XmlRpcValue& value) const;
    
    /** \brief Convert a message to this semantic map address
      */
    void fromMessage(const semantic_map_msgs::Address& message);
    
    /** \brief Convert this semantic map address to a message
      */
    semantic_map_msgs::Address toMessage() const;
    
  protected:
    /** \brief The room number of this semantic map address
      */
    std::string roomNumber_;
    
    /** \brief The floor number of this semantic map address
      */
    std::string floorNumber_;
    
    /** \brief The street number of this semantic map address
      */
    std::string streetNumber_;
    
    /** \brief The street name of this semantic map address
      */
    std::string streetName_;
    
    /** \brief The city name of this semantic map address
      */
    std::string cityName_;
  };
};

#include <semantic_map_common/Address.tpp>

#endif
