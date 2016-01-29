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

/** \file Size.h
  * \brief Header file providing the Size class interface
  */

#ifndef ROS_SEMANTIC_MAP_SIZE_H
#define ROS_SEMANTIC_MAP_SIZE_H

#include <Eigen/Core>

#include <XmlRpcValue.h>

#include <ros/exception.h>

#include <semantic_map_msgs/Size.h>

namespace semantic_map {
  /** \brief Semantic map size
    */    
  class Size {
  public:
    /** \brief Exception thrown in case of a failure to convert a semantic
      *   map size from/to an XML-RPC value
      */    
    class XmlRpcConversionFailed :
      public ros::Exception {
    public:
      XmlRpcConversionFailed(const std::string& description);
    };
    
    /** \brief Default constructor
      */
    Size(double width = 0.0, double height = 0.0, double depth = 0.0);
      
    /** \brief Copy constructor
      */
    Size(const Size& src);
    
    /** \brief Destructor
      */
    virtual ~Size();
    
    /** \brief Set the width dimension of this size
      */
    void setWidth(double width);
    
    /** \brief Retrieve the width dimension of this size
      */
    double getWidth() const;
    
    /** \brief Set the height dimension of this size
      */
    void setHeight(double height);
    
    /** \brief Retrieve the height dimension of this size
      */
    double getHeight() const;
    
    /** \brief Set the depth dimension of this size
      */
    void setDepth(double depth);
    
    /** \brief Retrieve the depth dimension of this size
      */
    double getDepth() const;
    
    /** \brief Set the dimensions of this size
      */
    void setDimensions(const Eigen::Vector3d& dimensions);
    
    /** \brief Retrieve the dimensions of this size
      */
    const Eigen::Vector3d& getDimensions() const;
    
    /** \brief Convert an XML-RPC value to this semantic map size
      */
    void fromXmlRpcValue(const XmlRpc::XmlRpcValue& value);
    
    /** \brief Convert this semantic map size to an XML-RPC value
      */
    void toXmlRpcValue(XmlRpc::XmlRpcValue& value) const;
    
    /** \brief Convert a message to this semantic map size
      */
    void fromMessage(const semantic_map_msgs::Size& message);
    
    /** \brief Convert this semantic map size to a message
      */
    semantic_map_msgs::Size toMessage() const;
    
  protected:
    /** \brief The dimensions of this size
      */
    Eigen::Vector3d dimensions_;
  };
};

#endif
