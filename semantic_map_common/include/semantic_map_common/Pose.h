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

/** \file Pose.h
  * \brief Header file providing the Pose class interface
  */

#ifndef ROS_SEMANTIC_MAP_POSE_H
#define ROS_SEMANTIC_MAP_POSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <XmlRpcValue.h>

#include <ros/exception.h>

#include <geometry_msgs/Pose.h>

namespace semantic_map {
  /** \brief Semantic map size
    */    
  class Pose {
  public:
    /** \brief Exception thrown in case of a failure to convert a semantic
      *   map pose from/to an XML-RPC value
      */    
    class XmlRpcConversionFailed :
      public ros::Exception {
    public:
      XmlRpcConversionFailed(const std::string& description);
    };
    
    /** \brief Default constructor
      */
    Pose(double x = 0.0, double y = 0.0, double z = 0.0, double i = 0.0,
      double j = 0.0, double k = 0.0, double w = 1.0);
      
    /** \brief Copy constructor
      */
    Pose(const Pose& src);
    
    /** \brief Destructor
      */
    virtual ~Pose();
    
    /** \brief Set the position of this pose
      */
    void setPosition(const Eigen::Vector3d& position);
    
    /** \brief Retrieve the position of this pose
      */
    const Eigen::Vector3d& getPosition() const;
    
    /** \brief Set the orientation of this pose
      */
    void setOrientation(const Eigen::Quaterniond& orientation);
    
    /** \brief Retrieve the orientation of this pose
      */
    const Eigen::Quaterniond& getOrientation() const;
    
    /** \brief Convert this semantic map pose to an XML-RPC value
      */
    XmlRpc::XmlRpcValue toXmlRpcValue() const;
    
    /** \brief Convert this semantic map pose to a message
      */
    geometry_msgs::Pose toMessage() const;
    
  protected:
    friend class Object;
        
    /** \brief The position of this pose
      */
    Eigen::Vector3d position_;
    
    /** \brief The orientation of this pose
      */
    Eigen::Quaterniond orientation_;
    
    /** \brief Constructor (overloaded version taking an XML-RPC value)
      */
    Pose(const XmlRpc::XmlRpcValue& value);
    
    /** \brief Constructor (overloaded version taking a message)
      */
    Pose(const geometry_msgs::Pose& message);
  };
};

#endif
