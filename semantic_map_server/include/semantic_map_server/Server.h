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

/** \file Server.h
  * \brief Header file providing the Server class interface
  */

#ifndef ROS_SEMANTIC_MAP_SERVER_H
#define ROS_SEMANTIC_MAP_SERVER_H

#include <semantic_map_msgs/GetSemanticMap.h>

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

namespace semantic_map {
  namespace server {
    /** \brief Semantic map server implementation
      */  
    class Server :
      public nodewrap::NodeImpl {
    public:
      /** \brief Default constructor
        */
      Server();
      
      /** \brief Destructor
        */
      virtual ~Server();
    
    protected:
      /** \brief Initialize the semantic map server
        */
      void init();
      
      /** \brief Cleanup the semantic map server
        */
      void cleanup();
      
    private:
      /** \brief The service server for retrieving the semantic map
        */
      nodewrap::ServiceServer getSemanticMapServer_;
      
      /** \brief The service callback for retrieving the semantic map
        */
      bool getSemanticMapCallback(semantic_map_msgs::GetSemanticMap::
        Request& request, semantic_map_msgs::GetSemanticMap::Response&
        response);
    };
  };
};

#endif
