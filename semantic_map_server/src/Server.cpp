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

#include "semantic_map_server/Server.h"

NODEWRAP_EXPORT_CLASS(semantic_map_server, semantic_map::server::Server)

namespace semantic_map { namespace server {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Server::Server() {
}

Server::~Server() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Server::init() {
  getSemanticMapServer_ = advertiseService("get_semantic_map",
    "get_semantic_map", &Server::getSemanticMapCallback);
}

void Server::cleanup() {
  getSemanticMapServer_.shutdown();
}

bool Server::getSemanticMapCallback(semantic_map_msgs::GetSemanticMap::
    Request& request, semantic_map_msgs::GetSemanticMap::Response&
    response) {
}

}}
