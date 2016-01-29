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

#include <XmlRpcException.h>

#include "semantic_map_common/NamespacePrefix.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

NamespacePrefix::XmlRpcConversionFailed::XmlRpcConversionFailed(const
    std::string& description) :
  ros::Exception("XML-RPC value conversion of semantic map namespace "
    "prefix failed: "+description) {
}

NamespacePrefix::NamespacePrefix() {
}

NamespacePrefix::NamespacePrefix(const std::string& prefix, const
    std::string& ns) :
  impl_(new Impl(prefix, ns)) {
}

NamespacePrefix::NamespacePrefix(const NamespacePrefix& src) :
  impl_(src.impl_) {
}

NamespacePrefix::~NamespacePrefix() {  
}

NamespacePrefix::Impl::Impl(const std::string& prefix, const
    std::string& ns) :
  prefix_(prefix),
  ns_(ns) {
  BOOST_ASSERT(!prefix.empty());
  BOOST_ASSERT(!ns.empty());
}

NamespacePrefix::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string NamespacePrefix::getPrefix() const {
  if (impl_.get())
    return impl_->prefix_;
  else
    return std::string();
}

void NamespacePrefix::setNamespace(const std::string& ns) {
  BOOST_ASSERT(!ns.empty());
  
  if (impl_.get())
    impl_->ns_ = ns;
}

std::string NamespacePrefix::getNamespace() const {
  if (impl_.get())
    return impl_->ns_;
  else
    return std::string();
}

bool NamespacePrefix::isValid() const {
  return impl_.get();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void NamespacePrefix::fromXmlRpcValue(const XmlRpc::XmlRpcValue& value) {
  try {
    std::string prefix = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
      value)["prefix"];
    std::string ns = (std::string)const_cast<XmlRpc::XmlRpcValue&>(
      value)["ns"];
      
    if (impl_.get()) {
      BOOST_ASSERT(prefix == getPrefix());
      setNamespace(ns);
    }
    else
      impl_.reset(new Impl(prefix, ns));
  }
  catch (const XmlRpc::XmlRpcException& exception) {
    throw XmlRpcConversionFailed(exception.getMessage());
  }
}

void NamespacePrefix::toXmlRpcValue(XmlRpc::XmlRpcValue& value) const {
  if (impl_.get()) {
    try {
      value["prefix"] = getPrefix();
      value["ns"] = getNamespace();
    }
    catch (const XmlRpc::XmlRpcException& exception) {
      throw XmlRpcConversionFailed(exception.getMessage());
    }
  }
}

void NamespacePrefix::fromMessage(const semantic_map_msgs::NamespacePrefix&
    message) {
  if (impl_.get()) {
    BOOST_ASSERT(message.prefix == getPrefix());
    setNamespace(message.ns);
  }
  else
    impl_.reset(new Impl(message.prefix, message.ns));  
}

semantic_map_msgs::NamespacePrefix NamespacePrefix::toMessage() const {
  semantic_map_msgs::NamespacePrefix message;
  
  message.prefix = getPrefix();
  message.ns = getNamespace();
  
  return message;
}

}
