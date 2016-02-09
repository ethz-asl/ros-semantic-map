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

#include "semantic_map_common/Ontology.h"

namespace semantic_map {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Ontology::Ontology() {
}

Ontology::Ontology(const std::string& ns) {
  impl_.reset(new Impl(ns));
}

Ontology::Ontology(const Ontology& src) :
  impl_(src.impl_) {
}

Ontology::~Ontology() {
}

Ontology::Impl::Impl(const std::string& ns) :
  ns_(ns) {
}

Ontology::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Ontology::setNamespace(const std::string& ns) {
  if (impl_.get())
    impl_->ns_ = ns;
}

std::string Ontology::getNamespace() const {
  if (impl_.get())
    return impl_->ns_;
  else
    return std::string();
}

size_t Ontology::getNumImports() const {
  if (impl_.get())
    return impl_->imports_.size();
  else
    return 0;
}

std::list<std::string> Ontology::getImports() const {
  if (impl_.get())
    return impl_->imports_;
  else
    return std::list<std::string>();
}

size_t Ontology::getNumPrefixes() const {
  if (impl_.get())
    return impl_->prefixes_.size();
  else
    return 0;
}

boost::unordered_map<std::string, NamespacePrefix> Ontology::getPrefixes()
    const {
  if (impl_.get())
    return impl_->prefixes_;
  else
    return boost::unordered_map<std::string, NamespacePrefix>();
}

NamespacePrefix Ontology::getPrefix(const std::string& prefix) const {
  if (impl_.get()) {
    boost::unordered_map<std::string, NamespacePrefix>::const_iterator it =
      impl_->prefixes_.find(prefix);
      
    if (it != impl_->prefixes_.end())
      return it->second;
  }
  
  return NamespacePrefix();
}

bool Ontology::isValid() const {
  return impl_.get();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Ontology::addImport(const std::string& import) {
  BOOST_ASSERT(!import.empty());
  
  if (impl_.get())
    impl_->imports_.push_back(import);
}

void Ontology::clearImports() {
  if (impl_.get())
    impl_->imports_.clear();
}

void Ontology::addPrefix(const NamespacePrefix& prefix) {
  BOOST_ASSERT(prefix.isValid());
  
  if (impl_.get())
    impl_->prefixes_.insert(std::make_pair(prefix.getPrefix(), prefix));
}

NamespacePrefix Ontology::addPrefix(const std::string& prefix, const
    std::string& ns) {
  NamespacePrefix namespacePrefix(prefix, ns);
  
  addPrefix(namespacePrefix);
  
  return namespacePrefix;
}

void Ontology::clearPrefixes() {
  if (impl_.get())
    impl_->prefixes_.clear();
}

}
