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

/** \file Ontology.h
  * \brief Header file providing the Ontology class interface
  */

#ifndef ROS_SEMANTIC_MAP_ONTOLOGY_H
#define ROS_SEMANTIC_MAP_ONTOLOGY_H

#include <list>
#include <string>

#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>

#include <semantic_map_common/NamespacePrefix.h>

namespace semantic_map {
  /** \brief Semantic map ontology
    */    
  class Ontology {
  public:
    /** \brief Default constructor
      */
    Ontology();
    
    /** \brief Constructor (overloaded version taking a namespace)
      */
    Ontology(const std::string& ns);
    
    /** \brief Copy constructor
      */
    Ontology(const Ontology& src);
    
    /** \brief Destructor
      */
    virtual ~Ontology();
    
    /** \brief Set the namespace of this semantic map ontology
      */
    void setNamespace(const std::string& ns);
    
    /** \brief Retrieve the namespace of this semantic map ontology
      */
    std::string getNamespace() const;
    
    /** \brief Retrieve the number of imports of this semantic map ontology
      */
    size_t getNumImports() const;
    
    /** \brief Retrieve the imports of this semantic map ontology
      */
    std::list<std::string> getImports() const;
    
    /** \brief Retrieve the number of namespace prefixes of this semantic
      *   map ontology
      */
    size_t getNumPrefixes() const;
    
    /** \brief Retrieve the namespace prefixes of this semantic map ontology
      */
    boost::unordered_map<std::string, NamespacePrefix> getPrefixes() const;
    
    /** \brief Retrieve a namespace prefix of this semantic map ontology
      */
    NamespacePrefix getPrefix(const std::string& prefix) const;
    
    /** \brief True, if this semantic map ontology is valid
      */
    bool isValid() const;
    
    /** \brief Add an import to this semantic map ontology
      */
    void addImport(const std::string& import);
    
    /** \brief Clear the imports of this semantic map ontology
      */
    void clearImports();
    
    /** \brief Add a namespace prefix to this semantic map ontology
      *   (overloaded version taking a namespace prefix)
      */
    void addPrefix(const NamespacePrefix& prefix);
    
    /** \brief Add a namespace prefix to this semantic map ontology
      *   (overloaded version taking a prefix and a namespace)
      */
    NamespacePrefix addPrefix(const std::string& prefix, const
      std::string& ns);
    
    /** \brief Clear the namespace prefixes of this semantic map ontology
      */
    void clearPrefixes();
    
  protected:
    /** \brief Semantic map ontology (implementation)
      */
    class Impl {
    public:
      Impl(const std::string& ns);
      virtual ~Impl();
      
      std::string ns_;
      
      std::list<std::string> imports_;
      boost::unordered_map<std::string, NamespacePrefix> prefixes_;
    };
    
    /** \brief The semantic map ontology's implementation
      */
    boost::shared_ptr<Impl> impl_;
  };
};

#endif
