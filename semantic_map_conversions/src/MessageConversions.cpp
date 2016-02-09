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

#include "semantic_map_conversions/MessageConversions.h"

namespace semantic_map { namespace conversions {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageConversions::ConversionFailed::ConversionFailed(const std::string&
    description) :
  ros::Exception("Message conversion failed: "+description) {
}

MessageConversions::MessageConversions() {
}

MessageConversions::~MessageConversions() {  
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

ActionOnObject MessageConversions::actionOnObjectFromMessage(const
    semantic_map_msgs::ActionOnObject& message, Map& map) const {
  Object object = map.getObject(message.object_acted_on);
  
  if (object.isValid())
    return object.addAction(message.id, message.type, message.asserted);
  else
    throw ConversionFailed("Object with identifier ["+
      message.object_acted_on+"] is undefined.");
}

semantic_map_msgs::ActionOnObject MessageConversions::actionOnObjectToMessage(
    const ActionOnObject& action) const {
  semantic_map_msgs::ActionOnObject message = actionToMessage<
    semantic_map_msgs::ActionOnObject>(action);
  
  message.object_acted_on = action.getObjectActedOn().getIdentifier();
  
  return message;
}

Address MessageConversions::addressFromMessage(const semantic_map_msgs::
    Address& message) const {
  return Address(message.room_nr, message.floor_nr, message.street_nr,
    message.street_name, message.city_name);
}

semantic_map_msgs::Address MessageConversions::addressToMessage(const Address&
    address) const {
  semantic_map_msgs::Address message;
  
  message.room_nr = address.getRoomNumber<std::string>();
  message.floor_nr = address.getFloorNumber<std::string>();
  message.street_nr = address.getStreetNumber<std::string>();
  message.street_name = address.getStreetName();
  message.city_name = address.getCityName();
  
  return message;
}

DataProperty MessageConversions::dataPropertyFromMessage(const
    semantic_map_msgs::DataProperty& message, boost::unordered_map<
    std::string, Entity>& entities) const {
  boost::unordered_map<std::string, Entity>::iterator it = entities.
    find(message.subject);
  
  if ((it == entities.end()) || !it->second.isValid())
    throw ConversionFailed("Entity with identifier ["+
      message.subject+"] is undefined.");
    
  if (message.value_type == semantic_map_msgs::DataProperty::
      VALUE_TYPE_STRING)
    return it->second.addProperty(message.id, message.value);
  else if (message.value_type == semantic_map_msgs::DataProperty::
      VALUE_TYPE_BOOL)
    return it->second.addProperty(message.id, boost::lexical_cast<bool>(
      message.value));
  else if (message.value_type == semantic_map_msgs::DataProperty::
      VALUE_TYPE_FLOAT)
    return it->second.addProperty(message.id, boost::lexical_cast<double>(
      message.value));
  else if (message.value_type == semantic_map_msgs::DataProperty::
      VALUE_TYPE_INT)
    return it->second.addProperty(message.id, boost::lexical_cast<int>(
      message.value));
  else
    return DataProperty();
}

semantic_map_msgs::DataProperty MessageConversions::dataPropertyToMessage(
    const DataProperty& property) const {
  semantic_map_msgs::DataProperty message = propertyToMessage<
    semantic_map_msgs::DataProperty>(property);
  
  DataProperty::ValueType valueType = property.getValueType();
  
  if (valueType == DataProperty::String)
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_STRING;
  else if (valueType == DataProperty::Boolean)
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_BOOL;
  else if (valueType == DataProperty::Float)
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_FLOAT;
  else if (valueType == DataProperty::Integer)
    message.value_type = semantic_map_msgs::DataProperty::VALUE_TYPE_INT;
  
  message.value = property.getValue<std::string>();
  
  return message;
}

Map MessageConversions::mapFromMessage(const semantic_map_msgs::Map& message)
    const {
  Map map(message.id, message.type);
  
  map.setFrame(message.header.frame_id);
  map.setStamp(message.header.stamp);
  
  map.setOntology(ontologyFromMessage(message.ontology));
  
  map.setAddress(addressFromMessage(message.address));
  
  boost::unordered_map<std::string, Entity> entities;
  entities.insert(std::make_pair(map.getIdentifier(), map));
  
  for (size_t index = 0; index < message.objects.size(); ++index) {
    Object object = objectFromMessage(message.objects[index], map);
    entities.insert(std::make_pair(object.getIdentifier(), object));
  }
  
  for (size_t index = 0; index < message.actions.size(); ++index) {
    ActionOnObject action = actionOnObjectFromMessage(message.actions[
      index], map);
    entities.insert(std::make_pair(action.getIdentifier(), action));
  }
  
  for (size_t index = 0; index < message.object_properties.size(); ++index)
    objectPropertyFromMessage(message.object_properties[index], entities);
  for (size_t index = 0; index < message.data_properties.size(); ++index)
    dataPropertyFromMessage(message.data_properties[index], entities);
  
  return map;
}

semantic_map_msgs::Map MessageConversions::mapToMessage(const Map& map)
    const {
  semantic_map_msgs::Map message = entityToMessage<
    semantic_map_msgs::Map>(map);
  
  message.header.frame_id = map.getFrame();
  message.header.stamp = map.getStamp();
  
  message.ontology = ontologyToMessage(map.getOntology());
  
  message.address = addressToMessage(map.getAddress());
  
  boost::unordered_map<std::string, Object> objects = map.getObjects();
  boost::unordered_map<std::string, ActionOnObject> actions;
  boost::unordered_multimap<std::string, Property> properties = map.
    getProperties();
  
  message.objects.reserve(objects.size());  
  for (boost::unordered_map<std::string, Object>::const_iterator it =
      objects.begin(); it != objects.end(); ++it) {
    message.objects.push_back(objectToMessage(it->second));
  
    boost::unordered_map<std::string, ActionOnObject> actionsOnObject =
      it->second.getActions();
    for (boost::unordered_map<std::string, ActionOnObject>::const_iterator
        jt = actionsOnObject.begin(); jt != actionsOnObject.end(); ++jt)
      actions.insert(*jt);
    
    boost::unordered_multimap<std::string, Property> objectProperties =
      it->second.getProperties();
    for (boost::unordered_multimap<std::string, Property>::const_iterator
        jt = objectProperties.begin(); jt != objectProperties.end(); ++jt)
      properties.insert(*jt);
  }
  
  message.actions.reserve(actions.size());
  for (boost::unordered_map<std::string, ActionOnObject>::const_iterator
      it = actions.begin(); it != actions.end(); ++it) {
    message.actions.push_back(actionOnObjectToMessage(it->second));  
  
    boost::unordered_multimap<std::string, Property> actionProperties =
      it->second.getProperties();
    for (boost::unordered_multimap<std::string, Property>::const_iterator
        jt = actionProperties.begin(); jt != actionProperties.end(); ++jt)
      properties.insert(*jt);
  }
  
  message.object_properties.reserve(properties.size());
  message.data_properties.reserve(properties.size());
  for (boost::unordered_multimap<std::string, Property>::const_iterator
      it = properties.begin(); it != properties.end(); ++it) {
    if (it->second.isObjectProperty())
      message.object_properties.push_back(objectPropertyToMessage(
        it->second));
    else
      message.data_properties.push_back(dataPropertyToMessage(it->second));
  }
  
  return message;
}

Mission MessageConversions::missionFromMessage(const semantic_map_msgs::
    Mission& message) const {
  Mission mission(message.id, message.type, message.map);
  
  mission.setOntology(ontologyFromMessage(message.ontology));
  
  boost::unordered_map<std::string, Entity> entities;
  entities.insert(std::make_pair(mission.getIdentifier(), mission));
  
  for (size_t index = 0; index < message.tasks.size(); ++index) {
    Task task = taskFromMessage(message.tasks[index], mission);
    entities.insert(std::make_pair(task.getIdentifier(), task));
  }
  
  for (size_t index = 0; index < message.object_properties.size(); ++index)
    objectPropertyFromMessage(message.object_properties[index], entities);
  for (size_t index = 0; index < message.data_properties.size(); ++index)
    dataPropertyFromMessage(message.data_properties[index], entities);
  
  return mission;
}

semantic_map_msgs::Mission MessageConversions::missionToMessage(const
    Mission& mission) const {
  semantic_map_msgs::Mission message = entityToMessage<
    semantic_map_msgs::Mission>(mission);
  
  message.ontology = ontologyToMessage(mission.getOntology());
  
  message.map = mission.getMap();
  
  boost::unordered_multimap<std::string, Property> properties = mission.
    getProperties();
  
  message.tasks.reserve(mission.getNumTasks());
  for (std::list<Task>::const_iterator it = mission.begin();
      it != mission.end(); ++it) {
    message.tasks.push_back(taskToMessage(*it));
  
    boost::unordered_multimap<std::string, Property> taskProperties =
      it->getProperties();
    for (boost::unordered_multimap<std::string, Property>::const_iterator
        jt = taskProperties.begin(); jt != taskProperties.end(); ++jt)
      properties.insert(*jt);
  }
  
  message.object_properties.reserve(properties.size());
  message.data_properties.reserve(properties.size());
  for (boost::unordered_multimap<std::string, Property>::const_iterator
      it = properties.begin(); it != properties.end(); ++it) {
    if (it->second.isObjectProperty())
      message.object_properties.push_back(objectPropertyToMessage(
        it->second));
    else
      message.data_properties.push_back(dataPropertyToMessage(it->second));
  }
  
  return message;
}

Object MessageConversions::objectFromMessage(const semantic_map_msgs::Object&
    message, Map& map) const {
  Object object;
  
  if (!message.parent.empty()) {
    Object parent = map.getObject(message.parent);
    
    if (parent.isValid())
      object = parent.addPart(message.id, message.type);
    else
      throw ConversionFailed("Object with identifier ["+
        message.parent+"] is undefined.");
  }
  else
    object = map.addObject(message.id, message.type);
  
  object.setFrame(message.header.frame_id);
  object.setStamp(message.header.stamp);
  
  object.setPose(poseFromMessage(message.pose));
  object.setSize(sizeFromMessage(message.size));
  
  return object;
}

semantic_map_msgs::Object MessageConversions::objectToMessage(const Object&
    object) const {
  semantic_map_msgs::Object message = entityToMessage<
    semantic_map_msgs::Object>(object);
  
  message.header.frame_id = object.getFrame();
  message.header.stamp = object.getStamp();
  
  message.parent = object.getParentObject().getIdentifier();
  
  message.pose = object.getPose().toMessage();
  message.size = object.getSize().toMessage();

  return message;
}

ObjectProperty MessageConversions::objectPropertyFromMessage(const
    semantic_map_msgs::ObjectProperty& message, boost::unordered_map<
    std::string, Entity>& entities) const {
  boost::unordered_map<std::string, Entity>::iterator it = entities.
    find(message.subject);
  
  if ((it == entities.end()) || !it->second.isValid())
    throw ConversionFailed("Entity with identifier ["+
      message.subject+"] is undefined.");
    
  boost::unordered_map<std::string, Entity>::iterator jt = entities.
    find(message.object);
  
  if ((jt == entities.end()) || !jt->second.isValid())
    throw ConversionFailed("Entity with identifier ["+
      message.object+"] is undefined.");

  return it->second.addProperty(message.id, jt->second);
}

semantic_map_msgs::ObjectProperty MessageConversions::objectPropertyToMessage(
    const ObjectProperty& property) const {
  semantic_map_msgs::ObjectProperty message = propertyToMessage<
    semantic_map_msgs::ObjectProperty>(property);
  
  message.object = property.getObject().getIdentifier();
  
  return message;
}

Ontology MessageConversions::ontologyFromMessage(const semantic_map_msgs::
    Ontology& message) const {
  Ontology ontology(message.ns);
  
  for (size_t index = 0; index < message.imports.size(); ++index)
    ontology.addImport(message.imports[index]);
  
  for (size_t index = 0; index < message.ns_prefixes.size(); ++index)
    ontology.addPrefix(prefixFromMessage(message.ns_prefixes[index]));
  
  return ontology;
}

semantic_map_msgs::Ontology MessageConversions::ontologyToMessage(const
    Ontology& ontology) const {
  semantic_map_msgs::Ontology message;
  
  message.ns = ontology.getNamespace();

  std::list<std::string> imports = ontology.getImports();
  message.imports.reserve(imports.size());
  for (std::list<std::string>::const_iterator it = imports.begin();
      it != imports.end(); ++it)
    message.imports.push_back(*it);
  
  boost::unordered_map<std::string, NamespacePrefix> prefixes =
    ontology.getPrefixes();
  message.ns_prefixes.reserve(prefixes.size());
  for (boost::unordered_map<std::string, NamespacePrefix>::const_iterator
      it = prefixes.begin(); it != prefixes.end(); ++it)
    message.ns_prefixes.push_back(prefixToMessage(it->second));
  
  return message;
}

Pose MessageConversions::poseFromMessage(const geometry_msgs::Pose& message)
    const {
  return Pose(message.position.x, message.position.y, message.position.z,
    message.orientation.x, message.orientation.y, message.orientation.z,
    message.orientation.w);
}

geometry_msgs::Pose MessageConversions::poseToMessage(const Pose& pose)
    const {
  geometry_msgs::Pose message;
  
  const Eigen::Vector3d& position = pose.getPosition();
  message.position.x = position[0];
  message.position.y = position[1];
  message.position.z = position[2];
  
  const Eigen::Quaterniond& orientation = pose.getOrientation();
  message.orientation.x = orientation.x();
  message.orientation.y = orientation.y();
  message.orientation.z = orientation.z();
  message.orientation.w = orientation.w();
  
  return message;
}

NamespacePrefix MessageConversions::prefixFromMessage(const semantic_map_msgs::
    NamespacePrefix& message) const {
  return NamespacePrefix(message.prefix, message.ns);
}

semantic_map_msgs::NamespacePrefix MessageConversions::prefixToMessage(const
    NamespacePrefix& prefix) const {
  semantic_map_msgs::NamespacePrefix message;
  
  message.prefix = prefix.getPrefix();
  message.ns = prefix.getNamespace();
  
  return message;
}

Size MessageConversions::sizeFromMessage(const semantic_map_msgs::Size&
    message) const {
  return Size(message.y, message.z, message.x);
}

semantic_map_msgs::Size MessageConversions::sizeToMessage(const Size& size)
    const {
  semantic_map_msgs::Size message;
  
  const Eigen::Vector3d& dimensions = size.getDimensions();
  message.x = dimensions[0];
  message.y = dimensions[1];
  message.z = dimensions[2];
  
  return message;
}

semantic_map_msgs::Task MessageConversions::taskToMessage(const Task& task)
    const {
  semantic_map_msgs::Task message = actionToMessage<semantic_map_msgs::Task>(
    task);
  
  message.actions.reserve(task.getNumActions());
  for (std::list<Action>::const_iterator it = task.begin();
      it != task.end(); ++it)
    message.actions.push_back(it->getIdentifier());
    
  if (task.getQuantification() == Task::Union)
    message.quantification = semantic_map_msgs::Task::QUANTIFICATION_UNION;
  else
    message.quantification = semantic_map_msgs::Task::
      QUANTIFICATION_INTERSECTION;
  
  message.unordered = task.isUnordered();
  
  return message;
}

}}
