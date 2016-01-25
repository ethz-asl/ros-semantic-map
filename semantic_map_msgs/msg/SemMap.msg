# Semantic map message type, Moritz Tenorth, tenorth@cs.tum.edu
# Modifications by Ralf Kaestner, ralf.kaestner@gmail.com

Header header                             # map timestamp and frame id

string namespace                          # OWL namespace of the map
string id                                 # unique map identifier

SemMapPrefix[] prefixes                   # OWL prefixes of the map
string[] imports                          # OWL imports of the map

SemMapAddress address                     # address of the map

SemMapObject[] objects                    # list of objects in the map
SemMapAction[] actions                    # list of actions defined on the map

SemMapObjectProperty[] object_properties  # list of object properties
SemMapDataProperty[] data_properties      # list of data properties
