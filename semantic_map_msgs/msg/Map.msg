Header header                         # map timestamp and frame id

string ns                             # OWL namespace of this map
string id                             # unique map identifier

NamespacePrefix[] prefixes            # OWL namespace prefixes of this map
string[] imports                      # OWL imports of this map

Address address                       # address of this map

Object[] objects                      # list of objects in this map

ObjectProperty[] object_properties    # list of object properties of this map
DataProperty[] data_properties        # list of data properties of this map
