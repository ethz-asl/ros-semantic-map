Header header                       # mission timestamp and frame id

string ns                           # OWL namespace of this mission
string id                           # unique mission identifier

NamespacePrefix[] prefixes          # OWL namespace prefixes of this mission
string[] imports                    # OWL imports of this mission

string map                          # map identifier of this mission

Task[] tasks                        # list of tasks of this mission

ObjectProperty[] object_properties  # list of object properties of this mission
DataProperty[] data_properties      # list of data properties of this mission
