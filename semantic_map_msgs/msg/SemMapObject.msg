# Semantic map object message type, Moritz Tenorth, tenorth@cs.tum.edu
# Modifications by Ralf Kaestner, ralf.kaestner@gmail.com

Header header               # object timestamp and frame id

string id                   # unique object identifier
string type                 # object class

SemMapObjectSize size       # object size
geometry_msgs/Pose pose     # object pose

string part_of              # identifier of the parent object
