# Semantic map data property message type, Ralf Kaestner, ralf.kaestner@gmail.com

uint8 VALUE_TYPE_STRING = 0
uint8 VALUE_TYPE_BOOL   = 1
uint8 VALUE_TYPE_FLOAT  = 2
uint8 VALUE_TYPE_INT    = 3

string id           # data property identifier
string subject      # data property subject
uint8 value_type    # data property value type (as defined above)
string value        # data property value
