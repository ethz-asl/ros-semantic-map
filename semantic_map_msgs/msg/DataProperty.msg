uint8 VALUE_TYPE_INVALID = 0      # value is invalid
uint8 VALUE_TYPE_STRING  = 1      # value is a string
uint8 VALUE_TYPE_BOOL    = 2      # value is a Boolean
uint8 VALUE_TYPE_FLOAT   = 3      # value is a float
uint8 VALUE_TYPE_INT     = 4      # value is an integer

string id                         # data property identifier
string subject                    # data property subject
uint8 value_type                  # data property value type (as defined above)
string value                      # data property value
