# Semantic map action message type, Ralf Kaestner, ralf.kaestner@gmail.com

int8 INTERSECTION_OF = 0    # logic conjunction (and)
int8 UNION_OF = 1           # logic disjunction (or)

string id                   # unique action class identifier
string type                 # action superclass
bool asserted               # if true, this action is asserted

string object_acted_on      # identifier of the object acted on

string[] subactions         # identifiers of the subactions of this action
int8 quantification         # logic quantification of subactions as given above
bool unordered              # if true, subactions of this actions are unordered
