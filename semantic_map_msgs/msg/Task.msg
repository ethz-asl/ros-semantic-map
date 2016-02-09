int8 QUANTIFICATION_INTERSECTION = 0 # conjunction of actions/subtasks (and)
int8 QUANTIFICATION_UNION = 1        # disjunction of actions/subtasks (or)

string id                            # unique task class identifier
string type                          # task superclass

bool asserted                        # if true, this task is asserted

string[] actions                     # identifiers of the actions/subtasks
int8 quantification                  # quantification of actions as given above
bool unordered                       # if true, actions/subtasks are unordered
