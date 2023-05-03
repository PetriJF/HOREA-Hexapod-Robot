# A message used to describe a step. The direction is used to represent the 
# direction of the step while the angle is used to represent rotation of the 
# robot. 
# If we want a curve movement, we need to activate both flags and use both 
# components

float64 direction   # Represents the direction of the step
float64 angle       # Represents the angle of the step (e.g. turning)
float64[2] ang_rep  # Used to represent the angular turn rather than the direction 
float64 step_len    # Represents the step length
float64 gait_alt    # Represents the gait altitude
float64 gait_wid    # Represents the gait width
bool dir_component  # Flag used to represent the use of the directional component
bool ang_component  # Flag used to represent the use of the angular component