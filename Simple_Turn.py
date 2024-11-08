import odrive
from odrive.enums import *
import time

# Find an ODrive (this assumes you only have one ODrive connected)
print("Finding an ODrive...")
odrv0 = odrive.find_any()

# Set the desired position to 0.5 turns
desired_position = 2  # in turns

# Assuming you're using axis0 for control
axis = odrv0.axis0

# Set the control mode to position control
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


# Set the input mode to passthrough
axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

# Send the position command (0.5 turns)
axis.controller.input_pos = desired_position

position_inputs = [1,2,1,2,1,2]

print("Initiating Movement:")
for position in position_inputs:
    axis.controller.input_pos = position 
    time.sleep(2)






print("Movement complete.")