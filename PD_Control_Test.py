import can
import struct
import time

# Define gains for position and velocity controllers
pos_gain = 3
vel_gain = 0.08
vel_integrator_gain = 0.1

nodes = [0, 1]  # Node IDs for node 0 and node 1
desired_position = 0.33  # Desired position for hopping motion
target_position = 0.0  # Target position to hold
bus = can.interface.Bus("can0", interface="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None):
    pass

def set_closed_loop_control(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 8),
        is_extended_id=False
    ))
    
def set_torque_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),
        data=struct.pack('<II', 1, 1),
        is_extended_id=False
    ))

def set_position_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),
        data=struct.pack('<II', 3, 1),
        is_extended_id=False
    ))

def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),
        data=struct.pack('<f', torque),
        is_extended_id=False
    ))

def set_position(node_id, position, ff_vel, ff_tor):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<fhh',position,int(ff_vel),(ff_tor)),
        is_extended_id=False
    ))

def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))
    
# Function to get position and velocity feedback from the encoder
def encoder_estimates(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09),  # Request for encoder data
        data=b'',
        is_extended_id=False
    ))
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09):  # Response with encoder data
            pos_return_value, vel_return_value = struct.unpack_from('<ff', msg.data)
            return pos_return_value, vel_return_value
        
# Cascade control structure with position and velocity loops
def cascade_control(current_position, current_velocity, pos_setpoint, vel_feedforward=0.0, current_feedforward=0.0):
    # Position Control Loop (P controller)
    pos_error = pos_setpoint - current_position
    vel_cmd = pos_error * pos_gain + vel_feedforward
    
    # Velocity Control Loop (PI controller)
    vel_error = vel_cmd - current_velocity
    cascade_control.current_integral += vel_error * vel_integrator_gain
    current_cmd = vel_error * vel_gain + cascade_control.current_integral + current_feedforward
    
    return current_cmd

# Initialize integrator for the velocity controller
cascade_control.current_integral = 0.0


    

# Put each node into closed loop control mode
for node_id in nodes:
    set_closed_loop_control(node_id)
    set_position_control_mode(node_id)

# Initial position setup
for node_id in nodes:
    set_position(node_id, desired_position,100,100)
time.sleep(2)

# Switch each node to torque control mode
for node_id in nodes:
    set_torque_control_mode(node_id)
    
# Run control loop to reach target position
reached_target = False

while True:
    
    for node_id in nodes:
        # Get current position and velocity feedback from the encoder
        current_position, current_velocity = encoder_estimates(node_id)
        
        # Determine target position based on whether we've reached the target
        if reached_target:
            pos_setpoint = target_position  # Hold position at target
        else:
            pos_setpoint = desired_position  # Move towards desired position
            
        # Calculate PD torque command
        current_cmd = cascade_control(current_position, current_velocity, pos_setpoint)
        # Set calculated torque command for each node
        set_torque(node_id, current_cmd)
    
    # Break the while loop if any node has reached the target position
    if current_position <= target_position:
        reached_target = True
    
    time.sleep(0.05)  # Control loop delay
    
    # Break if both nodes have reached the target and are in hold mode
    if reached_target:
        break


# Enter holding phase
while True:
    for node_id in nodes:
        # Get current position and velocity feedback from the encoder
        current_position, current_velocity = encoder_estimates(node_id)
        
        # Calculate torque to hold position at target
        current_cmd = cascade_control(current_position, current_velocity, target_position)
        
        # Set calculated torque command for the node
        set_torque(node_id, current_cmd)

    time.sleep(0.05)  # Control loop delay for holding position
