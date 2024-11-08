import can
import struct
import time

nodes = [0, 1]  # Node IDs for node 0 and node 1

bus = can.interface.Bus("can0", interface="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): 
    pass

# Function to put a node into closed loop control mode
def set_closed_loop_control(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
        data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

# Function to put a node into closed loop control mode
def set_torque_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
        data=struct.pack('<II', 1, 1),  # 6: TORQUE_RAMP
        is_extended_id=False
    ))

def set_position_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
        data=struct.pack('<II', 3, 1),  # 6: Position
        is_extended_id=False
    ))

# Function to set a torque command for a node
def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),  # 0x0e: Set_Input_Torque
        data=struct.pack('<f', torque),  # Set desired torque
        is_extended_id=False
    ))

def set_position(node_id, position, ff_vel, ff_tor):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<fhh', position, int(ff_vel), int(ff_tor)),
        is_extended_id=False
    ))

    
def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))


            

# Put each node into closed loop control mode
for node_id in nodes:
    set_closed_loop_control(node_id)
    set_position_control_mode(node_id)


# Wait for each node to enter closed loop control by scanning heartbeat messages
for node_id in nodes:
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
                break
torque_command = 0.2  # Torque value to set for each node
           
#set the position origin
set_position(nodes[0], 0.2, 100, 100)
set_position(nodes[1], 0.2, 100, 100)
time.sleep(5)

# Put each node into closed loop control mode
for node_id in nodes:
    set_torque_control_mode(node_id)


# Set torque command for each node
set_torque(nodes[0], torque_command)
set_torque(nodes[1], torque_command)
time.sleep(0.5)

#set_torque(nodes[0], -torque_command)
#set_torque(nodes[1], -torque_command)
#time.sleep(2)

set_idle(nodes[0])
set_idle(nodes[1])

