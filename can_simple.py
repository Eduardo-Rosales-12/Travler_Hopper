import can
import struct

# Define the node IDs you want to control
node_ids = [0, 1]  # for node 0 and node 1

bus = can.interface.Bus("can0", interface ="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): 
    pass

# Put each axis into closed loop control state
for node_id in node_ids:
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
        data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

# Wait for each axis to enter closed loop control by scanning heartbeat messages
for node_id in node_ids:
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
                break
            
            
# Set velocity to 1.0 turns/s for each axis
for node_id in node_ids:
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0d),  # 0x0d: Set_Input_Vel
        data=struct.pack('<ff', 1.0, 0.0),  # 1.0: velocity, 0.0: torque feedforward
        is_extended_id=False
    ))
    
    
