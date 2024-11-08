"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""

import can
import struct
import time

node_id_0 = 0 # must match `<odrv>.axis0.config.can.node_id`. The default is 0.

bus = can.interface.Bus("can0", interface="socketcan")


# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

# Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id_0 << 5 | 0x07), # 0x07: Set_Axis_State
    data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))


# Wait for axis to enter closed loop control by scanning heartbeat messages
for msg in bus:
    if msg.arbitration_id == (node_id_0 << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
            break
 

bus.send(can.Message(
        arbitration_id=(node_id_0 << 5 | 0x0c), # 0x0c: Set_Input_Pos
        data=struct.pack('<fhh', -0.1, int(1), int(2)), # Position: 0.5, feed forward velocity: 0, feed forward torque:)
        is_extended_id=False
))
time.sleep(1)

# Velocity_Vector = [-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1]
# 
# for Velocity in Velocity_Vector:
#     
#     # Set velocity to 1.0 turns/s
#     bus.send(can.Message(
#         arbitration_id=(node_id_0 << 5 | 0x0d), # 0x0d: Set_Input_Vel
#         data=struct.pack('<ff', Velocity, 1), # 1.0: velocity, 0.0: torque feedforward
#         is_extended_id=False
#     ))
#     
#     time.sleep(1)
# 


# 
# # Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id_0 << 5 | 0x07), # 0x07: Set_Axis_State
    data=struct.pack('<I', 1), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))



# Print encoder feedback
for msg in bus:
    if msg.arbitration_id == (node_id_0 << 5 | 0x09): # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos M1: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")

