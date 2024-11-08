import odrive
import time
import numpy as np
from odrive.enums import *
import can
import struct
import time
import json 
class PDController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.previous_error = 0

    def update(self, target_val, measured_val, dt):
        Proportional_Error = target_val - measured_val
        Derivative_Error = (Proportional_Error - self.previous_error)/dt
        self.previous_error = Proportional_Error
        Corrected_Signal = self.Kp * Proportional_Error + self.Kd * Derivative_Error
        return Corrected_Signal
    

def get_pos_estimate(node_id):
    #Define pos path 
    pos_path = 'axis0.pos_estimate'

    # Convert path to endpoint ID
    pos_endpoint_id = endpoints[pos_path]['id']
    pos_endpoint_type = endpoints[pos_path]['type']
    
    # Send read command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, pos_endpoint_id, 0),
        is_extended_id=False
    ))
    
    # Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
            break
        
    # Unpack and print reply
    _, _, _, pos_return_value = struct.unpack_from('<BHB' + format_lookup[pos_endpoint_type], msg.data)
    
    return pos_return_value

def set_position(node_id, position, ff_vel, ff_torque):
    
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c), # 0x0c: Set_Input_Pos
        data=struct.pack('<fhh', position, int(ff_vel), int(ff_torque)), # Position: 0.5, feed forward velocity: 0, feed forward torque:)
        is_extended_id=False
    ))

def set_velocity(node_id, velocity, ff_torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
        data=struct.pack('<ff', velocity, ff_torque), # 1.0: velocity, 0.0: torque feedforward
        is_extended_id=False
    ))

def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e), # 0x0e: Set_Input_Tor
        data=struct.pack('<f', torque), # torque: Desired torque value
        is_extended_id=False
    ))

        
def set_closed_loop_control(node_id):
    # Put axis into closed loop control state
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    # Wait for axis to enter closed loop control by scanning heartbeat messages
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                break
            
def set_idle(node_id):
    # Put axis into idle
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 1), #1:idle
        is_extended_id=False
    ))

def get_ver(node_id):
    # Send read command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x00), # 0x00: Get_Version
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x00): # 0x00: Get_Version
            break
    return msg
    
if __name__ == "__main__":

    node_id_0 = 0 # must match `<odrv>.axis0.config.can.node_id`. The default is 0.
    node_id_1 = 1

    bus = can.interface.Bus("can0", interface="socketcan")

    #initialization function for animation

    with open('flat_endpoints.json', 'r') as f:
        endpoint_data = json.load(f)
        endpoints = endpoint_data['endpoints']
    # -- end load

    # -- start definitions
    OPCODE_READ = 0x00
    OPCODE_WRITE = 0x01

    # See https://docs.python.org/3/library/struct.html#format-characters
    format_lookup = {
        'bool': '?',
        'uint8': 'B', 'int8': 'b',
        'uint16': 'H', 'int16': 'h',
        'uint32': 'I', 'int32': 'i',
        'uint64': 'Q', 'int64': 'q',
        'float': 'f'
    }

    while not (bus.recv(timeout=0) is None): pass

    msg = get_ver(node_id_0)
    msg = get_ver(node_id_1)

    _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

    # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
    assert endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
    assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"


    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): pass

    #set closed loop control
    set_closed_loop_control(node_id_0)
    set_closed_loop_control(node_id_1)
    
    #Set Start position
    #offset = -0.0125
    #position = 0.15
    position2 = 0.1
    position1 = 0.165
    ff_vel = 0.1
    ff_torque = 4000
    
    set_position(node_id_0, position1, ff_vel, ff_torque)
    set_position(node_id_1, position2, ff_vel, ff_torque)
    
    time.sleep(10)
    
    
    #put leg in idle again
    set_idle(node_id_0)
    set_idle(node_id_1)


