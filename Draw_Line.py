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
        Derivative_Error = (Proportional_Error - self.previous_error) / dt
        self.previous_error = Proportional_Error
        Corrected_Signal = self.Kp * Proportional_Error + self.Kd * Derivative_Error
        return Corrected_Signal

def get_pos_estimate(node_id):
    pos_path = 'axis0.pos_estimate'
    pos_endpoint_id = endpoints[pos_path]['id']
    pos_endpoint_type = endpoints[pos_path]['type']
    
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04),
        data=struct.pack('<BHB', OPCODE_READ, pos_endpoint_id, 0),
        is_extended_id=False
    ))
    
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x05):
            break
        
    _, _, _, pos_return_value = struct.unpack_from('<BHB' + format_lookup[pos_endpoint_type], msg.data)
    return pos_return_value

def set_position(node_id, position, ff_vel, ff_torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<fhh', position, int(ff_vel), int(ff_torque)),
        is_extended_id=False
    ))

def set_velocity(node_id, velocity, ff_torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0d),
        data=struct.pack('<ff', velocity, ff_torque),
        is_extended_id=False
    ))

def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),
        data=struct.pack('<f', torque),
        is_extended_id=False
    ))

def set_closed_loop_control(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 8),
        is_extended_id=False
    ))
    
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x01):
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8:
                break
            
def send_message(node_id, path, value):
    endpoint_id = endpoints[path]['id']
    endpoint_type = endpoints[path]['type']
    
    # Send write command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04),
        data=struct.pack('<BHB' + format_lookup[endpoint_type], OPCODE_WRITE, endpoint_id, 0, value),
        is_extended_id=False
    ))
    
    # Flush CAN RX buffer to clear old messages
    while not (bus.recv(timeout=0) is None): pass

    # Send read command to confirm write
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04),
        data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
        is_extended_id=False
    ))
    
    # Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x05):
            break

    # Print confirmation
    _, _, _, return_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)
    print(f"Received: {return_value}")
    
def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))
    
def get_ver(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x00),
        data=b'',
        is_extended_id=False
    ))

    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x00):
            break
    return msg

def save_config(node_id, path):
    endpoint_id = endpoints[path]['id']
    
    while not (bus.recv(timeout=0) is None): pass
    
    # Send save command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x16),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))
    
    print("Configuration saved")

if __name__ == "__main__":
    node_id_0 = 0
    node_id_1 = 1

    bus = can.interface.Bus("can0", interface="socketcan")

    with open('flat_endpoints.json', 'r') as f:
        endpoint_data = json.load(f)
        endpoints = endpoint_data['endpoints']

    OPCODE_READ = 0x00
    OPCODE_WRITE = 0x01

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

    assert endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
    assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"

    while not (bus.recv(timeout=0) is None): pass
    
    set_closed_loop_control(node_id_0)
    set_closed_loop_control(node_id_1)

    path_control_mode = "axis0.controller.config.control_mode"
    send_message(node_id_0, path_control_mode, 1)
    send_message(node_id_1, path_control_mode, 1)
    
    path_torque_constant = 'axis0.config.motor.torque_constant'
    send_message(node_id_0, path_torque_constant, 0.09)
    send_message(node_id_1, path_torque_constant, 0.09)
    
    path_enable_vel_limit = "axis0.controller.config.enable_torque_mode_vel_limit"
    send_message(node_id_0, path_enable_vel_limit, True)
    send_message(node_id_1, path_enable_vel_limit, True)


    #Torque based oscilations
    set_torque(node_id_0, 0.3)
    set_torque(node_id_1, 0.3)
    time.sleep(5)
    
    set_torque(node_id_0, -0.1)
    set_torque(node_id_1, -0.1)
    time.sleep(5)


    
    set_idle(node_id_0)
    set_idle(node_id_1)
   

