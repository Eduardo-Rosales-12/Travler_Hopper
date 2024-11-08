

# -- start load
import struct
import json
import numpy as np
import time
import math
import pytz
from datetime import datetime
import csv
import can
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import odrive
from odrive.enums import *
import pandas as pd
from matplotlib.animation import FuncAnimation

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
    pos_return_value = pos_return_value/(math.pi*2)
    
    return pos_return_value

def get_torque_estimate(node_id):
    #define torque path 
    torque_path = 'axis0.motor.torque_estimate'

    # Convert path to endpoint ID
    torque_endpoint_id = endpoints[torque_path]['id']
    torque_endpoint_type = endpoints[torque_path]['type']
    
    # Send read command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, torque_endpoint_id, 0),
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
            break
        
    # Unpack and print reply
    _, _, _, torque_return_value = struct.unpack_from('<BHB' + format_lookup[torque_endpoint_type], msg.data)
    
    return torque_return_value
        
def set_closed_loop_control(node_id):
    # Put axis into closed loop control state
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 1), # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    # Wait for axis to enter closed loop control by scanning heartbeat messages
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 1: # 8: AxisState.CLOSED_LOOP_CONTROL
                break
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

#Set Up Motor DAQ
#Set up file to save data 
file_path = "Torque_and_Position_Data.csv"

#define file header 
csv_header = ['Global Time', 'Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque','Motor 1 Torque', 'Force']

with open(file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(csv_header)
    
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


node_id_0 = 0 
node_id_1 = 1

bus = can.interface.Bus("can0", interface="socketcan")

# -- start version check
# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

msg = get_ver(node_id_0)
msg = get_ver(node_id_1)

_, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

# If one of these asserts fail, you're probably not using the right flat_endpoints.json file
assert endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

# Set closed loop control
set_closed_loop_control(node_id_0)
set_closed_loop_control(node_id_1)


time_vector = np.ones(1000000)
start_time = time.time()
time_zone = pytz.timezone("US/Pacific")


while True:
        #Read Pos
    Motor0_pos_estimate_value = get_pos_estimate(node_id_0)
    Motor1_pos_estimate_value = get_pos_estimate(node_id_1)
    
    #Read Torque
    Motor0_torque_estimate_value = get_torque_estimate(node_id_0)
    Motor1_torque_estimate_value = get_torque_estimate(node_id_1)
    
    #Calculate Timing 
    elapsed_time = time.time() - start_time
    global_time = datetime.now(pytz.utc)
    pst_time = global_time.astimezone(time_zone)
    formatted_time = pst_time.strftime("%H:%M:%S")
    

    #Save data to csv file
    new_data = [
        [formatted_time, elapsed_time, Motor0_pos_estimate_value, Motor1_pos_estimate_value, Motor0_torque_estimate_value, Motor1_torque_estimate_value]
    ]
    
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        
        for row in new_data:
            writer.writerow(row)
    time.sleep(0.001)



    


