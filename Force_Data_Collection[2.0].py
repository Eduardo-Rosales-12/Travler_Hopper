import struct
import json
import numpy as np
import time
import math
import pytz
from datetime import datetime
import csv
import can
import odrive
from odrive.enums import *
import pandas as pd


def get_pos_estimate(node_id):

   # Send read command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, 195, 0),
        is_extended_id=False
    ))
    
        # Await reply
    
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
            break
    # Unpack and print reply
    #_, _, _, pos_return_value = struct.unpack_from('<BHB' + 'f', msg.data)
    #pos_return_value = pos_return_value/(math.pi*2)
    
    return msg.data
    


def get_torque_estimate(node_id):

   # Send read command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, 363, 0),
        is_extended_id=False
    ))
    
        # Await reply
    
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
            break
    # Unpack and print reply
    #_, _, _, pos_return_value = struct.unpack_from('<BHB' + 'f', msg.data)
    #pos_return_value = pos_return_value/(math.pi*2)
    
    return msg.data


#Set up file to save data 
file_path = "/home/traveler/Downloads/Data/06:50:-11-7-24.csv"

#define file header 
csv_header = ['Global Time', 'Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque','Motor 1 Torque']

with open(file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(csv_header)    


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



# Start time for logging
time_zone = pytz.timezone("US/Pacific")
data_buffer = []
buffer_size = 1000
start_time = time.perf_counter()



f_s = 1000 # Hz
sample_ms = 0.001


    
while True:
    loop_start = time.perf_counter()
    
    # Calculate elapsed time in sec
    elapsed_time = loop_start - start_time
    
    # Retrieve estimates
    Motor0_pos = get_pos_estimate(node_id_0)
    Motor1_pos = get_pos_estimate(node_id_1)
    Motor0_torque = get_torque_estimate(node_id_0)
    Motor1_torque = get_torque_estimate(node_id_1)
    
    
    # Append the new data to the buffer
    data_buffer.append([elapsed_time, Motor0_pos, Motor1_pos, Motor0_torque, Motor1_torque])
    #print([formatted_time, elapsed_time, Motor0_pos, Motor1_pos])
    
    # Write buffer to file if it reaches the buffer size
    if len(data_buffer) >= buffer_size:
        break
        


            

with open(file_path, mode='a', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(data_buffer)
    
print("Data Logging Complete")
        
        
        
