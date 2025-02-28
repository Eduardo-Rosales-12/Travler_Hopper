

# -- start load
import struct
import json
import time
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

def motor_calibration(node_id, path):
    endpoint_id = endpoints[path]['id']
    endpoint_type = endpoints[path]['type']
    
    # Send write command
        #make sure we arent using old CAN messages 
    while not (bus.recv(timeout=0) is None): pass

    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 7), # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    print("calibrating ")

def save_config(node_id, path):
    endpoint_id = endpoints[path]['id']
    endpoint_type = endpoints[path]['type']
    
    while not (bus.recv(timeout=0) is None): pass
    
    # Send write command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x16), # 0x04: RxSdo
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))
    
    print("saved")
    
def dump_errors(node_id, path):
    endpoint_id = endpoints[path]['id']
    endpoint_type = endpoints[path]['type']
    
    while not (bus.recv(timeout=0) is None): pass
    
    # Send write command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, endpoint_id,0),
        is_extended_id=False
    ))
    
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
            break

    # Unpack and print reply
    _, _, _, return_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)
    print(f"received: {return_value}")
    

    
node_id = 0 # must match the configured node_id on your ODrive (default 0)
# -- end definitions

import can
bus = can.interface.Bus("can0", interface="socketcan")

# -- start version check
# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

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

_, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

# If one of these asserts fail, you're probably not using the right flat_endpoints.json file
assert endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"
# -- end version check

# -- start write

path_calibration = 'axis0.controller.config.pos_gain'
path_save_config = 'save_configuration'


motor_calibration(node_id, path_calibration)
time.sleep(10)
save_config(node_id, path_save_config)




