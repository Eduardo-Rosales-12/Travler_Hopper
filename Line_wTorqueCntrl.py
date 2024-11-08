import odrive
import time
import can
import struct
import json 

def set_position(node_id, position):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x00),  # 0x00 or 0x0D: Set_Input_Pos (check your device documentation)
        data=struct.pack('<f', position),  # Desired position value
        is_extended_id=False
    ))

def set_closed_loop_control(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
        data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))
    
    # Wait for axis to enter closed loop control
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
            _, state, _, _ = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8:  # AxisState.CLOSED_LOOP_CONTROL
                break
            
def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
        data=struct.pack('<I', 1),  # 1: idle
        is_extended_id=False
    ))

if __name__ == "__main__":
    node_id_0 = 0
    node_id_1 = 1
    bus = can.interface.Bus("can0", interface="socketcan")

    with open('flat_endpoints.json', 'r') as f:
        endpoint_data = json.load(f)
        endpoints = endpoint_data['endpoints']
    
    time.sleep(0.5)
    # Set motors to closed-loop control mode
    set_closed_loop_control(node_id_0)
    set_closed_loop_control(node_id_1)
    
    # Apply a single position command to each motor
    target_position_0 = 0.4  # Adjust position as needed
    target_position_1 = 0.4  # Adjust position as needed
    
    set_position(node_id_0, target_position_0)
    set_position(node_id_1, target_position_1)

    print("Position commands sent to motors.")
    time.sleep(8)  # Hold position for a moment

    # Reset motors to idle
    set_position(node_id_0, 0)
    set_position(node_id_1, 0)
    time.sleep(4)

    set_idle(node_id_0)
    set_idle(node_id_1)
