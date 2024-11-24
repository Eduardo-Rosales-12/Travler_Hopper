import can
import struct
import time
import math
import numpy as np
from datetime import datetime
import os
import csv
from datetime import datetime

class PDController:
    def __init__(self, Kp, Kd, setpoint):
        self.Kp = Kp
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, measured_val, current_time):
        Proportional_Error = self.setpoint - measured_val
        
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time
            
        Derivative_Error = (Proportional_Error - self.prev_error) / dt if dt > 0 else 0.0
        
        Corrected_Signal = self.Kp * Proportional_Error + self.Kd * Derivative_Error
        
        self.prev_error = Proportional_Error
        self.prev_time = current_time
        
        
        return Corrected_Signal
    
    def reset(self):
    
        self.prev_error = 0.0
        self.prev_time = None
    
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

def set_velocity_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),
        data=struct.pack('<II', 2, 1),
        is_extended_id=False
    ))
    
def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),
        data=struct.pack('<f', torque),
        is_extended_id=False
    ))

def set_position(node_id, position):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<f', position),
        is_extended_id=False
    ))

def set_velocity(node_id, velocity):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0d),
        data=struct.pack('<f', velocity),
        is_extended_id=False
    ))
    
def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))
    
def encoder_estimates(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09),
        data=b'',
        is_extended_id=False
    ))
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09):
            pos_return_value, vel_return_value = struct.unpack_from('<ff', msg.data)
            return pos_return_value, vel_return_value

def get_torque_estimate(node_id):

    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x1c), 
        data=b'',
        is_extended_id=False
    ))

    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x1c): 
            break
    
    torque_target, torque_return_value = struct.unpack_from('<ff', msg.data)

    return torque_return_value

def get_state_variables(encoder0_pos_estimate, encoder1_pos_estimate, encoder0_vel_estimate, encoder1_vel_estimate):
    reference_point = 0.25
    phi_1 = (math.pi / 2) + ((reference_point - encoder0_pos_estimate) * (math.pi * 2))
    phi_2 = ((3 * math.pi) / 2) + ((encoder1_pos_estimate - reference_point) * (math.pi * 2))
    
    theta = 0.5 * (phi_1 + phi_2)
    rho = 0.5 * (phi_1 - phi_2) + math.pi

    phi_1_vel = encoder0_vel_estimate
    phi_2_vel = -encoder1_vel_estimate

    Jacobian = np.array([[0.5, 0.5], [0.5, -0.5]])
    Polar_Velocity_Vector = np.array([[phi_1_vel], [phi_2_vel]])
    

    return phi_1, phi_2, phi_1_vel, phi_2_vel



if __name__ == "__main__":
    nodes = [0, 1]
    
    bus = can.interface.Bus("can0", interface="socketcan")
    
    while not (bus.recv(timeout=0) is None):
        pass

    Motor0, Motor1 = nodes[0], nodes[1]

    set_closed_loop_control(Motor0)
    set_closed_loop_control(Motor1)
    set_velocity_control_mode(Motor0)
    set_velocity_control_mode(Motor1)

    # Setup data log
    data_log = []

    # Determine elapsed time
    New_Time = time.perf_counter()
    Elapsed_Start_Time = time.perf_counter()
    
    target_position = 0.1
    target_vel = -9
    current_position_0, current_velocity_0 = encoder_estimates(Motor0)
    current_position_1, current_velocity_1 = encoder_estimates(Motor1)
    
    motor0_tor = get_torque_estimate(Motor0)
    motor1_tor = get_torque_estimate(Motor1)
    latch = 0
    latch2 = 0

    try:
        
        while True:
            print(current_position_0, current_position_1)
            New_Time = time.perf_counter()  # Update time inside loop
            current_position_0, current_velocity_0 = encoder_estimates(Motor0)
            current_position_1, current_velocity_1 = encoder_estimates(Motor1)
            motor0_tor = get_torque_estimate(Motor0)
            motor1_tor = get_torque_estimate(Motor1)
            
            elapsed_time = New_Time - Elapsed_Start_Time
            current_time = time.time()  
            global_time = datetime.now()
            
            formatted_global_time = global_time.strftime("%Y-%m-%d %H:%M:%S.%f")
            data_log.append([formatted_global_time, elapsed_time, current_position_0, current_position_1, motor0_tor, motor1_tor])

            phi_1, phi_2, phi_1_vel, phi_2_vel = get_state_variables(current_position_0, current_position_1, current_velocity_0, current_velocity_1)
            
            if (current_position_0 > target_position or current_position_1 > target_position) and (latch == 0):
                set_velocity(Motor0, target_vel)
                set_velocity(Motor1, target_vel)
                
            elif (current_position_0 < target_position or current_position_1 < target_position) and (latch2 == 0):
                last_pos0 = current_position_0
                last_pos1 = current_position_1
                latch = 1
                latch2 = 1
                
            if(latch == 1 and latch2 == 1):
                set_position_control_mode(Motor0)
                set_position_control_mode(Motor1)
                set_position(Motor0,  last_pos0-0.09)
                set_position(Motor1,  last_pos1-0.09)
    
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Setting nodes to idle...")
        for node_id in nodes:
            set_idle(node_id)
        print("Nodes set to idle. Exiting program.")


    # Get the current date and time
        now = datetime.now()

        # Format the filename
        filename = now.strftime("DPROBE-%H:%M_%m-%d.csv")

        # Set up file to save data 
        file_path = "/home/traveler/Traveler_Hopper_sw-bundle/Data/DPROBE"
        
        # Ensure the directory exists
        os.makedirs(file_path, exist_ok=True)
        
        full_path = os.path.join(file_path, filename)
        
        # Start Logging Data
        print("Saving Data to: " + full_path)

        # Define file header 
        csv_header = ['Global Time', 'Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque', 'Motor 1 Torque']

        with open(full_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(csv_header)
            writer.writerows(data_log)  # Combined writing rows into a single block
            print("Save Complete")



