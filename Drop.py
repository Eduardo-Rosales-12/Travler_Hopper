import csv  # Added import
import can
import struct
import time
import math
import numpy as np
from datetime import datetime
import os


# Function for encoder estimates with error handling
def encoder_estimates(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09),
        data=b'',
        is_extended_id=False
    ))
    msg = bus.recv(timeout=1.0)  # Added a timeout
    if msg and msg.arbitration_id == (node_id << 5 | 0x09):
        try:
            pos_return_value, vel_return_value = struct.unpack_from('<ff', msg.data)
            return pos_return_value, vel_return_value
        except struct.error:
            print(f"Error unpacking data for node {node_id}: {msg.data}")
            return None, None
    else:
        print(f"No response or invalid message for node {node_id}")
        return None, None


if __name__ == "__main__":
    soft_start_duration = 2.0
    nodes = [0, 1]
    
    target_rho = 3.0
    target_theta = 3.14
    
    Theta_PD_Controller = PDController(5, 0.05, target_theta)
    Rho_PD_Controller = PDController(5, 0.2, target_rho)
    
    bus = can.interface.Bus("can0", interface="socketcan")
    
    while not (bus.recv(timeout=0) is None):
        pass

    Motor0, Motor1 = nodes[0], nodes[1]

    set_closed_loop_control(Motor0)
    set_closed_loop_control(Motor1)
    set_torque_control_mode(Motor0)
    set_torque_control_mode(Motor1)
    
    # Setup data log
    data_log = []

    # Determine elapsed time
    New_Time = time.perf_counter()
    Elapsed_Start_Time = time.perf_counter()
    
    try:
        while True:
            New_Time = time.perf_counter()  # Update time inside loop
            
            current_position_0, current_velocity_0 = encoder_estimates(Motor0)
            current_position_1, current_velocity_1 = encoder_estimates(Motor1)
            
            if None in (current_position_0, current_velocity_0, current_position_1, current_velocity_1):
                continue  # Skip this iteration if data is invalid

            elapsed_time = New_Time - Elapsed_Start_Time
            
            phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel = get_state_variables(
                current_position_0, current_position_1, current_velocity_0, current_velocity_1
            )

            theta_torque = Theta_PD_Controller.update(theta, elapsed_time)
            rho_torque = Rho_PD_Controller.update(rho, elapsed_time)

            Motor0_Torque, Motor1_Torque = get_torques(theta_torque, rho_torque)
            
            set_torque(Motor0, Motor0_Torque)
            set_torque(Motor1, Motor1_Torque)
            
            data_log.append([elapsed_time, current_position_0, current_position_1, Motor0_Torque, Motor1_Torque])
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Setting nodes to idle...")
        for node_id in nodes:
            set_idle(node_id)
        print("Nodes set to idle. Exiting program.")
        
        # Get the current date and time
        now = datetime.now()

        # Format the filename
        filename = now.strftime("DROP-%H:%M_%m-%d.csv")

        # Set up file to save data 
        file_path = "/home/traveler/Downloads/Data/DROP/"
        
        # Ensure the directory exists
        os.makedirs(file_path, exist_ok=True)
        
        full_path = os.path.join(file_path, filename)
        
        # Start Logging Data
        print("Saving Data to: " + full_path)

        # Define file header 
        csv_header = ['Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque', 'Motor 1 Torque']

        with open(full_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(csv_header)
            writer.writerows(data_log)  # Combined writing rows into a single block
            print("Save Complete")
