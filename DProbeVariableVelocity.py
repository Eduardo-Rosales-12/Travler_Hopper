import can
import struct
import time
import math
import numpy as np
from datetime import datetime
import os
import csv

class PDController:
    def __init__(self, Kp, Kd, setpoint):
        self.Kp = Kp
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, measured_val, current_time):
        error = self.setpoint - measured_val
        
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        control_signal = self.Kp * error + self.Kd * derivative
        
        self.prev_error = error
        self.prev_time = current_time
        
        return control_signal
    
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

def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),
        data=struct.pack('<f', torque),
        is_extended_id=False
    ))

def set_position(node_id, position, ff_vel, ff_tor):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<fhh', position, int(ff_vel), ff_tor),
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
    wibblit_deriv = np.dot(Jacobian, Polar_Velocity_Vector)
    theta_vel = wibblit_deriv[0][0]
    rho_vel = wibblit_deriv[1][0]

    return phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel

def get_torques(theta_torque, rho_torque):
    wibblit_torques = [[theta_torque], [rho_torque]]
    Jacobian = np.array([[0.5, 0.5], [0.5, -0.5]])
    motor_torque_cmd = np.dot(Jacobian.transpose(), wibblit_torques)
    
    Motor0_Torque = -motor_torque_cmd[0][0]
    Motor1_Torque = motor_torque_cmd[1][0]
    
    return Motor0_Torque, Motor1_Torque

if __name__ == "__main__":
    soft_start_duration = 2.0
    nodes = [0, 1]
    
    # Final desired targets (in radians)
    final_target_rho = 3.16
    final_target_theta = 3.323

    # Connect to the CAN bus
    bus = can.interface.Bus("can0", interface="socketcan")
    
    # Flush any pending messages from the bus
    while not (bus.recv(timeout=0) is None):
        pass

    Motor0, Motor1 = nodes[0], nodes[1]

    set_closed_loop_control(Motor0)
    set_closed_loop_control(Motor1)
    set_torque_control_mode(Motor0)
    set_torque_control_mode(Motor1)

    # -------------------------------------------------------
    # Get the current state to initialize the setpoints.
    init_pos0, init_vel0 = encoder_estimates(Motor0)
    init_pos1, init_vel1 = encoder_estimates(Motor1)
    _, _, _, _, initial_theta, initial_rho, _, _ = get_state_variables(init_pos0, init_pos1, init_vel0, init_vel1)
    
    current_theta_setpoint = initial_theta
    current_rho_setpoint = initial_rho

    # Define desired maximum velocities (setpoint update rates) in rad/s
    desired_theta_velocity = 0.1  # Adjust as needed
    desired_rho_velocity   = 0.1  # Adjust as needed

    # Initialize PD controllers with the current (initial) setpoints
    Theta_PD_Controller = PDController(3, 0.35, current_theta_setpoint)
    Rho_PD_Controller   = PDController(3, 0.35, current_rho_setpoint)
    # -------------------------------------------------------

    # Setup data logging
    data_log = []
    Elapsed_Start_Time = time.perf_counter()
    last_setpoint_update_time = time.time()

    try:
        while True:
            current_time = time.time()
            New_Time = time.perf_counter()
            
            # Gradually update the setpoints to control velocity
            dt_setpoint = current_time - last_setpoint_update_time
            if dt_setpoint > 0:
                if current_theta_setpoint < final_target_theta:
                    current_theta_setpoint = min(current_theta_setpoint + desired_theta_velocity * dt_setpoint, final_target_theta)
                elif current_theta_setpoint > final_target_theta:
                    current_theta_setpoint = max(current_theta_setpoint - desired_theta_velocity * dt_setpoint, final_target_theta)
                
                if current_rho_setpoint < final_target_rho:
                    current_rho_setpoint = min(current_rho_setpoint + desired_rho_velocity * dt_setpoint, final_target_rho)
                elif current_rho_setpoint > final_target_rho:
                    current_rho_setpoint = max(current_rho_setpoint - desired_rho_velocity * dt_setpoint, final_target_rho)
                
                Theta_PD_Controller.setpoint = current_theta_setpoint
                Rho_PD_Controller.setpoint = current_rho_setpoint
                last_setpoint_update_time = current_time

            # Read sensor feedback
            current_position_0, current_velocity_0 = encoder_estimates(Motor0)
            current_position_1, current_velocity_1 = encoder_estimates(Motor1)
            motor0_tor = get_torque_estimate(Motor0)
            motor1_tor = get_torque_estimate(Motor1)
            
            elapsed_time = New_Time - Elapsed_Start_Time
            global_time = datetime.now()
            formatted_global_time = global_time.strftime("%Y-%m-%d %H:%M:%S.%f")
            data_log.append([formatted_global_time, elapsed_time, current_position_0, current_position_1, motor0_tor, motor1_tor])

            # Compute state variables based on encoder data
            phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel = \
                get_state_variables(current_position_0, current_position_1, current_velocity_0, current_velocity_1)
            
            # ---------------- Adaptive Gain Scheduling for Rho ----------------
            # Assume that when rho is far from vertical (vertical_rho), gravity plays a bigger role.
            base_rho_Kp = 3.0
            base_rho_Kd = 0.35
            vertical_rho = 1.57  # radians: configuration when the linkage is vertical
            if abs(rho - vertical_rho) > 0.5:
                adaptive_factor = 1.5
            else:
                adaptive_factor = 1.0
            Rho_PD_Controller.Kp = base_rho_Kp * adaptive_factor
            Rho_PD_Controller.Kd = base_rho_Kd * adaptive_factor
            # --------------------------------------------------------------------

            # Compute PD outputs for theta and rho
            theta_torque = Theta_PD_Controller.update(theta, current_time)
            rho_torque   = Rho_PD_Controller.update(rho, current_time)

            # ---------------- Feed-Forward Gravity Compensation ----------------
            # Gravity feed-forward is added only to the rho channel.
            # We assume gravity exerts no moment when rho == vertical_rho and increases with sin(angle from vertical).
            gravity_gain = 2.0  # Tune this value based on your system
            ff_rho = -gravity_gain * math.sin(rho - vertical_rho)
            rho_torque = rho_torque + ff_rho
            # ------------------------------------------------------------------

            # Compute motor torques from the computed theta and rho torques
            Motor0_Torque, Motor1_Torque = get_torques(theta_torque, rho_torque)
            
            set_torque(Motor0, Motor0_Torque)
            set_torque(Motor1, Motor1_Torque)
            
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Setting nodes to idle...")
        for node_id in nodes:
            set_idle(node_id)
        print("Nodes set to idle. Exiting program.")

        now = datetime.now()
        filename = now.strftime("DPROBE-%H:%M_%m-%d.csv")
        file_path = "/home/traveler/Traveler_Hopper_sw-bundle/Data/DPROBE"
        os.makedirs(file_path, exist_ok=True)
        full_path = os.path.join(file_path, filename)
        
        print("Saving Data to: " + full_path)
        csv_header = ['Global Time', 'Elapsed Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque', 'Motor 1 Torque']
        with open(full_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(csv_header)
            writer.writerows(data_log)
        print("Save Complete")
