import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math
import numpy as np
import can
import struct
from datetime import datetime
import os
import csv

# -------------------------- Control Code --------------------------

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

def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),
        data=struct.pack('<f', torque),
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

# Global variables for control thread management.
stop_control = False
control_thread = None

# The control_loop function reads parameters from the UI (passed in via a dictionary)
# and then runs the control loop until stop_control becomes True.
def control_loop(params):
    global stop_control
    stop_control = False

    try:
        # Connect to CAN bus
        global bus
        bus = can.interface.Bus("can0", interface="socketcan")
        # Flush any pending messages
        while bus.recv(timeout=0) is not None:
            pass

        nodes = [0, 1]
        Motor0, Motor1 = nodes[0], nodes[1]

        set_closed_loop_control(Motor0)
        set_closed_loop_control(Motor1)
        set_torque_control_mode(Motor0)
        set_torque_control_mode(Motor1)

        # Get initial state for setpoints
        init_pos0, init_vel0 = encoder_estimates(Motor0)
        init_pos1, init_vel1 = encoder_estimates(Motor1)
        _, _, _, _, initial_theta, initial_rho, _, _ = get_state_variables(init_pos0, init_pos1, init_vel0, init_vel1)
        
        # Set starting setpoints from UI values
        current_theta_setpoint = params['theta_setpoint']
        current_rho_setpoint   = params['rho_setpoint']

        # Desired velocities for ramping the setpoints
        desired_theta_velocity = params['theta_velocity']
        desired_rho_velocity   = params['rho_velocity']

        # Create PD controllers with fixed gains from the UI
        Theta_PD_Controller = PDController(params['theta_Kp'], params['theta_Kd'], current_theta_setpoint)
        Rho_PD_Controller   = PDController(params['rho_Kp'], params['rho_Kd'], current_rho_setpoint)

        # Data logging list
        data_log = []
        start_time = time.perf_counter()
        last_setpoint_update_time = time.time()

        while not stop_control:
            current_time = time.time()
            perf_time = time.perf_counter()
            
            # Ramp the setpoints gradually (simple linear update)
            dt_setpoint = current_time - last_setpoint_update_time
            if dt_setpoint > 0:
                # Update theta setpoint
                if current_theta_setpoint < params['theta_setpoint']:
                    current_theta_setpoint = min(current_theta_setpoint + desired_theta_velocity * dt_setpoint, params['theta_setpoint'])
                elif current_theta_setpoint > params['theta_setpoint']:
                    current_theta_setpoint = max(current_theta_setpoint - desired_theta_velocity * dt_setpoint, params['theta_setpoint'])
                # Update rho setpoint
                if current_rho_setpoint < params['rho_setpoint']:
                    current_rho_setpoint = min(current_rho_setpoint + desired_rho_velocity * dt_setpoint, params['rho_setpoint'])
                elif current_rho_setpoint > params['rho_setpoint']:
                    current_rho_setpoint = max(current_rho_setpoint - desired_rho_velocity * dt_setpoint, params['rho_setpoint'])
                
                Theta_PD_Controller.setpoint = current_theta_setpoint
                Rho_PD_Controller.setpoint = current_rho_setpoint
                last_setpoint_update_time = current_time

            # Read sensor feedback
            current_position_0, current_velocity_0 = encoder_estimates(Motor0)
            current_position_1, current_velocity_1 = encoder_estimates(Motor1)
            motor0_tor = get_torque_estimate(Motor0)
            motor1_tor = get_torque_estimate(Motor1)
            
            elapsed_time = perf_time - start_time
            global_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            data_log.append([global_time, elapsed_time, current_position_0, current_position_1, motor0_tor, motor1_tor])

            # Compute state variables from encoder data
            phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel = \
                get_state_variables(current_position_0, current_position_1, current_velocity_0, current_velocity_1)
            
            # Compute PD outputs for theta and rho using fixed gains
            theta_torque = Theta_PD_Controller.update(theta, current_time)
            rho_torque   = Rho_PD_Controller.update(rho, current_time)
            
            # Compute motor torques from the PD outputs
            Motor0_Torque, Motor1_Torque = get_torques(theta_torque, rho_torque)
            
            set_torque(Motor0, Motor0_Torque)
            set_torque(Motor1, Motor1_Torque)

            # A short sleep to prevent a busy loop (adjust as needed)
            time.sleep(0.005)
            
    except Exception as e:
        print("Error in control loop:", e)
    finally:
        # On exit, set the nodes to idle.
        for node in [0, 1]:
            set_idle(node)
        print("Control loop terminated.")
        # If user requested data saving, save the log to a CSV file.
        if params['save_data']:
            now = datetime.now()
            filename = params['csv_name'] if params['csv_name'] else now.strftime("DPROBE-%H:%M_%m-%d.csv")
            file_path = os.path.join(os.getcwd(), "Data")
            os.makedirs(file_path, exist_ok=True)
            full_path = os.path.join(file_path, filename)
            try:
                with open(full_path, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(['Global Time', 'Elapsed Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque', 'Motor 1 Torque'])
                    writer.writerows(data_log)
                print("Data saved to:", full_path)
            except Exception as e:
                print("Error saving data:", e)

# -------------------------- UI Code --------------------------

class ControlUI:
    def __init__(self, root):
        self.root = root
        root.title("Robot Control UI")

        # Create frames for organization
        input_frame = ttk.Frame(root, padding="10")
        input_frame.grid(row=0, column=0, sticky="ew")
        button_frame = ttk.Frame(root, padding="10")
        button_frame.grid(row=1, column=0, sticky="ew")

        # --- Create UI elements for velocities, setpoints, and gains ---

        # Theta velocity
        ttk.Label(input_frame, text="Theta Velocity (rad/s):").grid(row=0, column=0, sticky="w")
        self.theta_vel_entry = ttk.Entry(input_frame)
        self.theta_vel_entry.insert(0, "0.1")
        self.theta_vel_entry.grid(row=0, column=1, sticky="w")

        # Rho velocity
        ttk.Label(input_frame, text="Rho Velocity (rad/s):").grid(row=1, column=0, sticky="w")
        self.rho_vel_entry = ttk.Entry(input_frame)
        self.rho_vel_entry.insert(0, "0.1")
        self.rho_vel_entry.grid(row=1, column=1, sticky="w")

        # Theta setpoint
        ttk.Label(input_frame, text="Theta Setpoint (rad):").grid(row=2, column=0, sticky="w")
        self.theta_set_entry = ttk.Entry(input_frame)
        self.theta_set_entry.insert(0, "3.323")
        self.theta_set_entry.grid(row=2, column=1, sticky="w")

        # Rho setpoint
        ttk.Label(input_frame, text="Rho Setpoint (rad):").grid(row=3, column=0, sticky="w")
        self.rho_set_entry = ttk.Entry(input_frame)
        self.rho_set_entry.insert(0, "3.16")
        self.rho_set_entry.grid(row=3, column=1, sticky="w")

        # Theta gains
        ttk.Label(input_frame, text="Theta Kp:").grid(row=4, column=0, sticky="w")
        self.theta_Kp_entry = ttk.Entry(input_frame)
        self.theta_Kp_entry.insert(0, "3")
        self.theta_Kp_entry.grid(row=4, column=1, sticky="w")

        ttk.Label(input_frame, text="Theta Kd:").grid(row=5, column=0, sticky="w")
        self.theta_Kd_entry = ttk.Entry(input_frame)
        self.theta_Kd_entry.insert(0, "0.35")
        self.theta_Kd_entry.grid(row=5, column=1, sticky="w")

        # Rho gains
        ttk.Label(input_frame, text="Rho Kp:").grid(row=6, column=0, sticky="w")
        self.rho_Kp_entry = ttk.Entry(input_frame)
        self.rho_Kp_entry.insert(0, "3")
        self.rho_Kp_entry.grid(row=6, column=1, sticky="w")

        ttk.Label(input_frame, text="Rho Kd:").grid(row=7, column=0, sticky="w")
        self.rho_Kd_entry = ttk.Entry(input_frame)
        self.rho_Kd_entry.insert(0, "0.35")
        self.rho_Kd_entry.grid(row=7, column=1, sticky="w")

        # Save data?
        self.save_data_var = tk.BooleanVar(value=False)
        self.save_data_check = ttk.Checkbutton(input_frame, text="Save Data?", variable=self.save_data_var)
        self.save_data_check.grid(row=8, column=0, sticky="w")

        # CSV file name
        ttk.Label(input_frame, text="CSV Filename:").grid(row=8, column=1, sticky="w")
        self.csv_name_entry = ttk.Entry(input_frame)
        self.csv_name_entry.insert(0, "data.csv")
        self.csv_name_entry.grid(row=8, column=2, sticky="w")

        # --- Buttons ---
        self.execute_button = ttk.Button(button_frame, text="Execute Control", command=self.execute_control)
        self.execute_button.grid(row=0, column=0, padx=5)
        self.abort_button = ttk.Button(button_frame, text="Abort Control", command=self.abort_control)
        self.abort_button.grid(row=0, column=1, padx=5)

        # A label for status messages
        self.status_label = ttk.Label(root, text="Status: Idle")
        self.status_label.grid(row=2, column=0, sticky="w", padx=10, pady=5)

    def execute_control(self):
        # Gather parameters from the UI
        try:
            params = {
                'theta_velocity': float(self.theta_vel_entry.get()),
                'rho_velocity': float(self.rho_vel_entry.get()),
                'theta_setpoint': float(self.theta_set_entry.get()),
                'rho_setpoint': float(self.rho_set_entry.get()),
                'theta_Kp': float(self.theta_Kp_entry.get()),
                'theta_Kd': float(self.theta_Kd_entry.get()),
                'rho_Kp': float(self.rho_Kp_entry.get()),
                'rho_Kd': float(self.rho_Kd_entry.get()),
                'save_data': self.save_data_var.get(),
                'csv_name': self.csv_name_entry.get().strip()
            }
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numeric values.")
            return

        global control_thread, stop_control
        stop_control = False
        control_thread = threading.Thread(target=control_loop, args=(params,), daemon=True)
        control_thread.start()
        self.status_label.config(text="Status: Running control...")

    def abort_control(self):
        global stop_control, control_thread
        stop_control = True
        if control_thread is not None:
            control_thread.join(timeout=2)  # wait briefly for the thread to exit
        self.status_label.config(text="Status: Control aborted.")

# -------------------------- Main --------------------------

if __name__ == "__main__":
    root = tk.Tk()
    app = ControlUI(root)
    root.mainloop()
