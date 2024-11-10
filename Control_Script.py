import numpy as np
import can 
import struct 
import time 
import odrive
import math
import argparse
from odrive.enums import *
import matplotlib.pyplot as plt
from pynput import keyboard

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

class Hopper_State_Machine:
    def __init__(self, Kc, Cd):
        self.Kc = Kc
        self.Cd = Cd 
        self.state = "Idle"

    def compression(self):
        self.Kc = -30
        self.Cd = 1
        return 

    def extension(self):  
        self.Kc = -2000
        self.Cd = 10
        return 
    
    def flight(self): #I know we discussed swithcing the spring stiffness during flight phase but it is already being switch once it enterss compression mode so do we need to switch if no torques are being applied?
        return        
    
    def get_state(self, motor1_encoder_estimate, motor2_encoder_estimate, latch_status):
        extension_limit_value = -0.3
        compression_limit_value = -0.1
        #offset = 0.045
        
        if latch_status == 0: 
            return "idle"
        
        elif latch_status == 1:
            if (motor1_encoder_estimate <= extension_limit_value) or (motor2_encoder_estimate <= extension_limit_value): 
                return "flight"       
                
            elif (motor1_encoder_estimate < compression_limit_value or motor1_encoder_estimate > extension_limit_value) and (motor2_encoder_estimate < compression_limit_value and motor2_encoder_estimate > extension_limit_value):
                return "compression"
                
            elif (motor1_encoder_estimate >= compression_limit_value) or (motor2_encoder_estimate >= compression_limit_value):
                return "extension"
                
            else:
                return "State could not be determined!"



def get_leg_geometry(Phi_1, Phi_2):
    reference_point = -0.15
    #Converting the absolute encoder estimated into radians with refernce the the vertical axis (this is need as it was the way the jacobian was derived)
    phi_2 = (math.pi/2) - ((Phi_2 - reference_point) * (math.pi*2))
    phi_1 = ((3*math.pi)/2) + ((Phi_1 - reference_point) * (math.pi*2))
    #phi1 and phi2 are actually swithched in our set up when comparing it to the derived kinematics
    
    Upper_Link_Len = 0.100 #m
    Lower_Link_Len = 0.200 #m
    Toe_Len = 0.0475 #m
    theta = 0.5*(phi_1 + phi_2)
    gamma = 0.5*(-phi_2 + phi_1)
    length = Toe_Len + Upper_Link_Len*math.cos(gamma) + math.sqrt((Lower_Link_Len)**2 - (Upper_Link_Len**2)*(math.sin(gamma)**2))
    beta = -Upper_Link_Len*math.sin(gamma)*(1 + ((Upper_Link_Len * math.cos(gamma))/math.sqrt((Lower_Link_Len)**2 - ((Upper_Link_Len)**2)*(math.sin(gamma)**2))))
    Leg_Geometry_Dict = {
        "Leg Length": length,
        "Leg Angle": theta, 
        "Beta": beta
    }
    return Leg_Geometry_Dict

def FK(length, theta):
    x = length*math.sin(theta)
    z = length*math.cos(theta)
    Coords = (x,z)
    return Coords

def IK(x,z):
    theta = math.atan2(x,z)
    length = math.sqrt(x**2 + z**2)
    Polar_Coords = (length, theta)
    return Polar_Coords

def get_Jacobian(length, theta, beta):
    Jacobian = np.array([[0.5*(-beta*math.sin(theta) + length*math.cos(theta)), 0.5*(beta*math.sin(theta) + length*math.cos(theta))], 
                         [ 0.5*(-beta *math.cos(theta) - length*math.sin(theta)), 0.5*(beta*math.cos(theta) - length*math.sin(theta))]])
    return Jacobian

def get_Torques(Jacobian, Forces):
    Torques = np.dot(Jacobian.transpose(), Forces)
    Max_Torque = 4 #Nm
    for Torque in Torques:
        if Torque[0] > Max_Torque:
            Torque = Max_Torque

    return Torques 

# Function to put a node into closed loop control mode
def set_closed_loop_control(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
        data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

# Function to put a node into closed loop control mode
def set_torque_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
        data=struct.pack('<II', 1, 1),  # 1: Torque control, 1:Input pass through mode
        is_extended_id=False
    ))

def set_position_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
        data=struct.pack('<II', 3, 1),  # 3: Position Control, 1: Input pass through mode
        is_extended_id=False
    ))

# Function to set a torque command for a node
def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),  # 0x0e: Set_Input_Torque
        data=struct.pack('<f', torque),  # Set desired torque
        is_extended_id=False
    ))

def set_position(node_id, position):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<f', position),
        is_extended_id=False
    ))

    
def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))

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
    _, _, _, pos_return_value = struct.unpack_from('<BHB' + 'f', msg.data)
    

    return pos_return_value

def get_torque_estimate(node_id):

#    # Send read command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, 363, 0),
        is_extended_id=False
    ))

    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x05): 
            break
        
    # Unpack and print reply
    _, _, _, torque_return_value = struct.unpack_from('<BHB' + 'f', msg.data)
    
    return torque_return_value

def get_desired_force(K_spring, K_damping, centerbar_length, centerbar_length_deriv):
    Force = K_spring * centerbar_length + K_damping* centerbar_length_deriv
    return Force 

def on_press(key):
    global running
    try:
        if key.char == 'q':
            print("Stop Running Script")
            running = False
            return False
    except AttributeError:
        pass
        
if __name__ == "__main__":
    #Set up keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    running = True 

    nodes = [0, 1]  # Node IDs for node 0 and node 1
    
    bus = can.interface.Bus("can0", interface="socketcan")
    
    OPCODE_READ = 0x00
    OPCODE_WRITE = 0x01
    
    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): 
        pass
    
    # Put each node into closed loop control mode
    for node_id in nodes:
        set_closed_loop_control(node_id)
        set_position_control_mode(node_id)

    
    # Wait for each node to enter closed loop control by scanning heartbeat messages
    for node_id in nodes:
        for msg in bus:
            if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
                    break

    #Initalize stat object 
    State_Machine = Hopper_State_Machine(0,0)

    #Initalizing Toe Position PD controller
    Toe_Position_Controller = PDController(0.5, 1)
    
    #Initalize the latch status 
    latch_status = 0
    #Check the hopping mode
    initial_position = -0.15
    set_position(nodes[0],initial_position)
    set_position(nodes[1],initial_position)
    time.sleep(3)


    #Get the initial motor position estimates
    initial_motor1_pos = get_pos_estimate(nodes[0])
    initial_motor2_pos =get_pos_estimate(nodes[1])
    Start_Time = time.perf_counter()
    Elapsed_Start_Time = time.perf_counter()
    
    Initial_Leg_Geometry_Estimate = get_leg_geometry(initial_motor1_pos, initial_motor2_pos)
    Initial_Toe_Position_Estimate = FK(Initial_Leg_Geometry_Estimate["Leg Length"], Initial_Leg_Geometry_Estimate["Leg Angle"])
    Initial_Centerbar_Length = Initial_Toe_Position_Estimate[1]

    while running:

        #grab initial motor position estimate 
        motor1_pos = get_pos_estimate(nodes[0])
        motor2_pos = get_pos_estimate(nodes[1])
        motor1_tor = get_torque_estimate(nodes[0])
        motor2_tor = get_torque_estimate(nodes[0])
        
        #Determine elapsed time
        New_Time = time.perf_counter()
        elapsed_time = New_Time - Elapsed_Start_Time
        
        #Save Data
        data_log.append([elapsed_time, motor1_pos, motor2_pos, motor1_tor, motor2_tor])

        #Check the system state
        State =  State_Machine.get_state(motor1_pos, motor2_pos, latch_status)
        print(State)
        
        #Calculate the length of the centerbar length 
        Leg_Geometry = get_leg_geometry(motor1_pos, motor2_pos)
        Toe_Position = FK(Leg_Geometry["Leg Length"], Leg_Geometry["Leg Angle"])
        Centerbar_Length = Toe_Position[1]
        
        #Calculate the centerbar derivatice 
        Centerbar_Length_Deriv = (Initial_Centerbar_Length - Centerbar_Length)/(New_Time - Start_Time)
        time.sleep(0.01)
        
        if State == "idle":
            latch_status = 1

        elif State == "compression":
            set_torque_control_mode(nodes[0])
            set_torque_control_mode(nodes[1])
            State_Machine.compression()
            Force = get_desired_force(State_Machine.Kc, State_Machine.Cd, Centerbar_Length, Centerbar_Length_Deriv)
            Jacobian = get_Jacobian(Leg_Geometry["Leg Length"], Leg_Geometry["Leg Angle"], Leg_Geometry["Beta"])
            Torques = get_Torques(Jacobian, [[0],[Force]])
            set_torque(nodes[0], Torques[0])
            set_torque(nodes[1], -Torques[1])


        elif State == "extension":
            set_torque_control_mode(nodes[0])
            set_torque_control_mode(nodes[1])
            State_Machine.extension()
            Force = get_desired_force(State_Machine.Kc, State_Machine.Cd, Centerbar_Length, Centerbar_Length_Deriv)
            Jacobian = get_Jacobian(Leg_Geometry["Leg Length"], Leg_Geometry["Leg Angle"], Leg_Geometry["Beta"])
            Torques = get_Torques(Jacobian, [[0],[Force]])
            set_torque(nodes[0], Torques[0])
            set_torque(nodes[1], -Torques[1])

        
        elif State == "flight":
            extension_limit_value = -0.3
            dt = New_Time - Start_Time
            set_position_control_mode(nodes[0])
            set_position_control_mode(nodes[1])
            State_Machine.flight()
            #compensated_motor1_position = Toe_Position_Controller.update(extension_limit_value, motor1_pos, dt)
            #compensated_motor2_position = Toe_Position_Controller.update(extension_limit_value, motor2_pos, dt)
            set_position(nodes[0], extension_limit_value + 0.02)
            set_position(nodes[1], extension_limit_value)
            
        else:
            print("Phase state could not be determined.")
            
    #Redfine the inital motor positions
    Initial_Centerbar_Length = Centerbar_Length
    Start_Time = New_Time
    
listener.join()

set_position_control_mode(nodes[0])
set_position_control_mode(nodes[1])
set_position(nodes[0], initial_position)
set_position(nodes[1], initial_position)
time.sleep(1)
set_idle(nodes[0])
set_idle(nodes[1])
print("Motion Terminated")

#Set up file to save data 
file_path = "/home/traveler/Downloads/Data/02:43:-11-10-24.csv"

#Start Logging Data
print("Saving Data to: " + file_path)

#define file header 
csv_header = ['Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque','Motor 1 Torque']

with open(file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(csv_header)
    
with open(file_path, mode='a', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(data)
print("Save Complete")

    