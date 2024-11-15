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
import csv 

class PDController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd

    def update(self, target_val, measured_val, dt):
        Proportional_Error = target_val - measured_val
        Derivative_Error = (target_val - measured_val)/dt
        Corrected_Signal = self.Kp * Proportional_Error + self.Kd * Derivative_Error
        return Corrected_Signal

class Hopper_State_Machine:
    def __init__(self, Kc, Cd):
        self.Kc = Kc
        self.Cd = Cd 
        self.state = "Idle"

    def compression(self):
        self.Kc = -1
        self.Cd = 1
        return 

    def extension(self):  
        self.Kc = -800
        self.Cd = 1
        return 
    
    def flight(self): #I know we discussed swithcing the spring stiffness during flight phase but it is already being switch once it enterss compression mode so do we need to switch if no torques are being applied?
        return        
 
    
    def get_state(self, Centerbar_Length, Force_Estimate, Previous_State, latch_status):
        Centerbar_Compression_Target = 0.15
        Centerbar_Extension_Target = 0.20
        Threshold = 10
        
        Compression_Centerbar_Length_Error = math.abs(Centerbar_Compression_Target - Centerbar_Length)/Target_Centerbar_Length
        Extension_Centerbar_Length_Error = math.abs(Centerbar_Extension_Target - Centerbar_Length)/Target_Centerbar_Length
         
        if latch_status == 0: 
            return "idle"
        
        elif latch_status == 1:
            if (Force_Estimate > Threshold): 
                return "compression"       
                
            if (Compression_Centerbar_Length_Error <= 0.02):
                return "extension"
                
            if (Extension_Centerbar_Length_Error <= 0.02):
                return "flight"
                
            else:
                return Previous_State



def get_state_variables(encoder0_pos_estimate, encoder1_pos_estimate, encoder0_vel_estimate, encoder1_vel_estimate):
    reference_point = 0.25
    #Converting the absolute encoder estimated into radians with refernce the the vertical axis (this is need as it was the way the jacobian was derived)
    phi_1 = (math.pi/2) + ((reference_point - encoder0_pos_estimate) * (math.pi*2))
    phi_2 = ((3*math.pi)/2) + ((encoder1_pos_estimate - reference_point) * (math.pi*2))

    #The velcoities estimates must be slightly modified to get ensure that they corespond to the definition of phi_1 and phi_2
    #The velocity of encoder1 must be switched to negative as phi2 is measured cw with respect to the vertical axis but the encoder velocity is 
    #taken ccw with respect to the absolute zero position. 
    phi_1_vel = encoder0_vel_estimate
    phi_2_vel = -encoder1_vel_estimate
    
    #phi1 and phi2 are actually swithched in our set up when comparing it to the derived kinematics
    Upper_Link_Len = 0.100 #m
    Lower_Link_Len = 0.200 #mqq
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

    #Calculate Wibblit state variables
    #Using the calculated values for phi1 and phi2 the new coordinate system can be calculated in terms of theta and rho (aka wibblets)
    theta = 0.5*(phi_1 + phi_2)
    rho = 0.5*(phi_1 - phi_2) + math.pi

    #set up jacobian to convert polar to wibblit system
    Jacobian = np.array([[0.5, 0.5], [0.5, -0.5]])
    Polar_Velocity_Vector = [[phi_1_vel], [phi_2_vel]]
    
    #Map the angular velocities of phi1 and phi2 to the wibblit velocities using the perviously defined jacobian 
    wibblit_deriv = np.dot(Jacobian, Polar_Velocity_Vector)
    theta_vel= Polar_Velocity_Vector[0][0]
    rho_vel = Polar_Velocity_Vector[1][0]
    
    return Leg_Geometry_Dict, phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel


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
    Motor1_Torque = Torques[0]
    Motor2_Torque = -Torques[1]
    return Motor1_Torque, Motor2_Torque

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

def get_encoder_estimate(node_id):
    
   # Send read command
#     bus.send(can.Message(
#         arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
#         data=struct.pack('<BHB', OPCODE_READ, 195, 0),
#         is_extended_id=False
#     ))
    
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09), # 0x04: RxSdo
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09):  # Response with encoder data
            pos_return_value, vel_return_value = struct.unpack_from('<ff', msg.data)
            return pos_return_value, vel_return_value

def get_torque_estimate(node_id):

#    # Send read command
#     bus.send(can.Message(
#         arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
#         data=struct.pack('<BHB', OPCODE_READ, 363, 0),
#         is_extended_id=False
#     ))

    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x1c), 
        data=b'',
        is_extended_id=False
    ))

    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x1c): 
            break
        
    # Unpack and print reply
   # _, _, _, torque_return_value = struct.unpack_from('<BHB' + 'f', msg.data)
    
    torque_target, torque_return_value = struct.unpack_from('<ff', msg.data)

    return torque_return_value

def get_Forces(Jacobian, Torque_Vector):
    Jacobian_Inverse = np.linalg.inv(Jacobian)
    Forces = Torque_Vector @ Jacobian_Inverse
    Force_Magnitude = math.hypot(Forces[0][0], Forces[0][1])

    return Force_Magnitude

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
    #Set Up motor parameters 
    Motor1 = nodes[0] #Front right linkage (phi2)
    Motor2 = nodes[1] #Back left linkage (phi1)

    #Set Closed loop control 
    set_closed_loop_control(Motor1)
    set_closed_loop_control(Motor2)

    # Wait for each node to enter closed loop control by scanning heartbeat messages
    for node_id in nodes:
        for msg in bus:
            if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
                    break

    #Initalize state object 
    State_Machine = Hopper_State_Machine(0,0)
    Previous_State = "idle"
    #Initalizing Toe Position PD controller in Flight 
    Theta_PD_Controller = PDController(0.3,0.1)
    Rho_PD_Controller = PDController(0.2,0.1)

    #Initalizing Centerbar PD Controller in Compression 
    Centerbar_Compression_PD_Controller = PDController(0.2,0.1)

    #Initalizing Centerbar PD Controller in Compression 
    Centerbar_Extension_PD_Controller = PDController(0.2,0.1)
    
    #Initalize the latch status 
    latch_status = 0
    
    #Check the hopping mode
    initial_position = 0.25
    offset = 0.015
    set_position_control_mode(Motor1)
    set_position_control_mode(Motor2)
    
    set_position(Motor1,initial_position - offset)
    set_position(Motor2,initial_position)
    time.sleep(3)

    #Setup data log
    data_log = []
    
    #State_Vector
    State_Vector = []

    #Set Torque Control 
    set_torque_control_mode(Motor1)
    set_torque_control_mode(Motor2)
    
    #Get the initial motor position estimates
    initial_motor1_pos, initial_motor1_vel = get_encoder_estimate(Motor1)
    initial_motor2_pos, initial_motor2_vel = get_encoder_estimate(Motor2)
    
    Start_Time = time.perf_counter()
    Elapsed_Start_Time = time.perf_counter()
    
    Initial_Leg_Geometry, initial_phi_1, initial_phi_2, initial_phi_1_vel, initial_phi_2_vel, initial_theta, initial_rho, initial_theta_vel, initial_rho_vel  = get_state_variables(initial_motor1_pos, initial_motor2_pos, initial_motor1_vel, initial_motor2_vel)
    Initial_Toe_Position_Estimate = FK(Initial_Leg_Geometry["Leg Length"], Initial_Leg_Geometry["Leg Angle"])
    Initial_Centerbar_Length = Initial_Leg_Geometry[1]
    initial_centerbar_length_condition = 0

    #Initialize the state tracker
    Extension_Tracker = False
    Compression_Tracker = False 
    
    while running:
        #grab initial motor position estimate 
        motor1_pos, motor1_vel = get_encoder_estimate(Motor1)
        motor2_pos, motor2_vel = get_encoder_estimate(Motor2)
        motor1_tor = get_torque_estimate(Motor1)
        motor2_tor = get_torque_estimate(Motor2)
        
        #Calculate the length of the centerbar length 
        Leg_Geometry, phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel  = get_state_variables(motor1_pos, motor2_pos, motor1_vel, motor2_vel)
        Toe_Position = FK(Leg_Geometry["Leg Length"], Leg_Geometry["Leg Angle"])
        Centerbar_Length = Toe_Position[1]
        
        #Calculate Force 
        Jacobian = get_Jacobian(Leg_Geometry["Leg Length"], Leg_Geometry["Leg Angle"], Leg_Geometry["Beta"])
        Force_z = get_Forces(Jacobian, [[motor1_tor], [motor2_tor]])
        
        #Check and save the system state
        State =  State_Machine.get_state(Centerbar_Length, Force_z, Previous_State, latch_status)
        print(State)

        #Determine elapsed time
        New_Time = time.perf_counter()
        elapsed_time = New_Time - Elapsed_Start_Time
        
        #Save Data
        data_log.append([elapsed_time, motor1_pos, motor2_pos, motor1_tor, motor2_tor, State])


        if State == "idle":
            latch_status = 1
            State = "compression"

        elif State == "compression":
            State_Machine.compression()
            Target_Centerbar_Length = 0.15
            dt = New_Time - Start_Time
            
            Force = Centerbar_Compression_PD_Controller.update(Target_Centerbar_Length, Centerbar_Length, dt)
            Motor1_Torque, Motor2_Torque = get_Torques(Jacobian, [[0],[Force]])
            
            set_torque(Motor1, Motor1_Torque)
            set_torque(Motor2, Motor2_Torque)


        elif State == "extension":
            State_Machine.extension()
            Target_Centerbar_Length = 0.25
            dt = New_Time - Start_Time

            Force = Centerbar_Extension_PD_Controller.update(Target_Centerbar_Length, Centerbar_Length, dt)
            Motor1_Torque, Motor2_Torque = get_Torques(Jacobian, [[0],[Force]])
            
            set_torque(Motor1, Motor1_Torque)
            set_torque(Motor2, Motor2_Torque)
        
        elif State == "flight":
            target_rho = 0.5 * math.pi
            target_theta = 3.14
            dt = New_Time - Start_Time

            #pass the respective parameters through the each pd loop 
            theta_torque = Theta_PD_Controller.update(target_theta, theta, dt)
            rho_torque = Rho_PD_Controller.update(target_rho, rho, dt)

            #Wibblit torques to real-world torques
            Motor1_Torque, Motor2_Torque = get_torques(theta_torque, rho_torque)
            
            set_torque(Motor0, Motor1_Torque)
            set_torque(Motor1, Motor2_Torque)

            
        else:
            print("Phase state could not be determined.")
            
        #Redfine the inital motor positions
        Initial_Centerbar_Length = Centerbar_Length
        Start_Time = New_Time
        Previous_State = State
    
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
file_path = "/home/traveler/Downloads/Data/04:05:-11-10-24.csv"

#Start Logging Data
print("Saving Data to: " + file_path)

#define file header 
csv_header = ['Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque','Motor 1 Torque', 'Force']

with open(file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(csv_header)
    
with open(file_path, mode='a', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(data_log)
print("Save Complete")

