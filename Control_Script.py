import numpy as np
import can 
import struct 
import time 
import odrive
import math
import argparse
import keyboard
from odrive.enums import *
import matplotlib.pyplot as plt

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
        self.Kc = 10 
        self.Cd = 10
        return 

    def extension(self):  
        self.K_c = -100
        self.C_d = 10 
        return 
    
    def flight(self): #I know we discussed swithcing the spring stiffness during flight phase but it is already being switch once it enterss compression mode so do we need to switch if no torques are being applied?
        print("To Infinity and Beyond")
        
    
    def get_state(self, motor1_encoder_estimate, motor2_encoder_estimate, latch_status):
        extension_limit_value = 0.3
        compression_limit_value = 0.1
        
        if latch_status == 0: 
            return "idle"
        
        elif latch_status == 1:
            if motor1_encoder_estimate >= extension_limit_value & motor2_encoder_estimate >= extension_limit_value: 
                return "idle/flight"       
                
            elif len_deriv < -0.5:
                return "compression"
                
            elif motor1_encoder_estimate > compression_limit_value & motor2_encoder_estimate > compression_limit_value:
                return "extension"
                
            else:
                return "State could not be determined!"



def get_leg_geometry(Phi_1, Phi_2):
    Upper_Link_Len = 100 #mm
    Lower_Link_Len = 200 #mm
    Toe_Len = 47.5 #mm
    theta = 0.5*(Phi_1 + Phi_2)
    gamma = 0.5*(Phi_2 - Phi_1)
    len = Toe_Len + Upper_Link_Len*math.cos(gamma) + math.sqrt((Lower_Link_Len)^2 + (Upper_Link_Len)^2*(math.sin(gamma)^2))
    beta = -Upper_Link_Len*math.sin(gamma)*(1 + ((Upper_Link_Len * math.cos(gamma))/math.sqrt((Lower_Link_Len)^2 - (Upper_Link_Len)^2*math.sin(gamma)^2)))
    Leg_Geometry_Dict = {
        "Leg Length": len,
        "Leg Angle": theta, 
        "Beta": beta
    }
    return Leg_Geometry_Dict

def FK(len, theta):
    x = len*math.sin(theta)
    z = len*math.cos(theta)
    Coords = (x,z)
    return Coords

def IK(x,z):
    theta = math.atan2(x,z)
    len = math.sqrt(x**2 + z**2)
    Polar_Coords = (len, theta)
    return Polar_Coords

def get_Jacobian(len, theta, beta):
    Jacobian = np.array([[0.5(-beta*math.sin(theta) + len*math.cos(theta)), 0.5(beta*math.sin(theta) + len*math.cos(theta))], 
                         [ 0.5(-beta *math.cos(theta) - len*math.sin(theta)), 0.5(beta*math.cos(theta) - len*math.sin(theta))]])
    return Jacobian

def get_Torques(Jacobian, Forces):
    Torques = Jacobian.transpose()*Forces
    Max_Torque = 3.5 #Nm

    for Torque in Torques:
        if Torque > Max_Torque:
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

def set_position(node_id, position, ff_vel, ff_tor):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<fhh', position, int(ff_vel), int(ff_tor)),
        is_extended_id=False
    ))

    
def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))

def get_pos_estimate(node_id):
    
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09), # 0x04: RxSdo
        data=b'',
        is_extended_id=False
    ))
    
    # Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09): # 0x05: TxSdo
            break
    # Unpack and print reply
    _, _, _, pos_return_value = struct.unpack_from('<BHB' + 'f', msg.data)
    pos_return_value = pos_return_value*(math.pi*2)
    
    return pos_return_value
    


def get_torque_estimate(node_id):

    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x1c), 
        data=b'',
        is_extended_id=False
    ))
    
    #Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x1c): 
            break
        
    # Unpack and print reply
    _, _, _, torque_return_value = struct.unpack_from('<BHB' + 'f', msg.data)
    
    return torque_return_value

if __name__ == "__main__":

    nodes = [0, 1]  # Node IDs for node 0 and node 1
    
    bus = can.interface.Bus("can0", interface="socketcan")
    
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

    #Initalize the latch status 
    latch_status = 0
    #Check the hopping mode
    if args.mode == "Hopping":
        initial_motor1_pos = get_pos_estimate(nodes[0])
        initial_motor2_pos =get_pos_estimate(nodes[1])
        
        while True:
            #grab initial motor position estimate 
            motor1_pos = get_pos_estimate(nodes[0])
            motor2_pos = get_pos_estimate(nodes[1])
            
            #Check the system state
            State =  State_Machine.get_state(motor1_pos, motor2_pos, latch_status)
            
            if State == "idle":
                latch_status = 1

            elif State == "compression":


            elif State == "extension":


            elif State == "flight":


            else:
                print("Phase state could not be determined.")

    if args.mode == "Dropping":
        while True: 
            print("I'm falling :o")

    if args.mode == "Dynamic Probing":
        while True: 
            print("I'm probing ;)")

    if args.mode == "Static Probing":
        while True: 
            print("I'm probing some more ;)")

