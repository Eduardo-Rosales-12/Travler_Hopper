import numpy as np
import can 
import struct 
import time 
import odrive
import math
import argparse
from odrive.enums import *
import time
from odrive.utils import *
import asyncio  # Import asyncio for async support


def get_encoder_values_USB(odrv_object):
    encoder_pos = (odrv_object.axis0.encoder.pos_estimate)/360 #degrees
    encoder_vel = (odrv_object.axis0.encoder.vel_estimate)/360 #degrees/s


def get_leg_geometry(Phi_1, Phi_2):
    Upper_Link_Len = 100 #mm
    Lower_Link_Len = 200 #mm
    Toe_Len = 47.5 #mm
    theta = 0.5*(Phi_1 + Phi_2)
    gamma = 0.5*(Phi_2 - Phi_1)
    len = Toe_Len + Upper_Link_Len*math.cos(gamma) + math.sqrt((Lower_Link_Len)**2 + (Upper_Link_Len)**2*(math.sin(gamma)**2))
    beta = -Upper_Link_Len*math.sin(gamma)*(1 + ((Upper_Link_Len * math.cos(gamma))/math.sqrt((Lower_Link_Len)**2 - (Upper_Link_Len)**2*math.sin(gamma)**2)))
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
    Jacobian = np.array([[0.5*(-beta*math.sin(theta) + len*math.cos(theta)), 0.5*(beta*math.sin(theta) + len*math.cos(theta))], [ 0.5*(-beta *math.cos(theta) - len*math.sin(theta)), 0.5*(beta*math.cos(theta) - len*math.sin(theta))]])
    return Jacobian

def get_Torques(Jacobian, Forces):
    Torques = Jacobian.transpose()*Forces
    Max_Torque = 3.5 #Nm

    for Torque in Torques:
        if Torque > Max_Torque:
            Torque = Max_Torque

    return Torques

def get_gamma(l):
    #l1 = 100.0 #mm
    #l2 = 200.0 #mm
    #l3 = 47.5 #mm
    #gamma = [2*math.atan(math.sqrt((-l**2 + 2*l*l1 + 2*l*l3 - l1**2 - 2*l1*l3 + l2**2 - l3**2)/(l**2 + 2*l*l1 - 2*l*l3 + l1**2 - 2*l1*l3 - l2**2 + l3**2))), 
    #        -2*math.atan(math.sqrt(-(l**2 - 2*l*l1 - 2*l*l3 + l1**2 + 2*l1*l3 - l2**2 + l3**2)/(l**2 + 2*l*l1 - 2*l*l3 + l1**2 - 2*l1*l3 - l2**2 + l3**2)))]
    #print(gamma)
    #value = (-Upper_Link_Len**2 + Lower_Link_Len**2 - Toe_Len**2 + 2*Toe_Len*len - len**2) / (2 * Upper_Link_Len * (Toe_Len - len))*(180/math.pi)
    #value = ((4*len**2 - 380*len - 110975)/(800*len - 38000))

    #if -1 <= value <= 1:
    #    gamma = -math.acos(value)
    #else:
    #    print("Value out of range for acos:", value)
    #    gamma = float('nan')  # or handle the error appropriately
    #    print(gamma)
    #return gamma 

    l1 = 100.0 #mm
    l2 = 200.0 #mm
    l3 = 47.5 #mm
    # Numerator and denominator for the first expression inside atan
    numerator_1 = (-l**2 + 2*l*l1 + 2*l*l3 - l1**2 - 2*l1*l3 + l2**2 - l3**2)
    numerator_2 = (l**2 - 2*l*l1 - 2*l*l3 + l1**2 + 2*l1*l3 - l2**2 + l3**2)
    denominator = (l**2 + 2*l*l1 - 2*l*l3 + l1**2 - 2*l1*l3 - l2**2 + l3**2)

    quotient1 = numerator_1/denominator
    quotient2 = numerator_2/denominator


    # Ensure the denominator is not zero to avoid division by zero
    if denominator != 0 and quotient1 >= 0:
        sqrt_term_1 = math.sqrt(quotient1)
        gamma = 2 * math.atan(sqrt_term_1)
        return gamma 
    
    elif denominator != 0 and quotient2 >= 0:
        sqrt_term_2 = math.sqrt(quotient2)
        gamma = -2 * math.atan(sqrt_term_2)
        return gamma
    else:
        return "nan"

# Function to create a position command message for ODrive
def create_position_command(node_id, position):
    # Create CAN message
    msg = can.Message(
    arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
    data=struct.pack('<ff', position, 0.0), # 1.0: velocity, 0.0: torque feedforward
    is_extended_id=False)
    return msg

# Function to send the CAN message
def send_position_command(bus, msg):
    try:
        bus.send(msg)
        print(f"Message Sent: {msg}")
    except can.CanError as e:
        print(f"Error sending CAN message: {e}")


if __name__ == "__main__":

    # Circle parameters
    radius = 20  # Adjust according to your setup, in encoder counts or mm
    num_steps = 40  # Number of steps in one full circle
    speed = 0.1  # Time delay between each step (adjust to control speed)

    Torque_Commands_Array_Motor1 = []
    Torque_Commands_Array_Motor2 = []

    #Set up CAN interface 
    bus = can.interface.Bus(channel='can0', interface='socketcan')

    #ODrive CAN ID 
    odrv1_id = 0
    odrv2_id = 1
    
    #make sure we arent using old CAN messages 
    while not (bus.recv(timeout=0) is None): pass


    # Put axis into closed loop control state
    bus.send(can.Message(
        arbitration_id=(odrv1_id << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    # Put axis into closed loop control state
    bus.send(can.Message(
        arbitration_id=(odrv2_id << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))
    
    # Wait for axis to enter closed loop control by scanning heartbeat messages
    for msg in bus:
        if msg.arbitration_id == (odrv1_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                break

    for msg in bus:
        if msg.arbitration_id == (odrv2_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                break



    for step in range(num_steps):

        # Calculate the angle (in radians)
        angle = 2 * math.pi * (step / num_steps)
        
        # Calculate the X and Y positions
        x_pos = radius * math.cos(angle)  # Motor 1 (X-axis)
        z_pos = (radius * math.sin(angle)) + 20  # Motor 2 (Y-axis)

        #Find the polar coords froms the cartesian coords 
        Polar_Coords = IK(x_pos, z_pos)

        #Find the phi positions from the polar coords
        Gamma = get_gamma(Polar_Coords[0])

        #Find phi1 and phi2 
        Motor_Pos1 = (Polar_Coords[1] - Gamma)/(2*math.pi)
        Motor_Pos2 = (Polar_Coords[1] + Gamma)/(2*math.pi)

        odrv1_position_command = create_position_command(odrv1_id, Motor_Pos1)
        odrv2_position_command = create_position_command(odrv2_id, Motor_Pos2)

        send_position_command(bus,odrv1_position_command)
        send_position_command(bus,odrv2_position_command)


        time.sleep(0.02)



  

