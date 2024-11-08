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



class Hopper_State_Machine():
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
        
    
    def get_state(self, len_deriv, latch_status):

        if latch_status == 0: 
            return "idle"
        
        elif latch_status == 1:
            if len_deriv >= -0.5 & len_deriv <= 0.5: #Here I am assuming that if the lenght of the centerbar linkage is changing by 0.5mm this is just noise introduced by the encoder 
                return "idle/flight"                 #Should I rewrite the idle/flight statement in terms of reaction force felt on the ground 
            elif len_deriv < -0.5:
                return "compression"
            elif len_deriv > 0.5:
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


def get_encoder_pos_CAN(can_bus, node_id):
    # Request Encoder Position
    pos_msg = can.Message(arbitration_id=0x009 | (node_id << 5), is_extended_id=False)
    can_bus.send(pos_msg)
    
    # Wait for response and process position
    position_response = can_bus.recv(0.001)  # Wait for 1 ms to receive response --> this might be to slow so this should be tested 
    if position_response is not None:
        # Unpack the 4-byte float from the data
        encoder_pos = (struct.unpack('<f', position_response.data[0:4])[0])/360
        print(f"Encoder Position: {encoder_pos} turns")
        return encoder_pos
    else:
        return "No Response :("
    
def  get_encoder_vel_CAN(can_bus, node_id):
    # Request Encoder Velocity
    vel_msg = can.Message(arbitration_id=0x00A  | (node_id << 5), is_extended_id=False)
    can_bus.send(vel_msg)
    
    # Wait for response and process velocity
    velocity_response = can_bus.recv(0.001)  # Wait for 1 ms to receive response --> this might be to slow so this should be tested 
    if velocity_response is not None:
        # Unpack the 4-byte float from the data
        encoder_vel = (struct.unpack('<f', velocity_response.data[0:4])[0])/360
        print(f"Encoder Velocity: {encoder_vel} turns/second")
        return encoder_vel
    else:
        return "No Response :("

def get_encoder_values_USB(odrv_object):
    encoder_pos = (odrv_object.axis0.encoder.pos_estimate)/360 #degrees
    encoder_vel = (odrv_object.axis0.encoder.vel_estimate)/360 #degrees/s

    return encoder_pos, encoder_vel

def update_plot(x, z):
    x_data.append(x)
    z_data.append(z)
    line.set_xdata(x_data)
    line.set_ydata(z_data)
    ax.relim()   # Recompute the limits based on the new data
    ax.autoscale_view()  # Automatically adjust the plot view
    plt.draw()
    plt.pause(0.001)  # Pause to allow for the plot to update

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description="Travler Hopper Operation.")

    # Add arguments
    parser.add_argument("--mode", type=float,choices=['Hopping', 'Dropping', 'Dynamic Probing'], default="Hopping")

    # Parse the arguments
    args = parser.parse_args()

    #Set up CAN interface 
    CAN_Interface = 'can0'

    #ODrive CAN ID 
    Odrive_CAN_ID = 0x01

    #Initialize Can Bus Object 
    can_bus = can.interface.BUS(channel=CAN_Interface, bustype = 'socketcan')

    #Initialize phase object 
    Motion_Phase = Hopper_State_Machine(Kc=1, Cd=1)

    #Define serial number of each odrive being used 
    Odrv0_Serial = "XXXXXXXXXXXXXXXX" #Change to correct serial number 
    Odrv1_Serial = "XXXXXXXXXXXXXXXX" #Change to correct serial number 


    #Connect to odrive 
    odrv0 = odrive.find_any(Odrv0_Serial)
    odrv1 = odrive.find_any(Odrv1_Serial)

    #Calibrate Motor 1 and wait for it to complete calibration
    odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while odrv0.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    #Calibrate Motor 2 and wait for it complete calibration
    odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while odrv1.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    #Establish closed loop control and set up torque control 
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

    #Define key variables
    idle_latch = 0
    dt = 0.001

    #Set up live plotter to test control scheme 
    # Set up Matplotlib for live plotting
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots()
    x_data, z_data = [], []
    line, = ax.plot([], [], 'r-')
    ax.set_xlim(-100, 100)  # Adjust as per the workspace of the hopper
    ax.set_ylim(0, 300)     # Adjust to accommodate the maximum height of the hopper
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Z Position (mm)')
    ax.set_title('Hopper Position During Motion')

    if args.mode == "Hopping":
        while True:
            Prev_Motor1_Pos = get_encoder_pos_CAN(can_bus, 1)
            Prev_Motor2_Pos = get_encoder_pos_CAN(can_bus, 2)
            Prev_Leg_Geometry = get_leg_geometry(Prev_Motor1_Pos[0], Prev_Motor2_Pos[0])

            #Wait 1 ms before collecting the next data point
            time.sleep(dt) #this might be to slow so this should be tested 

            #Recalculate centerbar linkage length 
            Curr_Motor1_Pos = get_encoder_pos_CAN(can_bus, 1)
            Curr_Motor2_Pos = get_encoder_pos_CAN(can_bus, 2)
            Curr_Leg_Geometry= get_leg_geometry(Curr_Motor1_Pos[0], Curr_Motor2_Pos[0])
           
            #update plot 
            update_plot(Curr_Coords[0], Curr_Coords[1])
            
            #Calculate Jacobian 
            Jacobian = get_Jacobian(Curr_Motor1_Pos, Curr_Motor2_Pos)

            #Calculate the change in length of the centerbar linkage overtime 
            Centerbar_Deriv = (Curr_Leg_Geometry[0] - Prev_Leg_Geometry[0])/(dt*4) # the value that is divided is the amount of time it takes for the get_encoder_values_CAN() function 
                                                                # to execute 4 times as it is called 4 times before this line plus te additional time it takes for 1ms to pass 
                                                                # between the first and second ecnoder measurement. I would love suggestions on how to improve this :/
            #Calulcate Change in the position of robots body with respect the joint frame of refrence 
            Prev_Coords = FK(Prev_Leg_Geometry["Leg Length"], Prev_Leg_Geometry["Leg Angle"])
            Curr_Coords= FK(Curr_Leg_Geometry["Leg Length"], Curr_Leg_Geometry["Leg Angle"])
            
            if keyboard.is_pressed('a'):
                idle_latch = 1

            if Motion_Phase.get_state(Centerbar_Deriv, idle_latch) == "idle":
                #The goal here is to initiate the compresion phase by applying small torques at the motors to shrink the length of the centerbar linkage 
                if keyboard.is_pressed('b'):
                    odrv0.axis0.controller.input_torque = 0.10 #Nm
                    odrv1.axis0.controller.input_torque = -0.10 #Nm

            elif Motion_Phase.get_state(Centerbar_Deriv, idle_latch) == "compression":
                Motion_Phase.compression()

                Force_PD_Controller = PDController(Kp=Motion_Phase.K_c, Kd=Motion_Phase.Cd)
                Forces = np.array([0,Force_PD_Controller.update(Curr_Coords[1], Prev_Coords[1], dt)])
                Torques = get_Torques(Jacobian, Forces)

                odrv0.axis0.controller.input_torque = Torques[0]
                odrv1.axis0.controller.input_torque = Torques[1]

            elif Motion_Phase.get_state(Centerbar_Deriv, idle_latch) == "extension":
                Motion_Phase.extension()

                Force_PD_Controller = PDController(Kp=Motion_Phase.K_c, Kd=Motion_Phase.Cd)
                Forces = np.array([0,Force_PD_Controller.update(Curr_Coords[1], Prev_Coords[1], dt)])
                Torques = get_Torques(Jacobian, Forces)

                odrv0.axis0.controller.input_torque = Torques[0]
                odrv1.axis0.controller.input_torque = Torques[1]

            elif Motion_Phase.get_state(Centerbar_Deriv, idle_latch) == "flight":
                Motion_Phase.flight()
                Flight_Positon_Controller = PDController(Kp=0.5, Kd=0.5)
                Motor1_Position_Control_Signal = Flight_Positon_Controller.update(0,Curr_Motor1_Pos,dt)
                Motor2_Position_Control_Signal = Flight_Positon_Controller.update(0,Curr_Motor1_Pos,dt)

                odrv0.axis0.controller.input_pos = Motor1_Position_Control_Signal
                odrv1.axis0.controller.input_pos = Motor2_Position_Control_Signal

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

plt.ioff()
plt.show()