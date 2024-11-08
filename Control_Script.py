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



if __name__ == "__main__":


    if args.mode == "Hopping":
        while True:
            
            if keyboard.is_pressed('a'):


            if Motion_Phase.get_state(Centerbar_Deriv, idle_latch) == "idle":


            elif Motion_Phase.get_state(Centerbar_Deriv, idle_latch) == "compression":


            elif Motion_Phase.get_state(Centerbar_Deriv, idle_latch) == "extension":


            elif Motion_Phase.get_state(Centerbar_Deriv, idle_latch) == "flight":


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

