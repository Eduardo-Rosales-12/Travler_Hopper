import can
import struct
import time
import math
import numpy as np

class PDController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd

    def update(self, target_val, measured_val, deriv):
        Proportional_Error = target_val - measured_val
        Derivative_Error = deriv
        Corrected_Signal = self.Kp * Proportional_Error + self.Kd * Derivative_Error
        return Corrected_Signal
    
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
        data=struct.pack('<fhh',position,int(ff_vel),(ff_tor)),
        is_extended_id=False
    ))

def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))
    
# Function to get position and velocity feedback from the encoder
def encoder_estimates(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09),  # Request for encoder data
        data=b'',
        is_extended_id=False
    ))
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09):  # Response with encoder data
            pos_return_value, vel_return_value = struct.unpack_from('<ff', msg.data)
            return pos_return_value, vel_return_value


def get_state_variables(encoder0_pos_estimate, encoder1_pos_estimate, encoder0_vel_estimate, encoder1_vel_estimate):
    #Ecnoder0 measures the position of the front right linkage, while encoder1 measures the position of the back left linkage 
    #This means ecnoder0 corresponds to phi1, while encoder1 corresponds to phi2
    reference_point = 0.25
    
    #Converting the absolute encoder estimated into radians with refernce the the vertical axis (this is need as it was the way the jacobian was derived)
    phi_1 = (math.pi/2) + ((reference_point - encoder0_pos_estimate) * (math.pi*2))
    phi_2 = ((3*math.pi)/2) + ((encoder1_pos_estimate - reference_point) * (math.pi*2))
    
    #Using the calculated values for phi1 and phi2 the new coordinate system can be calculated in terms of theta and rho (aka wibblets)
    theta = 0.5*(phi_1 + phi_2)
    rho = 0.5*(phi_1 - phi_2) + math.pi

    #The velcoities estimates must be slightly modified to get ensure that they corespond to the definition of phi_1 and phi_2
    #The velocity of encoder1 must be switched to negative as phi2 is measured cw with respect to the vertical axis but the encoder velocity is 
    #taken ccw with respect to the absolute zero position. 
    phi_1_vel = encoder0_vel_estimate
    phi_2_vel = -encoder1_vel_estimate

    #set up jacobian to convert polar to wibblit system
    Jacobian = np.array([[0.5, 0.5], [0.5, -0.5]])
    Polar_Velocity_Vector = [[phi_1_vel], [phi_2_vel]]
    
    #Map the angular velocities of phi1 and phi2 to the wibblit velocities using the perviously defined jacobian 
    wibblit_deriv = np.dot(Jacobian, Polar_Velocity_Vector)
    theta_vel= Polar_Velocity_Vector[0][0]
    rho_vel = Polar_Velocity_Vector[1][0]
    
    #print(phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel)
    return phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel

def get_torques(theta_torque, rho_torque)
    wibblit_torques = [[theta_torque_cmd], [rho_torque_cmd]]
    #convert wibblit torques back into orginal coordinate system 
    motor_torque_cmd = np.dot(Jacobian.transpose(), wibblit_torques)

    #In the get_state_variables function we redefine the angular position estimate to be measured cw with reference to the vertical. To reconvert to normal torques this means 
    #we must multiply the torque of back left linkage by a negative
    Motor0_Torque = motor_torque_cmd[0][0]
    Motor1_Torque = -motor_torque_cmd[1][0]
    
    return Motor0_Torque, Motor1_Torque

if __name__ == "__main__":

    soft_start_duration = 2.0  # Duration of the soft start in seconds
    nodes = [0, 1]  # Node IDs for node 0 and node 1
    
    Theta_PD_Controller = PDController(0.3,0.1)
    Rho_PD_Controller = PDController(0.2,0.1)
    
    target_rho = 0.5 * math.pi
    target_theta = 3.14
    bus = can.interface.Bus("can0", interface="socketcan")
    
    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None):
       pass

    #Set Up motor parameters 
    Motor0 = nodes[0] #Front right linkage (phi2)
    Motor1 = nodes[1] #Back left linkage (phi1)


    #Set Closed loop control 
    set_closed_loop_control(Motor0)
    set_closed_loop_control(Motor1)

    #Set Torque Control 
    set_torque_control_mode(Motor0)
    set_torque_control_mode(Motor1)


    # Run control loop to reach target position
    try:
        
        while True:      
            #Get position and velocity estimates
            current_position_0, current_velocity_0 = encoder_estimates(Motor0)
            current_position_1, current_velocity_1 = encoder_estimates(Motor1)
            
            #print("Encoder State: ",current_position_0, current_position_1, current_velocity_0, current_velocity_1)
            #Get system state variables
            phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel = get_state_variables(current_position_0, current_position_1, current_velocity_0, current_velocity_1)

            #pass the respective parameters through the each pd loop 
            theta_torque = Theta_PD_Controller.update(target_theta, theta, theta_vel)
            rho_torque = Rho_PD_Controller.update(target_rho, rho, rho_vel)

            #Wibblit torques to real-world torques
            Motor0_Torque, Motor1_Torque = get_torques(theta_torque, rho_torque)
            
            set_torque(Motor0, Motor0_Torque)
            set_torque(Motor1, Motor1_Torque)
            
            print(motor_torque_cmd)    
            
#         # Enter holding phase
#         while True:
#             for node_id in nodes:
#              theta_torque_cmd = Theta_PD_Controller.update()
#         rho_torque_cmd = Rho_PD_Controller.update()
#         Jacobian = [[0.5 0.5], [0.5 -0.5]]
#         wibblet_torque = [[theta_torque_cmd], [rho_torque_cmd]]
#         motor_torque_cmd = np.dot(Jacobian.transpose(), wibblet_torque)
#         
# 
# 
#             time.sleep(0.05)  # Control loop delay for holding position

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Setting nodes to idle...")
        # Set each node to idle mode
        for node_id in nodes:
            set_idle(node_id)
        print("Nodes set to idle. Exiting program.")
