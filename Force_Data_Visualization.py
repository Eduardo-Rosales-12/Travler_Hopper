import pandas as pd
import matplotlib.pyplot as plt
import math 
import numpy as np

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

def get_Jacobian(length, theta, beta):
    Jacobian = np.array([[0.5*(-beta*math.sin(theta) + length*math.cos(theta)), 0.5*(beta*math.sin(theta) + length*math.cos(theta))], 
                         [ 0.5*(-beta *math.cos(theta) - length*math.sin(theta)), 0.5*(beta*math.cos(theta) - length*math.sin(theta))]])
    return Jacobian

def get_Forces(Jacobian, Torques):
    Jacobian_Inverse = np.linalg.inv(Jacobian)
    
    Forces = Torques @ Jacobian_Inverse
    
    Force_Magnitude = math.hypot(Forces[0][0], Forces[0][1])

    return Force_Magnitude

#Load CSV data
csv_file = "/home/traveler/Downloads/Data/02:43:-11-10-24.csv"
data = pd.read_csv(csv_file)

#Set X and Y values
x = data['Time']

y1 = data['Motor 0 Position'] / (2*math.pi)
y2 = data['Motor 1 Position'] / (2*math.pi)

Leg_Geometry_List = np.array([])

for index, value in y1.items():

    Leg_Geometry_List = np.append(Leg_Geometry_List, get_leg_geometry(y1.iloc[index], y2.iloc[index]))
                                  
z1 = data['Motor 0 Torque']
z2 = data['Motor 1 Torque']

Forces_List = np.array([])

for index, value in z1.items():
    Torque_Vector = np.array([[z1.iloc[index], z2.iloc[index]]])
    Curr_Jacobian = get_Jacobian(Leg_Geometry_List[index]['Leg Length'], Leg_Geometry_List[index]['Leg Angle'],Leg_Geometry_List[index]['Beta'])
   
    Force = get_Forces(Curr_Jacobian, Torque_Vector)
    Forces_List = np.append(Forces_List,Force)

data['Force'] = data['Force'].fillna(0) + pd.Series(Forces_List)

data.to_csv(csv_file, index=False)
#Create a figure
fig, (ax1, ax2, ax3) = plt.subplots(3,1)

#First Subplot
ax1.plot(x,y1, color='blue', label = 'Motor 1')
ax1.plot(x,y2, color='red', label = 'Motor 2')

ax1.set_xlabel('Time')
ax1.set_ylabel('Position')
ax1.legend()
ax1.grid(True)


#Second Subplot
ax2.plot(x,z1, color='blue', label = 'Motor 1')
ax2.plot(x,z2, color='red', label = 'Motor 2')

ax2.set_xlabel('Time')
ax2.set_ylabel('Torque')
ax2.grid(True)

#Third Subplot
ax3.plot(x,Forces_List, color='green')

ax3.set_xlabel('Time')
ax3.set_ylabel('Force')
ax3.grid(True)



plt.tight_layout()
plt.show()
