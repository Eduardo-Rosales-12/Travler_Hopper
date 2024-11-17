import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib import font_manager
from matplotlib.animation import FuncAnimation, PillowWriter

def get_leg_geometry(Phi_1, Phi_2):
    Upper_Link_Len = 100.0  # mm
    Lower_Link_Len = 200.0  # mm
    Toe_Len = 47.5  # mm
    theta = 0.5 * (Phi_1 + Phi_2)
    gamma = 0.5 * (Phi_2 - Phi_1)
    Len = Toe_Len + Upper_Link_Len * math.cos(gamma) + math.sqrt((Lower_Link_Len)**2 + (Upper_Link_Len)**2 * (math.sin(gamma)**2))
    beta = -Upper_Link_Len * math.sin(gamma) * (1 + ((Lower_Link_Len * math.cos(gamma)) / math.sqrt((Lower_Link_Len)**2 - (Upper_Link_Len)**2 * math.sin(gamma)**2)))
    return {"Leg Length": Len, "Leg Angle": theta, "Beta": beta}

def get_Jacobian(Len, theta, beta):
    Jacobian = np.array([[0.5 * (-beta * math.sin(theta) + Len * math.cos(theta)), 
                          0.5 * (beta * math.sin(theta) + Len * math.cos(theta))], 
                         [0.5 * (-beta * math.cos(theta) - Len * math.sin(theta)), 
                          0.5 * (beta * math.cos(theta) - Len * math.sin(theta))]])
    return Jacobian

def get_Forces(Jacobian, Torques):
    Jacobian_Inverse = np.linalg.inv(Jacobian)
    Forces = Torques @ Jacobian_Inverse
    Force_Magnitude = math.hypot(Forces[0][0], Forces[0][1])
    return Force_Magnitude

def process_data(leg_data, load_cell_data):
    data = pd.read_csv(leg_data)
    NIdata = pd.read_csv(load_cell_data)

    # Convert the 'Time' column to datetime format
    NIdata['Time'] = pd.to_datetime(NIdata['Time'], format='%m/%d/%Y %H:%M:%S.%f')

    # Initialize the 'NI_Time' column in seconds since the start time
    start_time = NIdata['Time'].iloc[0]
    NIdata['NI_Time'] = (NIdata['Time'] - start_time).dt.total_seconds()

    NIx = NIdata['NI_Time']
    NIy = (abs(NIdata['Voltage']) - abs(NIdata['Voltage'][0])) * 3
    NIy = NIy.to_numpy()


    # Set X and Y values
    x = data['Time'] - 1.2
    y1 = data['Motor 0 Position'] * (360 / 16383)
    y1 = (180 - y1) * (math.pi/180)
    y2 = data['Motor 1 Position'] * (360 / 16383) 
    y2 = (180 + y2) * (math.pi/180)

    Leg_Geometry_List = np.array([get_leg_geometry(y1.iloc[i], y2.iloc[i]) for i in range(len(y1))])
    z1 = data['Motor 0 Torque']
    z2 = data['Motor 1 Torque']

    Forces_List = np.array([
        get_Forces(get_Jacobian(Leg_Geometry_List[i]['Leg Length'], Leg_Geometry_List[i]['Leg Angle'], Leg_Geometry_List[i]['Beta']),
                np.array([[z1.iloc[i], z2.iloc[i]]]))
        for i in range(len(z1))
    ])

    data['Force'] = pd.Series(Forces_List)

    # Calculate time shift
    force_peak_time = x.iloc[np.argmax(Forces_List)]
    ni_peak_time = NIx.iloc[np.argmax(NIy)]
    time_shift = force_peak_time - ni_peak_time

    # Apply time shift to NIx
    NIx_shifted = NIx + time_shift
    NIx_shifted = NIx_shifted.to_numpy()


    return x, Forces_List, NIx_shifted, NIy

#########################################################################################################
# Load CSV data
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
csv_file = "C:/Users/er122/Downloads/Drop_Torque_and_Position_Data.csv"
NI_csv = "C:/Users/er122/Downloads/DROP1026.csv"
Drop_Data = process_data(csv_file, NI_csv)

NIy = Drop_Data[3][6300:8300]*9.8 + 0.03

NIx_shifted = Drop_Data[2][6300:8300] - 0.025

Forces_List = Drop_Data[1][305:1000]

x = Drop_Data[0][305:1000]

# Initialize the plot with two subplots
fig, (ax1) = plt.subplots(1, 1, figsize=(6, 5))
ax1= plt.gca()

ax1.spines['top'].set_visible(False)   # Remove top spine
ax1.spines['right'].set_visible(False) # Remove right spine



ax1.set_xlim(0, 1.75)
ax1.set_ylim(-0.1, 220)
force_line = ax1.plot(x, Forces_List, color='#DA1E28', label="Computed Force")
ni_line = ax1.plot(NIx_shifted, NIy, color=(1/255, 71/255, 133/255), label="Load Cell Force")
ax1.legend()
ax1.grid(False)
ax1.set_xlabel('Time [s]', fontsize=14)
ax1.set_ylabel('Force [N]', fontsize=14)
plt.tick_params(axis='both', which='minor', labelsize=12)
plt.legend(loc='upper left', frameon=False)  # Change 'upper right' to desired position
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'

#########################################################################################################
#########################################################################################################
# Load CSV data
Hop_Leg = "C:/Users/er122/Downloads/Hop_Torque_and_Position_Data.csv"
Hop_Load_Cell = "C:/Users/er122/Downloads/MINIHOP1026.csv"
Hop_Data = process_data(Hop_Leg, Hop_Load_Cell)


NIy = Hop_Data[3][3250:16000]*9.8
print(len(NIy))
NIx_shifted = Hop_Data[2][3250:16000] - 2.4

Forces_List = Hop_Data[1][800:4500] 
print(len(Forces_List))

x = Hop_Data[0][800:4500]  + 0.8 - 2.5

# Initialize the plot with two subplots
fig, (ax1) = plt.subplots(1, 1, figsize=(6, 5))
ax1= plt.gca()

#ax1[0,0].spines['top'].set_visible(False)   # Remove top spine
#ax1.spines['right'].set_visible(False) # Remove right spine




force_line = ax1.plot(x, Forces_List, color='#DA1E28', label="Computed Force")
ni_line = ax1.plot(NIx_shifted, NIy, color=(1/255, 71/255, 133/255), label="Load Cell Force")

ax1.spines['top'].set_visible(False)   # Remove top spine
ax1.spines['right'].set_visible(False) # Remove right spine


ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Force [N]')
ax1.set_xlim(0, 2.5)
ax1.set_ylim(-0.1, 1000)
ax1.legend()
ax1.grid(False)
ax1.set_xlabel('Time [s]', fontsize=14)
ax1.set_ylabel('Force [N]', fontsize=14)
plt.tick_params(axis='both', which='minor', labelsize=12)
plt.legend(loc='upper left', frameon=False)  # Change 'upper right' to desired position
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'

#########################################################################################################
#########################################################################################################
#########################################################################################################
# Load CSV data
SP_Leg = "C:/Users/er122/Downloads/Static_Probe_Torque_and_Position_Data.csv"
SP_Load_Cell = "C:/Users/er122/Downloads/STATIC-PROBE1026.csv"
SP_Data = process_data(SP_Leg, SP_Load_Cell)


NIy = SP_Data[3]*9.8

NIx_shifted = SP_Data[2] + 0.5

Forces_List = SP_Data[1] 

x = SP_Data[0] - 0.3 + 0.5

# Initialize the plot with two subplots
fig, (ax1) = plt.subplots(1, 1, figsize=(6, 5))
ax1= plt.gca()

ax1.spines['top'].set_visible(False)   # Remove top spine
ax1.spines['right'].set_visible(False) # Remove right spine


ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Force [N]')
ax1.set_xlim(0, 8)
ax1.set_ylim(0, 80)
force_line = ax1.plot(x, Forces_List, color='#DA1E28', label="Computed Force")
ni_line = ax1.plot(NIx_shifted, NIy, color=(1/255, 71/255, 133/255), label="Load Cell Force")
ax1.legend()
ax1.grid(False)
ax1.set_xlabel('Time [s]', fontsize=14)
ax1.set_ylabel('Force [N]', fontsize=14)
plt.tick_params(axis='both', which='minor', labelsize=12)
plt.legend(loc='upper left', frameon=False)  # Change 'upper right' to desired position
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'

#########################################################################################################
#########################################################################################################
#########################################################################################################
# Load CSV data
DP_Leg = "C:/Users/er122/Downloads/Dynamic_Probe_Torque_and_Position_Data.csv"
DP_Load_Cell = "C:/Users/er122/Downloads/DYNAMIC-PROBE1026.csv"
DP_Data = process_data(DP_Leg, DP_Load_Cell)


NIy = DP_Data[3]*9.8

NIx_shifted = DP_Data[2] - 5.2 + 0.5

Forces_List = DP_Data[1] 

x = DP_Data[0] + 0.5

# Initialize the plot with two subplots
fig, (ax1) = plt.subplots(1, 1, figsize=(6, 5))
ax1= plt.gca()

ax1.spines['top'].set_visible(False)   # Remove top spine
ax1.spines['right'].set_visible(False) # Remove right spine



ax1.set_xlim(0, 9)
ax1.set_ylim(0, 320)
force_line = ax1.plot(x, Forces_List, color='#DA1E28', label="Computed Force")
ni_line = ax1.plot(NIx_shifted, NIy, color=(1/255, 71/255, 133/255), label="Load Cell Force")
ax1.legend()
ax1.grid(False)
ax1.set_xlabel('Time [s]', fontsize=14)
ax1.set_ylabel('Force [N]', fontsize=14)
plt.tick_params(axis='both', which='minor', labelsize=12)
plt.legend(loc='upper left', frameon=False)  # Change 'upper right' to desired position
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
#########################################################################################################


plt.tight_layout()
plt.show()