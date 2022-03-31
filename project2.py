#-------------------------------------------------------------------------------
# FILE NAME:      project2.py
# DESCRIPTION:    uses python3 to do ,otion path planning
# USAGE:          python3 project2.py
#                 
# notes:          Converted Matlab to Python
#                 
#
# MODIFICATION HISTORY
# Author               Date           version
#-------------------  ------------    ---------------------------------------
# Annette McDonough   2022-03-21      1.0 first version 
# Annette McDonough   2022-03-22      1.1 still converting
# Annette McDonough   2022-03-23      1.2 still working on conversion
# Annette McDonough   2022-03-25      1.3 implemnting equations
# Annette McDonough   2022-03-26      1.4 initialization works
# Annette McDonough   2022-03-27      1.5 working on plots
# Annette McDonough   2022-03-28      1.6 working on bugs
# Annette McDonough   2022-03-29      1.7 cleaning up code
#-----------------------------------------------------------------------------

import numpy as np
import math
import matplotlib.pyplot as plt 
from random import random 


# ========Set parameters for simulation============
# number of dimensions
n=2
# set time step
delta_t = .05
# set total simulation time
t = np.arange(0, 10, delta_t)
# set scaling factor of attractive potential field
lambdaX = 8.5
# set maximum of robot velocity
vr_max = 50
theta_t = np.zeros([len(t),1])
qt_diff = np.zeros([len(t),n])
error = np.zeros([len(t), 1])
prv = np.zeros([len(t), n], float)
phi = np.zeros([len(t), 1])

# ==================Set VIRTUAL TARGET=================
# Initial positions of virtual target
qv = np.zeros([len(t),n], dtype = float)
# Set velocity of virtual target
pv = 1.2
# Initial heading of the virtual target

# ===========Set ROBOT =================
# Set initial state of robot (robot)
# initial position of robot
qr = np.zeros([len(t), n], dtype = float)
# Initial velocity of robot
v_rd = np.zeros([len(t), 1])
# Initial heading of the robot
theta_r = np.zeros([len(t), 1])

#===Set relative states between robot and VIRTUAL TARGET===
# Save relative positions between robot and virtual target
qrv = np.zeros([len(t), n], dtype = float)
# Save relative velocities between robot and virtual target
prv = np.zeros([len(t), n], dtype = float)

#==Compute initial relative states between robot and virtual target==
# Compute the initial relative position
qrv[0,:] = qv[0,:]-qr[0,:]
# Compute the initial relative velocity
prv[0] = [pv*np.cos(theta_t[0])-v_rd[0]*np.cos(theta_r[0]), 
       pv*np.sin(theta_t[0])-v_rd[0]*np.sin(theta_r[0]),]

# ====Set noise mean and standard deviation====
noise_mean = .5
#noise_std = 0.5 #try 0.2 also
noise_std = .2

for i in range(1, len(t)):
    #++++++++++CIRCULAR TRAJECTORY+++++++++++
    #Set target trajectory moving in CIRCULAR trajectory WITHOUT noise
    #qv_x = 60 - 15*np.cos(t[i])
    #qv_y = 30 + 15*np.sin(t[i])
    # Compute position of virtual target
    #qv[i,:] = [qv_x, qv_y]

    # circular with noise
    #qv_x = 60 - 15*np.cos(t[i])+ noise_std * random() + noise_mean
    #qv_y = 30 + 15*np.sin(t[i]) + noise_std * random() + noise_mean
    # Compute position of target
    #qv[i,:] = [qv_x, qv_y]

    #++++++++++SineWave Trajectory++++++++++++++
    #Set target trajectory moving in SineWave trajectory WITHOUT noise
    #qv_x = 60 + 5 * t[i]
    #qv_y = 30 + 15 * np.sin(t[i])
    # Compute position of virtual target
    #qv[i,:] = [qv_x, qv_y] 

    # SineWave with Noise
    qv_x = 60 + 5 * t[i] + noise_std * random() + noise_mean
    qv_y = 30 + 15 * np.sin(t[i]) + noise_std * random() + noise_mean
    # Compute position of virtual target
    qv[i,:] = [qv_x, qv_y]


    #+++++++Linear Trajectory+++++++++++++++
    #Set target trajectory moving in Linear trajectory WITHOUT noise
    #qv_x = 15 + 5 * t[i]
    #qv_y = 30 + qv_x
    # Compute position of virtual target
    #qv[i,:] = [qv_x, qv_y]

    # Linear with noise
    #qv_x = 15 - 5 * t[i] + noise_std * random() + noise_mean
    #qv_y = 30 + qv_x  + noise_std * random() + noise_mean
    # Compute position of virtual target
    #qv[i,:] = [qv_x, qv_y]

    # Compute the target heading
    qt_diff[i,:] =  qv[i,:]-qv[i-1,:]
    theta_t[i] = np.arctan2(qt_diff[i,1],qt_diff[i,0])

    # Equations here
    phi[i] = np.arctan2(qrv[i-1,1], qrv[i-1,0])
    v_rd[i] = np.sqrt(np.square(np.linalg.norm(pv))+
              2*lambdaX*np.linalg.norm(qrv[i-1])*np.linalg.norm(pv)*
              np.absolute(np.cos(theta_t[i] - phi[i]))+
              np.square(lambdaX)*np.square(np.linalg.norm(qrv[i-1])))
    
    v_rd[i] = np.minimum(v_rd[i],vr_max)
    theta_r[i] = phi[i]+np.arcsin((np.linalg.norm(pv)*np.sin(theta_t[i]-phi[i]))/v_rd[i])

    #======UPDATE position and velocity of robot===========  
    qr[i] = np.array(qr[i-1]) + np.array(v_rd[i]*delta_t*[np.cos(theta_r[i-1]),
              np.sin(theta_r[i-1])]).reshape(2)

    qrv[i] = qv[i] - qr[i]

    prv[i] = [pv*np.cos(theta_t[i])-v_rd[i]*np.cos(theta_r[i]),
              pv*np.sin(theta_t[i])-v_rd[i]*np.sin(theta_r[i])]

    error[i] = np.linalg.norm(qv[i,:]-qr[i,:]) 

    plt.grid()
    plt.title("Target Trajectory with noise\n max velocity=50 and stand_dev=.2")
    target = plt.scatter(qv[:, 0], qv[:, 1], s=1, color="red", label="Target")
    robot = plt.scatter(qr[:, 0], qr[:, 1], s=1, color="blue", label="Robot")
    plt.pause(.01)
    plt.legend(handles = [target, robot])
    plt.xlabel("X(m)")
    plt.ylabel("Y(m)")
plt.savefig("sinNoiseF.jpg", dpi=750)
plt.show()

# Plot error graph
plt.grid()
plt.title("Distance Error Between Robot and Virtual Target\n with noise\n max velocity=50 and stand_dev=.2")
plt.plot(error[range(1,len(t))], '', color="blue", label="Error")
plt.xlabel("Iterations")
plt.ylabel("Error Margin")
plt.legend()
plt.savefig("sinNoiseF_1.jpg", dpi=750)
plt.show()

plt.grid()
plt.title("Distance Error Between Robot and Virtual Target\n with noise\n max velocity=50 and stand_dev=.2")
plt.scatter(range(len(t)), error, s=1, color="blue", label="Error")
plt.xlabel("Iterations")
plt.ylabel("Error Margin")
plt.legend()
plt.savefig("sinNoiseF_2.jpg", dpi=750)
plt.show()

# Plot robot velocity
plt.grid()
plt.title("Robot Velocity with noise\n max velocity=50 and stand_dev=.2")
plt.plot(range(len(t)), v_rd, color="blue", label="Robot Velocity")
plt.xlabel("Iterations")
plt.ylabel("Robot Velocity")
plt.legend()
plt.savefig("sinNoiseF_3.jpg", dpi=750)
plt.show()

# plot track a target
plt.grid()
plt.title("Track a Target Moving with noise\n max velocity=50 and stand_dev=.2")
plt.scatter(range(len(t)), theta_t, s=1, color="blue", label="Target Orientation")
plt.scatter(range(len(t)), theta_r, s=1, color="red", label="Robot Orientation")
plt.scatter(range(len(t)), phi, s=1, color="green", label="Relative Orientation")
plt.xlabel("Iterations")
plt.ylabel("Raidians")
plt.legend()
plt.savefig("sinNoiseF_4.jpg", dpi=750)
plt.show()




