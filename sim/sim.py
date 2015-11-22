"""
Python script to test motion algorithms with Open CV / matplotlib simulator.
"""

import numpy as np
import math
import matplotlib.pyplot as plt


plt.ion()           # interactive mode
plt.axis('equal')


# global variables

pos = np.array([    # bot pos
    (1.0, 1.0),
    (2.0, 3.0),
    (3.0, 2.0),     
    (5.0, -3.0),
    (0.0, -7.0)
])
vel = np.zeros(pos.shape)
acc = np.zeros(pos.shape)

tau = 0.1           # time step
t = 0
R = 3               # desired dist betw Bot and centroid
L = R*2*math.sin(math.pi/len(pos))    # desired dist betw 2 bots
centroid = (3,0)
k_a = 0.08          # constant: betw other bots
k_b = 0.2           # constant: acct for overshoot
k_c = 0.08          # constant: betw centroid




region = plt.Circle(centroid, R, color='r')
plt.gcf().gca().add_artist(region)


# while t < 30:
while True:

    pos_temp = np.zeros(pos.shape)
    vel_temp = np.zeros(pos.shape)

    for i in range (len(pos)):

        dci = math.sqrt((pos[i][0] - centroid[0])**2 + (pos[i][1] - centroid[1])**2)    # dist betw Bot and centroid
        acc_xci = -k_c * (dci - R) * (1/dci) * (pos[i][0] - centroid[0])    # x-accel of Bot and centroid
        acc_yci = -k_c * (dci - R) * (1/dci) * (pos[i][1] - centroid[1])    # y-accel of Bot and centroid

        # define x- and y-accel betw Bot and other bots
        acc_xij = 0.0
        acc_yij = 0.0

        for j in range (len(pos)):

            dij = math.sqrt((pos[i][0] - pos[j][0])**2 + (pos[i][1] - pos[j][1])**2)    # dist betw Bot and bot j

            # calc x- and y-accel betw Bot and bot j
            if (i != j) and (dij <= L):
                acc_xij += -k_a * (dij - L) * (1/dij) * (pos[i][0] - pos[j][0])
                acc_yij += -k_a * (dij - L) * (1/dij) * (pos[i][1] - pos[j][1])
         
        acc[i] = (acc_xci + acc_xij - k_b*vel[i][0], acc_yci + acc_yij - k_b*vel[i][1])     # update accels
        vel_temp[i] = vel[i] + acc[i] * tau 

        pos_temp[i] = pos[i] + vel[i] * tau

    # print acc
    pos = pos_temp
    vel = vel_temp
    
    plt.plot([p[0] for p in pos], [p[1] for p in pos], "o" )

    plt.pause(0.0001)

    t += tau

while True:
    pass