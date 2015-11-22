"""
Python script to test motion algorithms with Open CV / matplotlib simulator.
"""

import numpy as np
import math
import matplotlib.pyplot as plt


plt.ion()           # interactive mode
plt.axis('equal')


# global variables
tau = 0.1           # time step
t = 0
L = math.sqrt(2)    # desired dist betw 2 bots
R = 1               # desired dist betw Bot and centroid
centroid = (3,0)
k_c = 0.08          # constant: betw centroid
k_a = 0.08          # constant: betw other bots
pos = np.array([
    (1.0, 1.0),     # Bot 0 pos
    (2.0, 3.0),     # Bot 1 pos
    (3.0, 2.0)      # Bot 2 pos
])
vel = np.zeros(pos.shape)
acc = np.zeros(pos.shape)


region = plt.Circle(centroid, 1, color='r')
plt.gcf().gca().add_artist(region)


while t < 10:

    pos_temp = np.zeros(pos.shape)

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
         
        acc[i] = (acc_xci + acc_xij, acc_yci + acc_yij)     # update accels
        vel[i] = vel[i] + acc[i] * tau 

        pos_temp[i] = pos[i] + vel[i] * tau

    # print acc
    pos = pos_temp
    
    plt.plot([p[0] for p in pos], [p[1] for p in pos], "o" )

    plt.pause(0.0001)

    t += tau

while True:
    pass