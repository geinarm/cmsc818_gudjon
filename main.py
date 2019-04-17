import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

from arm.arm import Arm
from arm.arm import Link

def handle_close(e):
    global running
    print("close")
    plt.close()
    running = False

arm = Arm([2,3,2,1])

fig, ax = plt.subplots(figsize=(10,6))
fig.canvas.mpl_connect('close_event', handle_close)

running = True
#while(running):
now = time.time()
ax.clear()

ax.set_xlim(-5, 5)
ax.set_ylim(0, 6)
P = np.array([3+np.cos(now), 2+np.sin(now)]) ## The target point

#arm.move_to(P)
arm.set_pose([0,np.pi/4, np.pi/4, np.pi/4, 0])
arm.plot_arm(ax)
#ax.scatter(P[0], P[1], c='r', s=120)

#plt.pause(0.1)

plt.show()