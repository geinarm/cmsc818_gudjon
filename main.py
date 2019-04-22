import time
import numpy as np
import matplotlib.pyplot as plt

from RRT import RRT
from workspace import Workspace
from frame2d import Frame2D
from arm.arm import Arm
from arm.arm import Link
from arm.obstacle import Box

def handle_close(e):
    global running
    print("close")
    plt.close()
    running = False

fig, ax = plt.subplots(figsize=(10,6))
#fig.canvas.mpl_connect('close_event', handle_close)

pose1 = [2.5, -0.5, 0]
#pose1 = [1.3, 0.4, 0, 0.2]
pose2 = [1.2, 0.0, 0]

workspace = Workspace()
#arm = Arm([3,2,1])
arm = Arm([2,2])
arm.set_pose(pose1)

box1 = Box(Frame2D(np.pi/4, -2,7), 2, 1)
box2 = Box(Frame2D(np.pi/2, -1.5,5), 1, 1)
box3 = Box(Frame2D(np.pi/5, 3,6), 2, 2)
box4 = Box(Frame2D(np.pi/6, -5,2), 1, 2)

workspace.add_arm(arm)
workspace.add_obstacle(box1)
workspace.add_obstacle(box2)
workspace.add_obstacle(box3)
workspace.add_obstacle(box4)

arm.set_pose(pose2)
workspace.draw(ax)

#limits = np.array([ [0, np.pi],[-np.pi, np.pi],[-np.pi, np.pi],[-np.pi, np.pi] ])
limits = np.array([ [0, np.pi],[-np.pi, np.pi], [0,0]] )
rrt = RRT(workspace, 0.2, 0.05, limits)
#rrt = RRT(workspace, 0.2, 0.05, np.array([ [-5, 5], [0, 8] ]))
path = rrt.findPath(np.array(pose1), np.array(pose2), ax=ax, maxNodes=200)
#path = rrt.findPath(np.array([1,1]), np.array([4, 7]), ax=ax)

if path is not None:
    for p in path:
        ax.clear()
        ax.set_aspect('equal')
        ax.set_xlim(-5, 5)
        ax.set_ylim(0, 8)

        arm.set_pose(p)
        workspace.draw(ax)
        plt.pause(0.2)
    plt.show()

plt.show()
print(workspace.in_collision())