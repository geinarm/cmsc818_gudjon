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
ax.set_aspect('equal')
ax.set_xlim(-5, 5)
ax.set_ylim(0, 8)
#fig.canvas.mpl_connect('close_event', handle_close)

pose1 = [2.5, -0.5, 0, 0]
pose2 = [1.2, 0.2, 0.7, 0.5]

workspace = Workspace()
arm = Arm([2,2,1])

obstacle1 = Box(Frame2D(np.pi/4, -2, 7), 2, 1)
obstacle2 = Box(Frame2D(np.pi/2, -1.2, 4.2), 1, 1)
obstacle3 = Box(Frame2D(np.pi/5, 3.5, 4.5), 2, 2)
obstacle4 = Box(Frame2D(np.pi/6, -4, 1), 1, 2)

box1 = Box(Frame2D(0, -4, 3), 0.5, 0.5)
box2 = Box(Frame2D(0, 4, 2), 0.5, 0.5)

workspace.add_arm(arm)
workspace.add_obstacle(obstacle1)
workspace.add_obstacle(obstacle2)
workspace.add_obstacle(obstacle3)
workspace.add_obstacle(obstacle4)
workspace.add_box(box1)
workspace.add_box(box2)

arm.set_pose(pose2)
workspace.draw(ax)

limits = np.array([ [0, np.pi],[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi]] )
rrt = RRT(workspace, 0.25, 0.2, limits)
t_start = time.time()
path = rrt.findPath(np.array(pose1), np.array(pose2), maxNodes=1000)
#path = None
print(time.time() - t_start)

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
else:
    arm.set_pose(pose1)
    workspace.draw(ax)

plt.show()