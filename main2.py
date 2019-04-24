import numpy as np

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


if __name__ == '__main__':
    pose1 = [2.5, -0.5, 0, 0]
    pose2 = [1.4, 0.2, 0.7, 0.5]

    workspace = Workspace()
    arm = Arm([2,2,1])
    arm.set_pose(pose2)

    box1 = Box(Frame2D(np.pi/4, -2, 7), 2, 1)
    box2 = Box(Frame2D(np.pi/2, -1.2, 4.2), 1, 1)
    box3 = Box(Frame2D(np.pi/5, 3.5, 4.5), 2, 2)
    box4 = Box(Frame2D(np.pi/6, -4, 1), 1, 2)

    workspace.add_arm(arm)
    workspace.add_obstacle(box1)
    workspace.add_obstacle(box2)
    workspace.add_obstacle(box3)
    workspace.add_obstacle(box4)

    limits = np.array([ [0, np.pi],[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi]] )
    rrt = RRT(workspace, 0.2, 0.05, limits)
    path = rrt.findPath(np.array(pose1), np.array(pose2), maxNodes=100)
