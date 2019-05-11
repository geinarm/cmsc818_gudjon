import time
import numpy as np
import matplotlib.pyplot as plt

from rrt.RRT_effector import RRT
from workspace import Workspace
from frame2d import Frame2D
from arm.arm import Arm
from arm.arm import Link
from arm.obstacle import Obstacle
from arm.jacobian import Jacobian

from gui import GUI

pose1 = np.array([1.8, 0.5, 0, 0])
pose2 = np.array([1.5, 0.2, 0.7, 0.5])
pose3 = np.array([np.pi/4, np.pi/4, np.pi/4, np.pi/3])
goal_point = np.array([-4,1])

workspace = Workspace()
arm = Arm([2.5,2,1.2])
workspace.add_arm(arm)
arm.set_pose(pose3)

workspace.add_obstacle( Obstacle(Frame2D(0, -4, 5), 1, 1) )
workspace.add_obstacle( Obstacle(Frame2D(0, 2, 1), 1, 1) )
workspace.add_obstacle( Obstacle(Frame2D(np.pi/5, 3, 5), 2, 2) )
workspace.add_obstacle( Obstacle(Frame2D(0, -2, 1), 1, 1) )

workspace.add_box(-4, 1)
workspace.add_box(4, 1)
workspace.add_box(0, 6, theta=-np.pi/4)

limits = np.array([ 
    [-np.deg2rad(0), np.deg2rad(180)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
])

rrt = RRT(workspace, 0.1, 0.1, limits)
gui = GUI(workspace, rrt)
gui.open()