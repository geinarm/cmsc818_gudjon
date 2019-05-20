import time
import numpy as np
import matplotlib.pyplot as plt

from rrt.RRT_effector import RRT
from workspace import Workspace, Area
from frame2d import Frame2D
from arm.arm import Arm
from arm.arm import Link
from arm.obstacle import Obstacle
from arm.jacobian import Jacobian

from controller import Controller

from gui import GUI

workspace = Workspace()
limits = np.array([ 
    [-np.deg2rad(0), np.deg2rad(180)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)],
    [-np.deg2rad(150), np.deg2rad(150)]
])
l = 6.0
#arm = Arm([1.5, 1.5, 1.5, 1.2], limits)  #[2.5, 2, 1.2] 5.7
#arm = Arm([l/2, l/2], limits[0:3, :])
#arm = Arm([l/3, l/3, l/3], limits[0:4, :])
arm = Arm([l/4, l/4, l/4, l/4], limits[0:5, :])
#arm = Arm([l/5, l/5, l/5, l/5, l/5], limits[0:6, :])
#arm = Arm([l/6, l/6, l/6, l/6, l/6, l/6], limits[0:7, :])
workspace.add_arm(arm)

#arm.set_pose(np.array([np.pi/4, 0.1, np.pi/4, np.pi/4, np.pi/3]))
pose = np.array([np.pi/2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
#arm.set_pose(np.array([np.pi/4, 0, 0, 0, 0]))
#arm.set_pose(pose[0:3])
#arm.set_pose(pose[0:4])
arm.set_pose(pose[0:5])
#arm.set_pose(pose[0:6])
#arm.set_pose(pose[0:7])

workspace.add_obstacle( Obstacle(Frame2D(0, -4, 4.5), 3, 0.3) )
workspace.add_obstacle( Obstacle(Frame2D(0, -4, 3), 3, 0.3) )

workspace.add_obstacle( Obstacle(Frame2D(np.pi/4, 4, 6.5), 3, 0.3) )
workspace.add_obstacle( Obstacle(Frame2D(np.pi/4, 5, 5), 3, 0.3) )

#workspace.add_obstacle( Obstacle(Frame2D(0, -4, 5), 1, 1) )
#workspace.add_obstacle( Obstacle(Frame2D(0, 2, 1), 1, 1) )
#workspace.add_obstacle( Obstacle(Frame2D(np.pi/5, 3, 5), 2, 2) )
#workspace.add_obstacle( Obstacle(Frame2D(0, -2, 1), 1, 1) )
#workspace.create_area(-3, 6, 3, 1)

workspace.add_box(-1, 6)
workspace.add_box(4, 1)
#workspace.add_box(0, 6, theta=-np.pi/4)

controller = Controller(workspace)
gui = GUI(controller, workspace)
gui.open()