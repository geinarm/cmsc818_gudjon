import numpy as np
import matplotlib.pyplot as plt
from arm.obstacle import Obstacle
from arm.goal import GoalEndPose, GoalCollection
from frame2d import Frame2D

class Box(object):

    BOX_SIZE = 0.4

    def __init__(self, x, y, theta=0, name="box"):
        self.obstacle = Obstacle(Frame2D(theta, x, y), Box.BOX_SIZE, Box.BOX_SIZE)
        self.frame = self.obstacle.frame
        
        ## Names are used to map between workspace and planning domain
        self.name = name

    @DeprecationWarning
    def get_grasp_poses(self):
        return self.get_grasp_goal()

    def get_grasp_goal(self):
        goals = []
        p1 = self.obstacle.frame.transform_points(np.array([0, Box.BOX_SIZE/2]))
        theta1 = -np.pi/2 + self.obstacle.frame.theta()
        goals.append(GoalEndPose(np.array([p1[0,0], p1[0,1],  theta1]), 0.1))
        
        p2 = self.obstacle.frame.transform_points(np.array([0, -Box.BOX_SIZE/2]))
        theta2 = np.pi/2 + self.obstacle.frame.theta()
        goals.append(GoalEndPose(np.array([p2[0,0], p2[0,1], theta2]), 0.1))

        p3 = self.obstacle.frame.transform_points(np.array([Box.BOX_SIZE/2, 0]))
        theta3 = np.pi + self.obstacle.frame.theta()
        goals.append(GoalEndPose(np.array([p3[0,0], p3[0,1], theta3]), 0.1))
        
        p4 = self.obstacle.frame.transform_points(np.array([-Box.BOX_SIZE/2, 0]))
        theta4 = self.obstacle.frame.theta()
        goals.append(GoalEndPose(np.array([p4[0,0], p4[0,1], theta4]), 0.1))
        
        return GoalCollection(goals)

    def get_collider(self):
        return self.obstacle.get_collider()

    def get_position(self):
        return self.obstacle.frame.origin()

    def draw(self, ax, color='blue', clear=True):
        self.obstacle.draw(ax, color=color, clear=clear)

    def draw_grasp_poses(ax):
        grasp_poses = self.get_grasp_poses()
        for pose in grasp_poses:
            self.ax.plot([origin[0], pose[0]], [origin[1], pose[1]])
            self.ax.scatter(pose[0], pose[1])

class Area(object):
    def __init__(self, x, y, width, height, name):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.obstacle = Obstacle(Frame2D(0, x, y), width, height)
        self.frame = self.obstacle.frame
        self.name = name

    def get_collider(self):
        return self.obstacle.get_collider()

    def get_bounding_box(self):
        return [
            self.x - self.width/2,
            self.x + self.width/2,
            self.y - self.height/2,
            self.y + self.height/2
            ]

    def draw(self, ax, color='#22CC22', clear=True):
        self.obstacle.draw(ax, color=color, clear=clear)

class Workspace(object):

    WIDTH = 10
    HEIGHT = 10

    def __init__(self):
        self.boxes = []
        self.obstacles = []
        self.areas = []
        self.arm = None
        self.arm_holding = None

    def add_box(self, x, y, theta=0, name=None):
        if name is None:
            name = "box{0}".format(len(self.boxes))

        b = Box(x, y, theta, name)
        self.boxes.append(b)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def add_area(self, area, name=None):
        if name is None:
            name = "area{0}".format(len(self.areas))

        self.areas.append(area)

    def create_area(self, x, y, width, height):
        name = "area{0}".format(len(self.areas))
        area = Area(x, y, width, height, name)
        self.areas.append(area)
        return area

    def add_arm(self, arm):
        self.arm = arm

    def in_collision(self):
        if self.arm is not None:
            for o in self.obstacles:
                if self.arm.collides(o.get_collider()):
                    return True
            for o in self.boxes:
                if self.arm.collides(o.get_collider()):
                    return True

        return False

    def box_at(self, x, y):
        for b in self.boxes:
            if b.obstacle.point_collides(x, y):
                return b

        return None

    def area_at(self, x, y):
        for a in self.areas:
            if a.obstacle.point_collides(x, y):
                return a

        return None

    def arm_grab(self, box):
        self.boxes.remove(box)
        self.arm.grab(box)

    def arm_drop(self):
        box = self.arm.drop()
        if box is not None:
            self.boxes.append(box)

    def draw(self, ax=None, t=0):
        if ax is None:
            fig, ax = plt.subplots(figsize=(10,6))
            #fig.canvas.mpl_connect('close_event', handle_close)
            ax.set_aspect('equal')
            ax.set_xlim(-5, 5)
            ax.set_ylim(0, 8)

        for a in self.areas:
            a.draw(ax)

        for o in self.obstacles:
            o.draw(ax, color='#FF5555')

        for b in self.boxes:
            b.draw(ax, color='blue')

        if self.arm is not None:
            self.arm.draw(ax)

        if ax is None:
            plt.show()

        if t > 0:
            plt.pause(t)

    def draw_frame(self, ax, dt):
        if self.arm is not None:
            self.arm.draw(ax)

        for o in self.obstacles:
            o.draw(ax)

        for b in self.boxes:
            b.draw(ax)
