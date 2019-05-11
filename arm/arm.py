import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

import shapely
import shapely.geometry
from shapely.ops import cascaded_union
#from shapely.geometry import Polygon, MultiPolygon

from frame2d import Frame2D

## Polygon
claw_shape = np.array([
    [ 0.3,   0.5 ],
    [ 0.3,   0.4 ],
    [ 0.9,   0.4 ],
    [ 0.9,   0.3],
    [ 0.3,   0.3],
    [ 0.3,  -0.3],
    [ 0.9,  -0.3],
    [ 0.9,  -0.4 ],
    [ 0.3,  -0.4 ],
    [ 0.3,  -0.5 ],
    [ 0.0,  -0.5 ],
    [ 0.0,  -0.4 ],
    [-0.2,  -0.3 ],
    [-0.2,   0.3 ],
    [-0.0,   0.4 ],
    [-0.0,   0.5 ]
    ])

link_shape = np.array([
    [0, -0.5],
    [0, 0.5],
    [1, 0.5],
    [1.0, -0.5]
])

class Link(object):
    def __init__(self, shape, frame):
        self.shape = shape
        self.frame = frame

    def set_rotation(self, theta):
        self.frame.set_rotation(theta)

    def rotate(self, theta):
        self.frame = self.frame.rotate(theta)

    def get_points(self):
        return self.frame.transform_points(self.shape)

class Arm(object):

    def __init__(self, link_lengths):
        self.a1 = 3
        self.a2 = 2
        self.a3 = 0.6
        self.base_frame = Frame2D(0, 0, 0.5)
        self.links = []
        self.link_lengths = link_lengths
        self.holding = None
        self.ax_artists = []

        num_links = len(link_lengths)
        for i in range(num_links+1):
            if i == 0:
                frame = Frame2D(0,0,0, parent=self.base_frame)
            else:
                frame = Frame2D(0,link_lengths[i-1],0, parent=self.links[i-1].frame)

            if i == num_links:
                shape = claw_shape
            else:
                shape = link_shape*np.array([link_lengths[i], 0.5])
            
            self.links.append(Link(shape, frame))
        
        self.end_frame = Frame2D(0,0.6,0, parent=self.links[len(self.links)-1].frame)
        self.link_lengths.append(0.6)
        self.pose = [0]*len(self.links)

    def set_pose(self, thetas):
        assert(len(thetas) == len(self.links))
        for i in range(len(thetas)):
            self.links[i].set_rotation(thetas[i])
        self.pose = thetas

    def get_pose(self):
        return self.pose

    def grab(self, box):
        box.obstacle.frame.set_position(0,0)
        box.obstacle.frame.set_rotation(0)
        box.obstacle.frame.parent = self.end_frame
        self.holding = box

    def drop(self):
        if self.holding is not None:
            box = self.holding
            pose = self.get_end_pose()
            box.frame.set_position(pose[0], pose[1])
            box.frame.set_rotation(pose[2])
            box.frame.parent = None
            self.holding = None

            return box
        return None

    def get_end_point(self):
        return self.end_frame.origin()

    def get_end_pose(self):
        pos = self.end_frame.origin()
        theta = self.end_frame.theta()
        return np.array([pos[0], pos[1], theta])

    def get_colliders(self):
        polygons = []
        for l in self.links:
            polygons.append(shapely.geometry.Polygon(l.get_points()))

        return polygons

    def collides(self, collider):
        link_colliders = self.get_colliders()
        for l in link_colliders:
            if l.intersects(collider):
                return True

        return False

    ## Plot the arm given the segment lengths and joint angles
    def draw(self, ax, c='black', clear=True):
        if clear:
            for a in self.ax_artists:
                a.remove()
            self.ax_artists = []

        end = self.end_frame.origin()
        points = [l.frame.origin() for l in self.links]
        points = np.stack(points, axis=0)
        
        a = ax.scatter(points[:, 0], points[:, 1], s=50, c=c, zorder=10)
        self.ax_artists.append(a)
        a = ax.scatter(end[0], end[1], s=100, marker='x')
        self.ax_artists.append(a)

        polygons = []
        for link in self.links:
            poly = Polygon(link.get_points(), True)
            polygons.append(poly)

        colors = 100*np.random.rand(len(polygons))
        p = PatchCollection(polygons, alpha=0.4)
        p.set_array(np.array(colors))
        self.ax_artists.append(ax.add_collection(p))

        if self.holding is not None:
            self.holding.draw(ax, clear=clear)

