import shapely
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

class Obstacle(object):
    def __init__(self, frame, width, height):
        self.frame = frame
        self.width = width
        self.height = height

        self.shape = np.array([ [-0.5, 0.5],[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5] ])
        self.shape *= np.array([width, height])
        self.ax_artists = []

    def get_position(self):
        return self.frame.origin()

    def get_points(self):
        return self.frame.transform_points(self.shape)

    def get_collider(self):
        return shapely.geometry.Polygon(self.get_points())

    def point_collides(self, x, y):
        point = shapely.geometry.Point(x, y)
        collider = self.get_collider()
        return point.intersects(collider)

    def draw(self, ax, color='red', clear=True):
        if clear:
            for a in self.ax_artists:
                a.remove()
            self.ax_artists = []

        poly = Polygon(self.get_points(), True)
        p = PatchCollection([poly], alpha=1.0, facecolors=color)
        #p.set_array(np.array(colors))
        #ax.add_collection(p)
        self.ax_artists.append(ax.add_collection(p))