import matplotlib.pyplot as plt

class Workspace(object):
    def __init__(self):
        self.boxes = []
        self.obstacles = []
        self.arm = None

    def add_box(self, box):
        self.boxes.append(box)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def add_arm(self, arm):
        self.arm = arm

    def in_collision(self):
        if self.arm is not None:
            for o in self.obstacles:
                if self.arm.collides(o.get_collider()):
                    return True

        return False

    def draw(self, ax=None, t=0):
        if ax is None:
            fig, ax = plt.subplots(figsize=(10,6))
            #fig.canvas.mpl_connect('close_event', handle_close)
            ax.set_aspect('equal')
            ax.set_xlim(-5, 5)
            ax.set_ylim(0, 8)

        if self.arm is not None:
            self.arm.draw(ax)

        for o in self.obstacles:
            o.draw(ax, color='#FF5555')

        for b in self.boxes:
            b.draw(ax, color='blue')

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
