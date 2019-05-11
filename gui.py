import time
import threading
import numpy as np
import matplotlib.pyplot as plt

from rrt.RRT_effector import RRT

class GUI(object):

    STATE_IDLE = 'STATE_IDLE'
    STATE_SEARCHING = 'STATE_SEARCHING'
    STATE_MOVING = 'STATE_MOVING'
    STATE_SEARCH_FAILED = 'STATE_SEARCH_FAILED'

    def __init__(self, workspace, rrt):
        self.workspace = workspace
        self.fig, self.ax = plt.subplots()
        self._open = False
        self._state = GUI.STATE_IDLE
        self.rrt = rrt
        self.rrt.set_node_added_callback(self.on_node_added)
        self.rrt.set_node_tested_callback(self.on_node_tested)
        self.path = None
        self.target_box = None
        
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_down)
        self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_up)
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        self.ax_artists = []

        self.reset()

    def on_node_added(self, node):
        if node.parent is not None:
            a = node.parent.end_point
            b = node.end_point

            artist, = self.ax.plot([a[0], b[0]], [a[1], b[1]])
            self.ax_artists.append(artist)
            artist = self.ax.scatter(b[0], b[1], c='black', s=10)
            self.ax_artists.append(artist)

    def on_node_tested(self, node):
        pass

    def clear_tree(self):
        for a in self.ax_artists:
            a.remove()
        self.ax_artists = []

    def reset(self):
        self.ax.clear()
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(0, 8)
        self.ax.set_aspect('equal')
        self._state = GUI.STATE_IDLE

    def start_rrt(self):
        ## Run the search on a thread so we can visualize the process
        self._state = GUI.STATE_SEARCHING
        rrt_thread = threading.Thread(target=self._run_rrt)
        rrt_thread.start()
        print('thread started')

    def _run_rrt(self):
        pose_start = self.workspace.arm.get_pose()
        path = self.rrt.find_path(pose_start, self.goals, max_samples=2000)
        if path is None:
            self.on_search_failed()
            return

        self.path = np.array(path)
        self.path_index = 0
        self._state = GUI.STATE_MOVING

    def on_mouse_down(self, event):
        point = np.array([event.xdata, event.ydata])

        if event.button == 1: # Left Click
            if self._state == GUI.STATE_IDLE:
                self.mouse_start = point

            elif self._state == GUI.STATE_MOVING:
                pass

            elif self._state == GUI.STATE_SEARCHING:
                self.rrt.interrupt()
                self.clear_tree()

    def on_mouse_up(self, event):
        mouse_end = np.array([event.xdata, event.ydata])
        if event.button == 1: # Left Click
            if self._state == GUI.STATE_IDLE:
                box_selected = self.workspace.box_at(mouse_end[0], mouse_end[1])
                if box_selected is not None:
                    self.goals = []
                    grasp_poses = box_selected.get_grasp_poses()
                    for pose in grasp_poses:
                        self.goals.append(np.array(pose))

                    self.target_box = box_selected
                    self.start_rrt()
                else:
                    ## Oriented goal pose
                    v = mouse_end - self.mouse_start
                    goal_x = self.mouse_start[0]
                    goal_y = self.mouse_start[1]
                    goal_theta = np.arctan2(v[1], v[0])
                    self.goals = [np.array([goal_x, goal_y, goal_theta])]

                    self.start_rrt()

    def on_target_reached(self):
        print("Target Reached")
        self.path = None
        self._state = GUI.STATE_IDLE
        self.clear_tree()

        if self.target_box is not None:
            self.workspace.arm_grab(self.target_box)
            self.target_box = None
        else:
            self.workspace.arm_drop()

    def on_search_failed(self):
        print("Failed")
        self._state = GUI.STATE_SEARCH_FAILED
        self.target_box = None

    def on_close(self, event):
        if self._state == GUI.STATE_SEARCHING:
            self.rrt.interrupt()

        self._open = False

    def open(self):
        self.workspace.draw(self.ax)
        self._open = True
        while self._open:

            if self.path is not None:
                q = self.path[self.path_index, :]
                self.workspace.arm.set_pose(q)
                self.workspace.draw(self.ax)

                self.path_index += 1
                if self.path_index >= len(self.path):
                    self.on_target_reached()

            if self._state == GUI.STATE_SEARCH_FAILED:
                self.clear_tree()
                self._state = GUI.STATE_IDLE

            plt.pause(0.1)