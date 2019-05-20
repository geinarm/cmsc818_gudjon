import time
import threading
import numpy as np
import matplotlib.pyplot as plt

from rrt.RRT_effector import RRT
from arm.goal import GoalEndPose, GoalArea

from workspace import Workspace, Area
from controller import Controller

class GUI(object):

    STATE_IDLE = 'STATE_IDLE'
    STATE_SEARCHING = 'STATE_SEARCHING'
    STATE_MOVING = 'STATE_MOVING'
    STATE_SEARCH_FAILED = 'STATE_SEARCH_FAILED'
    STATE_DRAG_BOX = 'STATE_DRAG_BOX'

    def __init__(self, controller, workspace):
        self.controller = controller
        self.workspace = workspace
        
        self._state = GUI.STATE_IDLE
        self._open = False
        self.fig, self.ax = plt.subplots()

        self.controller.rrt.set_node_added_callback(self.on_node_added)
        self.controller.rrt.set_node_tested_callback(self.on_node_tested)
        self.controller.register_callback(Controller.EVENT_POSE_CHANGED, self.on_arm_moved)
        self.controller.register_callback(Controller.EVENT_GOAL_REACHED, self.on_target_reached)
        self.controller.register_callback(Controller.EVENT_PATH_FOUND, self.on_path_found)
        self.controller.register_callback(Controller.EVENT_SEARCH_FAILED, self.on_search_failed)
        self.controller.register_callback(Controller.EVENT_GOAL_CHANGED, self.on_goal_changed)

        self.target_box = None
        
        ## Subscribe to GUI window events
        self.fig.canvas.mpl_connect("button_press_event", self.on_mouse_down)
        self.fig.canvas.mpl_connect("button_release_event", self.on_mouse_up)
        self.fig.canvas.mpl_connect("close_event", self.on_close)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_mouse_move)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key_down)

        self.draw_flag = True
        self.ax_artists = []
        self.drag_line_artist = None
        self.goal_text_artist = None

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

    def on_mouse_down(self, event):
        point = np.array([event.xdata, event.ydata])

        if event.button == 1: # Left Click
            self.mouse_start = point
            if self._state == GUI.STATE_IDLE:
                box_selected = self.workspace.box_at(point[0], point[1])
                if box_selected is not None:
                    self._state = GUI.STATE_DRAG_BOX
                    self.target_box = box_selected

            elif self._state == GUI.STATE_MOVING:
                pass

            elif self._state == GUI.STATE_SEARCHING:
                self.rrt.interrupt()
                self.clear_tree()

    def on_mouse_move(self, event):
        point = np.array([event.xdata, event.ydata])

        if self._state == GUI.STATE_DRAG_BOX:
            line_xs = [self.mouse_start[0], point[0]]
            line_ys = [self.mouse_start[1], point[1]]
            if self.drag_line_artist is not None:
                self.drag_line_artist.remove()
            self.drag_line_artist = self.ax.plot(line_xs, line_ys, c="#00ff00")[0]

    def on_mouse_up(self, event):
        mouse_end = np.array([event.xdata, event.ydata])
        if event.button == 1: # Left Click
            if self._state == GUI.STATE_IDLE:
                box_selected = self.workspace.box_at(mouse_end[0], mouse_end[1])
                area_selected = self.workspace.area_at(mouse_end[0], mouse_end[1])
                if box_selected is not None:
                    self.goal = box_selected.get_grasp_goal()
                    self.target_box = box_selected
                    self.start_rrt()
                elif area_selected is not None:
                    self.goal = GoalArea(area_selected.get_bounding_box(), -0.2)
                    self.start_rrt()
                else:
                    ## Oriented goal pose
                    v = mouse_end - self.mouse_start
                    goal_x = self.mouse_start[0]
                    goal_y = self.mouse_start[1]
                    if np.linalg.norm(v) < 0.1:
                        self.goal = GoalArea(np.array([goal_x-0.1, goal_x+0.1, goal_y-0.1, goal_y+0.1]))
                    else:
                        goal_theta = np.arctan2(v[1], v[0])
                        self.goal = GoalEndPose(np.array([goal_x, goal_y, goal_theta]), delta=0.1)
                    self.start_rrt()
            
            elif self._state == GUI.STATE_DRAG_BOX:
                v = mouse_end - self.mouse_start
                goal_x = mouse_end[0]
                goal_y = mouse_end[1]
                if np.linalg.norm(v) > 0.2:
                    self.controller.add_move_box_goal(self.target_box, np.array([goal_x, goal_y]))

                self._state = GUI.STATE_IDLE
                self.target_box = None
                if self.drag_line_artist is not None:
                    self.drag_line_artist.remove()
                    self.drag_line_artist = None


    def on_key_down(self, event):
        if event.key == "enter":
            plan = self.controller.find_plan()
            if plan is not None:
                self.controller.start_plan(plan)

    def on_path_found(self):
        self._state = GUI.STATE_MOVING

    def on_target_reached(self):
        print("Target Reached")
        self._state = GUI.STATE_IDLE
        self.clear_tree()

        #if self.target_box is not None:
        #    self.workspace.arm_grab(self.target_box)
        #    self.target_box = None
        #else:
        #    self.workspace.arm_drop()

    def on_search_failed(self):
        self.clear_tree()
        self.target_box = None
        self._state = GUI.STATE_IDLE

    def on_arm_moved(self):
        self.draw_flag = True

    def on_goal_changed(self):
        if self.goal_text_artist is not None:
            self.goal_text_artist.remove()

        goal = self.controller.goal_state
        self.goal_text_artist = self.ax.text(-5, 7.5, str(goal), fontsize=8, verticalalignment="top")
        self.draw_flag = True

    def on_close(self, event):
        if self._state == GUI.STATE_SEARCHING:
            self.rrt.interrupt()

        self._open = False

    def start_rrt(self):
        self.controller.start_rrt(self.goal)


    def open(self):
        self.workspace.draw(self.ax)
        self._open = True
        while self._open:
            if self.draw_flag:
                self.workspace.draw(self.ax)
                self.draw_flag = False
            plt.pause(0.1)
            