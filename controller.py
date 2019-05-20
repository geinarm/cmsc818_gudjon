import threading
import time
import numpy as np

from rrt.RRT_effector import RRT
from arm.goal import GoalArea

from planner.domain import PickAndPlaceDomain, Pick, Place, In, Holding, Reachable
from planner.logic import State, Predicate
from planner.planner import Planner
from workspace import Workspace, Area, Box

class Controller(object):

    EVENT_PATH_FOUND = "EVENT_PATH_FOUND"
    EVENT_SEARCH_FAILED = "EVENT_SEARCH_FAILED"
    EVENT_GOAL_REACHED = "EVENT_GOAL_REACHED"
    EVENT_POSE_CHANGED = "EVENT_POSE_CHANGED"
    EVENT_GOAL_CHANGED = "EVENT_GOAL_CHANGED"

    MODE_IDLE = "MODE_IDLE"
    MODE_RUN_PLAN = "MODE_RUN_PLAN"
    MODE_RUN_ACTION = "MODE_RUN_ACTION"
    MODE_RUN_RRT = "MODE_RUN_RRT"

    def __init__(self, workspace):
        self.workspace = workspace
        self.domain = PickAndPlaceDomain(workspace)
        self.rrt = RRT(workspace, 0.1, 0.1)

        self.callback_map = {}

        self._mode = Controller.MODE_IDLE
        self.goal_state = None

    def register_callback(self, event, callback):
        if event not in self.callback_map:
            self.callback_map[event] = [callback]
        else:
            self.callback_map[event].append(callback)
        
    def add_move_box_goal(self, box, point):
        """ Add a goal to move box to area """

        #ps_area = self.domain.add_area(area) ## assuming the area does not exist
        new_area = self.workspace.create_area(point[0], point[1], Box.BOX_SIZE, Box.BOX_SIZE)
        ps_area = self.domain.add_area(new_area, new_area.name)
        ps_box = self.domain.get_object_by_name(box.name)

        ## Stated goal: the box is in the area
        self.add_goal(In(ps_box, ps_area, True))


    def add_goal(self, predicate):
        if self.goal_state is None:
            self.goal_state = State()

        self.goal_state.set(predicate)

        self._call_callbacks(Controller.EVENT_GOAL_CHANGED, goal=self.goal_state)

    def find_plan(self):
        if self.goal_state is None:
            return

        planner = Planner(self.domain)
        plan = planner.find_plan(self._current_state(), self.goal_state)
        
        print(plan)
        return plan

    def start_plan(self, plan):
        self._mode = Controller.MODE_RUN_PLAN
        ## Run the plan on a thread
        plan_thread = threading.Thread(target=self._execute_plan, args=(plan,))
        plan_thread.start()
        print('Plan thread started')

    def _execute_plan(self, plan):
        """ Bloking function. Run on thread """
        for action in plan:
            self._perform_action(action)
            print("Action Done")

    def _perform_action(self, action):
        """ Bloking function. Run on thread """
        if isinstance(action, Pick):
            ## Get workspace object
            box = action.box
            ws_box = self.domain.get_workspace_object(box.name)
            
            ## get box grasp pose
            goal = ws_box.get_grasp_goal()

            ## RRT find path
            path = self._run_rrt(goal)

            ## Move arm
            self._follow_path(path)

            self.workspace.arm_grab(ws_box)
            time.sleep(0.5)

        elif isinstance(action, Place):
            box = action.box
            area = action.area
            ## Get workspace object
            ws_box = self.domain.get_workspace_object(box.name)
            ws_area = self.domain.get_workspace_object(area.name)

            ## Find goal pose
            goal = GoalArea(ws_area.get_bounding_box())

            ## RRT find path
            path = self._run_rrt(goal)

            ## Move arm
            self._follow_path(path)

            self.workspace.arm_drop()
            time.sleep(0.5)
        else:
            raise Exception(F"Unknown action: {action}")

    def start_rrt(self, goal):
        self._mode = Controller.MODE_RUN_RRT
        ## Run the search on a thread so we can visualize the process
        rrt_thread = threading.Thread(target=self._run_rrt, args=(goal,))
        rrt_thread.start()
        print('RRT thread started')

    def _call_callbacks(self, event, **kwarg):
        if event not in self.callback_map:
            return

        callbacks = self.callback_map[event]
        if callbacks is not None:
            for callback in callbacks:
                callback() ## Ignoring arguments for now

    def _on_path_found(self, path):
        self._call_callbacks(Controller.EVENT_PATH_FOUND, path=path)
        if self._mode == Controller.MODE_RUN_RRT:
            path_thread = threading.Thread(target=self._follow_path, args=(path,))
            path_thread.start()
            print('Path thread started')

    def _on_search_failed(self):
        self._call_callbacks(Controller.EVENT_SEARCH_FAILED)

    def _on_target_reached(self):
        self._call_callbacks(Controller.EVENT_GOAL_REACHED)

    def _current_state(self):
        robot = self.domain.robot

        state = State()
        state.set(Holding(robot, None))
        for box in self.workspace.boxes:
            ps_box = self.domain.get_object_by_name(box.name)
            state.set(Reachable(robot, ps_box))

        for area in self.workspace.areas:
            ps_area = self.domain[area.name]
            state.set(Reachable(robot, ps_area))

        return state

    def _run_rrt(self, goal):
        """ Bloking function. Run on thread """
        pose_start = self.workspace.arm.get_pose()
        path = self.rrt.find_path(pose_start, goal, max_samples=2000)
        if path is None:
            self._on_search_failed()
            return

        path = np.array(path)
        path_index = 0
        self._on_path_found(path)
        return path

    def _follow_path(self, path, delay=0.1):
        """ Bloking function. Run on thread """
        path_index = 0
        while True:
            q = path[path_index, :]
            self.workspace.arm.set_pose(q)
            self._call_callbacks(Controller.EVENT_POSE_CHANGED, pose=q)
            path_index += 1
            if path_index >= len(path):
                break

            time.sleep(delay)

        self._on_target_reached()