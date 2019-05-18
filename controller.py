
from planner.domain import Pick, Place
from planner.logic import State
from planner import Planner
from planner.domain import PickAndPlaceDomain

class Controller(object):
    def __init__(self, workspace, gui):
        self.workspace = workspace
        self.domain = PickAndPlaceDomain(workspace)
        self.gui = gui

        self.path = None
        self.path_index = 0

    def find_plan(self, goal):
        planner = Planner()
        plan = planner.find_plan(self._current_state(), goal)
        
        return plan

    def follow_path(self, path):
        self.path = path
        self.path_index = 0

    def execute_plan(self, plan):
        for action in plan:
            self.perform_action(action)

    def perform_action(self, action):
        if isinstance(action, Pick):
            box = action.box
            ## get box pose

            ## RRT find path

            ## Move arm
        elif isinstance(action, Place):
            ## Find goal pose

            ## RRT find path

            ## Move arm
        else:
            raise Exception(F"Unknown action: {action}")

    def on_target_reached(self):
        self.path = None

    def main_loop(self):
        while True:
            if self.path is not None:
                q = self.path[self.path_index, :]
                self.workspace.arm.set_pose(q)
                self.path_index += 1
                if self.path_index >= len(self.path):
                    self.on_target_reached()

            gui.update()

    def _current_state(self):
        return State()