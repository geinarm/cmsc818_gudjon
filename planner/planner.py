
class Node(object):
    def __init__(self, state, action, parent=None):
        self.state = state
        self.action = action
        self.parent = parent

    def plan(self):
        n = self
		actions = []
		while n.parent != None:
			actions.append(n.action)
			n = n.parent

		actions.reverse()
		return actions

class Planner(object):
    def __init__(self):
        pass

    def find_plan(self, state_start, state_goal):
        pass