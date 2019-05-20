from queue import SimpleQueue

class Node(object):
	def __init__(self, state, action, parent=None, cost=1):
		self.state = state
		self.action = action
		self.parent = parent
		if parent is not None:
			self.cost = parent.cost + cost
		else:
			self.cost = cost

	def plan(self):
		n = self
		actions = []
		while n.parent != None:
			actions.append(n.action)
			n = n.parent

		actions.reverse()
		return actions

class Planner(object):
	def __init__(self, domain):
		self.domain = domain

	def find_plan(self, state_start, state_goal):
		frontier = SimpleQueue()
		frontier.put(Node(state_start, None, None))
		count = 0

		while(not frontier.empty()):
			node = frontier.get_nowait()
			count += 1
			if count % 100 == 0:
				print(count)

			#print("------------------------")
			#print("Action: " + str(node.action))
			#print("Current state: " + str(node.state))
			#print("------------------------")
			#print("Availabe actions:")

			if(node.state.contains(state_goal)):
				print('Found goal', count)
				return node.plan()

			actions = self.domain.get_applicable_actions(node.state)
			#print( ",\n".join(['('+str(a)+')' for a in actions]) )
			for action in actions:
				newState = action.apply(node.state)
				newNode = Node(newState, action, node)
				frontier.put(newNode)

			#print("------------------------")