import abc
from .logic import Predicate, Object, Action, State

###
# Objects
###
class Robot(Object):
	def __init__(self, name):
		super().__init__(name, 'Robot')

class Box(Object):
	def __init__(self, name):
		super().__init__(name, 'Box')

class Area(Object):
	def __init__(self, name):
		super().__init__(name, 'Area')

###
# Predicates
###
class In(Predicate):
	def __init__(self, box, area, value=True):
		super().__init__('In', [box, area], value)

	def __str__(self):
		if self.value:
			return "{0} IN {1}".format(self.args[0], self.args[1])
		else:
			return "{0} NOT IN {1}".format(self.args[0], self.args[1])

class Holding(Predicate):
	def __init__(self, robot, value=None):
		super().__init__('Holding', [robot], value)

	def __str__(self):
		return "{0} is HOLDING {1}".format(self.args[0], self.value)

class Free(Predicate):
	def __init__(self, robot, value=True):
		super().__init__('Holding', [robot], value)

class Reachable(Predicate):
	def __init__(self, robot, pose, value=True):
		if not (isinstance(pose, Box) or isinstance(pose, Area)):
			raise Exception(F"Invalid argument type {pose}, expected Box or Area")

		super().__init__('Reachable', [robot, pose], value)


###
# Actions
###
class Pick(Action):
	def __init__(self, robot, box):
		self.robot = robot
		self.box = box
		super().__init__("Pick")

	def apply(self, state):
		if not self.applicable(state):
			raise Exception('Action is not applicable in this state')

		new_state = state.copy()
		new_state.remove(Holding(self.robot, None))
		new_state.remove(Reachable(self.robot, self.box))
		new_state.set(Holding(self.robot, self.box))
		return new_state

	def applicable(self, state):
		return state.check(Holding(self.robot, None)) and\
			state.check(Reachable(self.robot, self.box, True))

	def __str__(self):
		return "{0} Pick {1}".format(self.robot, self.box)

class Place(Action):
	def __init__(self, robot, box, area):
		self.robot = robot
		self.box = box
		self.area = area
		super().__init__("Place")

	def apply(self, state):
		if not self.applicable(state):
			raise Exception('Action is not applicable in this state')

		new_state = state.copy()
		new_state.remove(Holding(self.robot, self.box))
		new_state.set(Holding(self.robot, None))
		new_state.set(In(self.box, self.area))
		return new_state

	def applicable(self, state):
		return state.check(Holding(self.robot, self.box)) and\
			state.check(Reachable(self.robot, self.area, True))

	def __str__(self):
		return '{0} Place {1} in {2}'.format(self.robot, self.box, self.area)


class Domain(object):
	def __init__(self):
		self._map = {}

	def add_object(self, obj):
		if obj.name in self._map:
			raise Exception("Object name must be unique")

		self._map[obj.name] = obj

	def get_object_by_name(self, name):
		return self._map[name]

	def __getitem__(self, key):
		return self.get_object_by_name(key)

	@abc.abstractmethod
	def get_applicable_actions(self, state):
		raise NotImplementedError()

class PickAndPlaceDomain(Domain):
	def __init__(self, workspace):
		super().__init__()
		self.robot = Robot("arm")
		self.boxes = []
		self.areas = []
		self.workspace_map = {}
		self.workspace_map[self.robot.name] = workspace.arm

		for box in workspace.boxes:
			self.add_box(box, box.name)

		for area in workspace.areas:
			self.add_area(area, area.name)

	def add_area(self, obj, name=None):
		""" Add an area to planning domain. obj is a refrence to the workspace instance """
		if name is None:
			name = "area{0}".format(len(self.areas))

		area = Area(name)
		self.areas.append(area)
		self.workspace_map[name] = obj
		self.add_object(area)
		return area

	def add_box(self, obj, name=None):
		""" Add a box to planning domain. obj is a refrence to the workspace instance """
		if name is None:
			name = "box{0}".format(len(self.boxes))

		box = Box(name)
		self.boxes.append(box)
		self.workspace_map[name] = obj
		self.add_object(box)
		return box

	def get_workspace_object(self, name):
		return self.workspace_map[name]

	def get_applicable_actions(self, state):
		actions = []

		holding = state.find(Holding, self.robot)
		if holding is None:
			for box in self.boxes:
				if state.check(Reachable(self.robot, box)):
					actions.append(Pick(self.robot, box))
		else:
			for area in self.areas:
				if state.check(Reachable(self.robot, area)):
					actions.append(Place(self.robot, holding, area))

		return actions