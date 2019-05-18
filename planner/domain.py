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
	def __init__(self, box, area, val):
		super().__init__('In', [box, area], val=True)

class Holding(Predicate):
	def __init__(self, robot, val):
		super().__init__('Holding', [robot], val)

class Free(Predicate):
	def __init__(self, robot, val):
		super().__init__('Holding', [robot], val=True)

class Reachable(Predicate):
	def __init__(self, robot, pose, val=True):
		if not (isinstance(pose, Box) or isinstance(pose, Area)):
			raise Exception(F"Invalid argument type {pose}, expected Box or Area")

		super().__init__('Reachable', [robot, pose], val)


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

		newState = state.copy()
		newState.set(Holding(self.robot, self.box))
		return newState

	def applicable(self, state):
		return state.check(Holding(self.robot, None)) and\
			state.check(Reachable(self.robot, self.box, True))

	def __str__(self):
		return '{0} Pick {1}'.format(self.robot, self.box)

class Place(Action):
	def __init__(self, robot, box, area):
		self.robot = robot
		self.box = box
		self.area = area
		super().__init__("Place")

	def apply(self, state):
		if not self.applicable(state):
			raise Exception('Action is not applicable in this state')

		newState = state.copy()
		newState.set(Holding(self.robot, self.box))
		return newState

	def applicable(self, state):
		return state.check(Holding(self.robot, self.box)) and\
			state.check(Reachable(self.robot, self.area, True))

	def __str__(self):
		return '{0} Place {1} in {2}'.format(self.robot, self.box, self.area)


class Domain(object):
	def __init__(self):
		pass

	@abc.abstractmethod
	def get_applicable_actions(self, state):
		raise NotImplementedError()

class PickAndPlaceDomain(Domain):
	def __init__(self, workspace):
		super().__init__()
		self.robot = Robot("arm")
		self.boxes = []
		self.areas = []
		self.object_map = {}
		self.object_map[self.robot.name] = workspace.arm

		for box in workspace.boxes:
			self.add_box(box)

		for area in workspace.areas:
			self.add_area(area)

	def add_area(self, obj, name=None):
		""" Add an area to planning domain. obj is a refrence to the workspace instance """
		if name is None:
			name = "area{0}".format(len(self.areas))

		if name in self.object_map:
			raise Exception("Domain object names must be unique")

		self.areas.append(Area(name))
		self.object_map[name] = obj

	def add_box(self, obj, name=None):
		""" Add a box to planning domain. obj is a refrence to the workspace instance """
		if name is None:
			name = "box{0}".format(len(self.boxes))

		if name in self.object_map:
			raise Exception("Domain object names must be unique")

		self.boxes.append(Box(name))
		self.object_map[name] = obj

	def get_applicable_actions(self, state):
		actions = []

		holding = state.find(Holding, self.robot)
		if holding is None:
			for box in self.boxes:
				#if state.check(Reacheable(self.robot, box))
				actions.append(Pick(self.robot, box))
		else:
			for area in self.areas:
				#if state.check(Reacheable(self.robot, area))
				actions.append(Place(self.robot, holding, area))

		return []