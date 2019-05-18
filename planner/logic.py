import abc
import copy

class Predicate(object):
	def __init__(self, name, args, value):
		self.name = name
		self.args = args
		self.value = value

	def eval(self):
		return self.value

	def __str__(self):
		argStr = ",".join([str(x) for x in self.args])
		return '{0}({1}) = {2}'.format(self.name, argStr, self.value)

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return self.name == other.name and\
			self.value == other.value and\
			self.args == other.args
		return False


class Object(object):
	def __init__(self, name, typeName, static=False):
		self.name = name
		self.type = typeName
		self.static = static

	def __str__(self):
		return self.name

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return other.name == self.name

		return False		


class Action(object):
	def __init__(self, name):
		self.name = name

	@abc.abstractmethod
	def applicable(self, state) -> bool:
		return True

	@abc.abstractmethod
	def apply(self, state):
		return state

	def __str__(self):
		return self.name


class State(object):
	def __init__(self):
		self.predicates = []

	def check(self, pred):
		if (pred in self.predicates):
			return True
		else:
			if(pred.value == False):
				return True ## A missing predicate is same as false predicate?

		return False

	def find(self, type, *args):
		for pred in self.predicates:
			if isinstance(pred, type):
				pred.args == args
				return pred.value

		return None

	def set(self, pred):
		self.predicates.append(pred)

	def remove(self, pred):
		self.predicates.remove(pred)

	def copy(self):
		return copy.deepcopy(self)

	## True if state is subset of this state
	def contains(self, state):
		for pred in state.predicates:
			if not pred in self.predicates:
				return False
		return True

	def __str__(self):
		string = ",\n".join(['('+str(x)+')' for x in self.predicates])
		return string

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return other.predicates == self.predicates

		return False

