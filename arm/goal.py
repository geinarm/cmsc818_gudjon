import abc
import numpy as np

class Goal(object):

	@abc.abstractmethod
	def at_goal(self, arm) -> bool:
		""" Return True if the arm is in a goal pose """
		raise NotImplementedError()

	@abc.abstractmethod
	def sample(self, arm):
		""" Return a pose that satisfies the goal """
		raise NotImplementedError()

class GoalPose(Goal):
	def __init__(self, pose, delta=0.01):
		self.pose = pose
		self.delta = delta

	def at_goal(self, arm):
		pose = arm.get_pose()

		assert(pose.shape == self.pose)

		err = np.linalg.norm(pose - self.pose)
		if err < self.delta:
			return True

		return False

	def sample(self, arm):
		return self.pose

class GoalEndPose(Goal):
	def __init__(self, pose, delta=0.01):
		self.pose = pose
		self.delta = delta

	def at_goal(self, arm):
		pose = arm.get_end_pose()

		p_err = np.linalg.norm(pose[0:2] - self.pose[0:2])
		t_err = np.abs(pose[2] - self.pose[2])
		if p_err < self.delta and t_err < self.delta:
			return True

		return False

	def sample(self, arm):
		return self.pose

class GoalArea(Goal):
	def __init__(self, bbox, padding=0.0):
		self.min_x = bbox[0]
		self.max_x = bbox[1]
		self.min_y = bbox[2]
		self.max_y = bbox[3]
		self.padding = padding

		self.limits = np.array([
			[self.min_x, self.max_x],
			[self.min_y, self.max_y],
			[-np.pi, np.pi]
			])

	def at_goal(self, arm):
		pose = arm.get_end_point()

		return pose[0] > self.min_x-self.padding and pose[0] < self.max_x+self.padding and\
			pose[1] > self.min_y-self.padding and pose[1] < self.max_y+self.padding

	def sample(self, arm):
		rand = np.random.rand(3)
		drange = np.ptp(self.limits, axis=1)
		point = self.limits[:,0] + (rand * drange)
		return point

class GoalCollection(Goal):
	def __init__(self, goals):
		self.goals = goals

	def at_goal(self, arm):
		for goal in self.goals:
			if goal.at_goal(arm):
				return True

		return False

	def sample(self, arm):
		rand_goal = np.random.choice(self.goals)
		return rand_goal.sample(arm)