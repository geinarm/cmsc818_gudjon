import numpy as np
import random
import time

from arm.jacobian import Jacobian
from workspace import Workspace

class Node:
	def __init__(self, pose, end, parent):
		self.parent = parent
		self.pose = pose
		self.end_point = end

	def getPlan(self):
		plan = []
		n = self
		while(n.parent != None):
			plan.append(n.pose)
			n = n.parent

		plan.reverse()
		return plan

class RRT:

	def __init__(self, ws, eps, goalR):
		self.ws = ws
		self.epsilon = eps
		self.goalRadius = goalR
		self.limits = ws.arm.joint_limits
		self.J = Jacobian(ws.arm)

		self.on_node_added_callback = None
		self.on_node_tested_callback = None
		self._interrupt = False

	def interrupt(self):
		self._interrupt = True

	def set_node_added_callback(self, callback):
		self.on_node_added_callback = callback

	def set_node_tested_callback(self, callback):
		self.on_node_tested_callback = callback

	def add_node(self, node):
		self.nodes.append(node)
		if self.on_node_added_callback is not None:
			self.on_node_added_callback(node)

	def distance_p(self, n, p):
		pn = n.end_point
		distance = np.linalg.norm(pn[0:2]-p[0:2])
		theta_err = np.abs(pn[2] - p[2])
		theta_err = min(theta_err, (np.pi - theta_err))
		return distance #+ (theta_err * 0.5)

	def distance_q(self, n, q):
		qn = n.pose
		return np.linalg.norm(qn-q)

	def closest_node_q(self, q):
		minDist = 99999
		closest = None
		for node in self.nodes:
			dist = self.distance_q(node, q)
			if dist < minDist:
				closest = node
				minDist = dist

		return closest

	def closest_node_p(self, p):
		minDist = 99999
		closest = None
		for node in self.nodes:
			dist = self.distance_p(node, p)
			if dist < minDist:
				closest = node
				minDist = dist

		return closest

	def extend(self, n, q, eps):
		v = q - n.pose
		v = v / np.linalg.norm(v)
		q_new = n.pose + (v*eps)

		self.ws.arm.set_pose(q_new)
		p = self.ws.arm.get_end_pose()

		node = Node(q_new, p, n)
		return node

	def in_bounds(self, node):
		q = node.pose
		q_min = self.limits[:, 0]
		q_max = self.limits[:, 1]
		in_bounds = np.logical_and(q >= q_min, q <= q_max).all()
		return in_bounds

	def inGoalRegion(self, node, goal):
		dist = np.linalg.norm(node.end_point - goal)
		return (dist < self.goalRadius)

	def random_sample(self):
		rand = np.random.rand(len(self.limits))
		drange = np.ptp(self.limits, axis=1)
		point = self.limits[:,0] + (rand * drange)
		return point

	def find_path(self, q_start, goal, max_nodes=1000, max_samples=10000):
		self.nodes = []
		self._interrupt = False
		self.ws.arm.set_pose(q_start)
		p_start = self.ws.arm.get_end_pose()
		n_start = Node(q_start, p_start, None)

		self.add_node(n_start)
		done = False

		ng = 0
		samples = 0
		num_miss = 0
		while len(self.nodes) < max_nodes and samples < max_samples:
			## Generate a sample and extend the tree
			if np.random.rand() < 0.05:
				## Sample goal direction
				#rand_goal_idx = np.random.choice(len(p_goals))
				# p_goals[rand_goal_idx]
				p_goal = goal.sample(self.ws.arm)
				#q = goal.sample(self.ws.arm)
				ng += 1
				
				n_close = self.closest_node_p(p_goal)
				v = (p_goal - n_close.end_point)
				if v[2] > np.pi:
					v[2] -= (2*np.pi)
				if v[2] < -np.pi:
					v[2] += (2*np.pi)
				dq = self.J.eval_inv(n_close.pose, v)
				q = n_close.pose + dq
				n_new = self.extend(n_close, q, self.epsilon)
			else:
				##Pick random point
				q = self.random_sample()
				n_close = self.closest_node_q(q)
				n_new = self.extend(n_close, q, self.epsilon)

			samples += 1
			if self.on_node_tested_callback is not None:
				self.on_node_tested_callback(n_new)

			##Check for collision
			if not self.in_bounds(n_new):
				num_miss += 1
				print("self collision")
				continue
			self.ws.arm.set_pose(n_new.pose)
			if self.ws.in_collision():
				num_miss += 1
				continue

			##Add new node
			self.add_node(n_new)

			##Check goal
			#for p_goal in p_goals:
			if goal.at_goal(self.ws.arm): #self.inGoalRegion(n_new, p_goal):
				print('Success')
				print('%d samples, %d missed, goal sampled %d times' % (samples, num_miss, ng))
				self.solution = n_new
				return n_new.getPlan()

			if self._interrupt:
				print('Interrupted')
				print('%d samples, %d missed, goal sampled %d times' % (samples, num_miss, ng))
				self.ws.arm.set_pose(q_start) ## Restore the arm to its original pose
				return None

		print('Failed')
		print('%d samples, %d missed, goal sampled %d times' % (samples, num_miss, ng))
		self.ws.arm.set_pose(q_start) ## Restore the arm to its original pose