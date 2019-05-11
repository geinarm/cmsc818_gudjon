import numpy as np
import random

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

	def __init__(self, ws, eps, goalR, limits):
		self.ws = ws
		self.epsilon = eps
		self.goalRadius = goalR
		self.limits = limits
		self.J = Jacobian(ws.arm)

		self.on_node_added_callback = None
		self._interrupt = False

	def interrupt(self):
		self._interrupt = True

	def set_node_added_callback(self, callback):
		self.on_node_added_callback = callback

	def add_node(self, node):
		self.nodes.append(node)
		if self.on_node_added_callback is not None:
			self.on_node_added_callback(node)

	def distance_p(self, n, p):
		pn = n.end_point
		return np.linalg.norm(pn-p)

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
		p = self.ws.arm.get_end_point()

		node = Node(q_new, p, n)
		return node

	def inGoalRegion(self, node, goal):
		dist = np.linalg.norm(node.end_point - goal)
		return (dist < self.goalRadius)

	def random_sample(self):
		rand = np.random.rand(len(self.limits))
		drange = np.ptp(self.limits, axis=1)
		point = self.limits[:,0] + (rand * drange)
		return point

	def find_path(self, q_start, p_goal, max_nodes=1000, max_samples=10000):
		self.nodes = []
		self._interrupt = False
		self.ws.arm.set_pose(q_start)
		p_start = self.ws.arm.get_end_point()
		n_start = Node(q_start, p_start, None)

		#self.nodes.append(n_start)
		self.add_node(n_start)
		done = False

		ng = 0
		samples = 0
		num_miss = 0
		while len(self.nodes) < max_nodes and samples < max_samples:
			numNodes = len(self.nodes)
			
			## Generate a sample and extend the tree
			if np.random.rand() < 0.15:
				## Sample goal direction
				n_close = self.closest_node_p(p_goal)
				v = np.squeeze(p_goal - n_close.end_point)
				v = v / np.linalg.norm(v)
				dq = self.J.eval_inv(n_close.pose, v)
				q = n_close.pose + dq
			else:
				##Pick random point
				q = self.random_sample()
				n_close = self.closest_node_q(q)

			samples += 1
			##Find closest node
			#n_close = self.closestNode(q)
			#p_close = n_close.point

			##Extend node towards sample point
			#goalDist = self.distance(n_close, p_goal)
			#eps = min(goalDist, self.epsilon)
			n_new = self.extend(n_close, q, self.epsilon)

			##Check for collision
			self.ws.arm.set_pose(n_new.pose)
			if self.ws.in_collision():
				num_miss += 1
				continue

			##Add new node
			#self.nodes.append(n_new)
			self.add_node(n_new)
			#plt.pause(0.1)
			#print(newNode.point)

			##Check goal
			if self.inGoalRegion(n_new, p_goal):
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