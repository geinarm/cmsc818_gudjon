import numpy as np
import random

from workspace import Workspace
import matplotlib.pyplot as plt

class Node:

	def __init__(self, point, parent):
		self.parent = parent
		self.point = point

	def getPlan(self):
		plan = []
		n = self
		while(n.parent != None):
			plan.append(n.point)
			n = n.parent

		plan.reverse()
		return plan


class RRT:

	def __init__(self, ws, eps, goalR, limits):
		self.ws = ws
		self.epsilon = eps
		self.goalRadius = goalR
		self.limits = limits
		self.dimentions = len(limits)

	def distance(self, p1, p2):
		#return np.linalg.norm(p1-p2)
		#return np.sum(np.abs(p1 - p2))
		#return np.max(np.abs(p1 - p2))

		self.ws.arm.set_pose(p1)
		q1 = self.ws.arm.get_end_point()
		self.ws.arm.set_pose(p2)
		q2 = self.ws.arm.get_end_point()
		return np.linalg.norm(q1-q2)

	def closestNode(self, point, theshold=0):
		minDist = 99999
		closest = None
		for node in self.nodes:
			dist = self.distance(node.point, point)
			#if dist < theshold:
			#	continue
			if dist < minDist:
				closest = node
				minDist = dist

		return closest

	def extend(self, n, p, eps):
		#v = n.point - p
		v = p - n.point
		v = v / np.linalg.norm(v)
		p_new = n.point + (v*eps)

		node = Node(p_new, n)
		return node

	def inGoalRegion(self, node, goal):
		dist = self.distance(node.point, goal)
		return (dist < self.goalRadius)

	def findPath(self, start, goal, maxNodes=1000, maxSamples=10000, ax=None):
		self.nodes = []
		self.start = start
		self.goal = goal
		self.solution = None

		self.nodes.append(Node(start, None))
		done = False

		ax.scatter(goal[0]+3, goal[1], c='orange', s=30, marker='x')

		ng = 0
		samples = 0
		num_miss = 0
		while len(self.nodes) < maxNodes and samples < maxSamples:
			numNodes = len(self.nodes)
			pg = (float(numNodes)/maxNodes) * 0.3
			useGoal = False
			##Pick random point
			if np.random.rand() > (1.0-pg):
				ng += 1
				useGoal = True
				point = goal
			else:
				values = []
				for i in range(self.dimentions):
					val = random.uniform(self.limits[i][0], self.limits[i][1])
					values.append(val)
				point = np.array(values)

			samples += 1
			##Find closest node
			n_close = self.closestNode(point)
			#p_close = n_close.point

			##Extend node towards sample point
			goalDist = self.distance(n_close.point, goal)
			eps = min(goalDist, self.epsilon)
			ax.scatter(point[0]+3, point[1], c='g', s=5)
			n_new = self.extend(n_close, point, eps)

			##Check for collision
			self.ws.arm.set_pose(n_new.point)
			if self.ws.in_collision():
				num_miss += 1
				ax.scatter(n_new.point[0]+3, n_new.point[1], c='r', s=5)
				continue
			#if not self.ws.pointInBounds(newNode.point):
			#	continue

			#robot.setState(newNode.point)
			#collider = robot.getCollider()
			#if self.ws.inCollision(collider):
			#	continue
			#pathCollider = TrajectoryCollider(trajectory, robot.getWidth())
			#if self.ws.inCollision(pathCollider):
			#	continue

			##Add new node
			self.nodes.append(n_new)
			ax.scatter(n_new.point[0]+3, n_new.point[1], c='b', s=5)
			self.ws.draw(ax, 0.1)
			#plt.pause(0.1)
			#print(newNode.point)

			##Check goal
			if self.inGoalRegion(n_new, goal):
				print('Success')
				print('%d samples, %d missed, goal sampled %d times' % (samples, num_miss, ng))
				self.solution = n_new
				return n_new.getPlan()

		print('Failed')
		print('%d samples, %d missed, goal sampled %d times' % (samples, num_miss, ng))