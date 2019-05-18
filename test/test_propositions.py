import unittest

#from planner.logic import *# Object, Predicate, State, Action
from planner.domain import *

class TestPropositions(unittest.TestCase):

    def test_nothing(self):
        assert(True)

    def test_state(self):
        robot = Robot("R1")
        box1 = Box("B1")
        box2 = Box("B2")
        area1 = Area("A1")

        s1 = State()
        s1.set(Holding(robot, None))
        s1.set(In(box1, area1, True))
        s1.set(Reachable(robot, box2, True))

        assert(s1.check(Reachable(robot, box2, True)))
        assert(not s1.check(Reachable(robot, box1, True)))
        assert(not s1.check(Reachable(robot, area1, True)))
        assert(s1.check(Reachable(robot, area1, False)))

    def test_pick_action(self):
        robot = Robot("R1")
        box1 = Box("B1")
        box2 = Box("B2")
        area1 = Area("A1")

        s1 = State()
        s1.set(Holding(robot, None))

        action = Pick(robot, box1)
        assert(not action.applicable(s1))

        s1.set(Reachable(robot, box1, True))
        assert(action.applicable(s1))

        s1.set(Reachable(robot, box2, True))
        assert(action.applicable(s1))


if __name__ == '__main__':
    unittest.main()