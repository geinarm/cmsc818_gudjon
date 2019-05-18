import unittest
import numpy as np

from arm.arm import Frame2D

class TestFrame2D(unittest.TestCase):

    @unittest.skip
    def test_transform_point(self):
        p1 = np.array([1,0])
        f1 = Frame2D(np.pi, 0, 0)
        f2 = Frame2D(np.pi/2, 0, 0)
        f3 = Frame2D(0, 3, 0)

        q1 = f1.transform_points(p1)
        q2 = f2.transform_points(p1)
        q3 = f3.transform_points(p1)

        q1_inv = f1.transform_points_inv(p1)
        q2_inv = f2.transform_points_inv(p1)
        q3_inv = f3.transform_points_inv(p1)

        np.testing.assert_almost_equal(q1, np.array([[-1,0]]))
        np.testing.assert_almost_equal(q2, np.array([[0,1]]))
        np.testing.assert_almost_equal(q3, np.array([[4,0]]))

        np.testing.assert_almost_equal(q1_inv, np.array([[-1,0]]))
        np.testing.assert_almost_equal(q2_inv, np.array([[0,-1]]))
        np.testing.assert_almost_equal(q3_inv, np.array([[-2,0]]))

    @unittest.skip
    def test_rotate(self):
        p1 = np.array([0,0])
        f1 = Frame2D(0, 1, 0)
        f2 = f1.rotate(-np.pi/2)
        f3 = f1.rotate(np.pi/2)

        q1 = f1.transform_points(p1)
        q2 = f2.transform_points(p1)
        q3 = f3.transform_points(p1)

        np.testing.assert_almost_equal(q1, np.array([[1,0]]))
        np.testing.assert_almost_equal(q2, np.array([[0,-1]]))
        np.testing.assert_almost_equal(q3, np.array([[0,1]]))

    @unittest.skip
    def test_frame_chain(self):
        p1 = np.array([3,0])
        p2 = np.array([-2,4])

        f1 = Frame2D(np.pi/2, 0, 0)
        f2 = Frame2D(0, 1, 0)
        f3 = Frame2D(0, 0, 2)

        f = f1.transform(f2).transform(f3)

        q1 = f.transform_points(p1)
        q2 = f.transform_points_inv(p2)
        
        np.testing.assert_almost_equal(q1, np.array([[-2,4]]))
        np.testing.assert_almost_equal(q2, np.array([[3,0]]))


if __name__ == '__main__':
    unittest.main()