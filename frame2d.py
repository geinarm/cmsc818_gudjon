import numpy as np
import matplotlib.pyplot as plt

class Frame2D(object):
    def __init__(self, theta=0, x=0, y=0, parent=None, H=None):
        if H is not None:
            #self._H = H
            self._theta = np.arctan2(H[1, 0], H[0, 0])
            self._x = H[0,2]
            self._y = H[1,2]
            #self._H_inv = Frame2D._create_h_inv(self._theta, self._x, self._y)
            self._H = Frame2D._create_h(self._theta, self._x, self._y)
        else:
            #self._H_inv = Frame2D._create_h_inv(theta, x, y)
            self._theta = theta
            self._x = x
            self._y = y
            self._H = Frame2D._create_h(self._theta, self._x, self._y)

        self.parent = parent


    def transform_points(self, points):
        if len(points.shape) == 1:
            points = np.reshape(points, (1,2))

        N = len(points)
        p_temp = np.append(points, np.ones((N, 1)), axis=1)
        p_temp = np.matmul(p_temp, self._parent_H().T)

        return p_temp[:, 0:2]

    def transform_points_inv(self, points):
        if len(points.shape) == 1:
            points = np.reshape(points, (1,2))

        N = len(points)

        p_temp = np.append(points, np.ones((N, 1)), axis=1)
        p_temp = np.matmul(p_temp, self._H_inv.T)

        return p_temp[:, 0:2]

    def transform(self, frame):
        h_new = np.matmul(self._H, frame._H)
        return Frame2D(H=h_new)

    def transform_inv(self, frame):
        h_new = np.matmul(self._H_inv, frame._H)
        return Frame2D(H=h_new)

    def rotate(self, theta):
        h_temp = Frame2D._create_h(theta, 0,0)
        new_H = np.matmul(h_temp, self._H)
        return Frame2D(H=new_H)

    def set_rotation(self, theta):
        #new_H = Frame2D._create_h(theta, self._x, self._y)
        #return Frame2D(H=new_H, parent=self.parent)
        self._theta = theta
        self._H = self._create_h(self._theta, self._x, self._y)
        return self

    def translate(self, x, y):
        new_H = Frame2D._create_h(self._theta, self._x+x, self._y+y)
        return Frame2D(H=new_H)

    def _parent_H(self):
        if self.parent is None:
            return self._H
        else:
            return np.matmul(self.parent._parent_H(), self._H)

    def origin(self):
        return self.transform_points(np.array([0,0]))

    @staticmethod
    def identity():
        return Frame2D(H=np.eye(3))

    @staticmethod
    def _create_h(theta, x, y):
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0,0,1]
        ])
    @staticmethod
    def _create_h_inv(theta, x, y):
        return np.array([
            [np.cos(-theta), -np.sin(-theta), -x],
            [np.sin(-theta), np.cos(-theta), -y],
            [0,0,1]
        ])


if __name__ == '__main__':
    
    frame1 = Frame2D(0, 1, 0)
    frame2 = Frame2D(0, 0, 1, frame1)
    frame3 = Frame2D(0, 1, 0, frame2)
    frame4 = Frame2D(0, 0, 1, frame3)

    point = np.array([ [0,0] ])

    frame3 = frame3.set_rotation(np.pi/4)

    h1 = Frame2D._create_h(0,1,0)
    h2 = Frame2D._create_h(0,0,1)
    h3 = Frame2D._create_h(np.pi/2,0,0)
    h4 = Frame2D._create_h(0,0,1)

    #p1 = np.matmul(point, h1.T)
    #p2 = np.matmul(point, np.matmul(h1, h2).T )
    #p3 = np.matmul(point, np.matmul(np.matmul(h1, h2), h3).T )
    #p4 = np.matmul(point, np.matmul(np.matmul(np.matmul(h1, h2), h3), h4).T )

    p1 = frame1.transform_points(point)
    #o1 = frame1.origin()
    p2 = frame2.transform_points(point)
    #o2 = frame1.origin()
    p3 = frame3.transform_points(point)
    l = frame3.transform_points(np.array([ [0,0], [0.2,0], [0.4,0], [0.6,0] ]))
    #o3 = frame1.origin()
    p4 = frame4.transform_points(point)
    #o4 = frame1.origin()

    #plt.scatter(o1[:, 0], o1[:, 1], marker='x', color='r')
    #plt.scatter(o2[:, 0], o2[:, 1], marker='.', color='r')
    #plt.scatter(o3[:, 0], o3[:, 1], marker='o', color='r')
    #plt.scatter(o4[:, 0], o4[:, 1], marker='^', color='r')

    plt.scatter(0, 0, marker='x', color='r')
    plt.scatter(p1[:, 0], p1[:, 1], marker='x', color='b')
    plt.scatter(p2[:, 0], p2[:, 1], marker='.', color='b')
    plt.scatter(p3[:, 0], p3[:, 1], marker='o', color='b')
    plt.scatter(p4[:, 0], p4[:, 1], marker='^', color='b')
    plt.scatter(l[:, 0], l[:, 1], marker='.', color='g')

    plt.show()