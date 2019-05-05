import numpy as np

class Jacobian(object):
    def __init__(self, arm):
        self.arm = arm
        self.L = arm.link_lengths

    def eval(self, q, dq):
        """
        Estimate the linearized movement of the end effector given a pose q, and a change in pose dq
        """
        assert(len(q) == len(dq))
        assert(len(self.L) == len(q))

        J = self._compute_J(q)

        dx = np.matmul(J, dq)
        return dx

    def eval_inv(self, q, dx):
        """
        Estimate the change in pose that moves the end effector in the direction dx
        """
        assert(len(self.L) == len(q))
        assert(len(dx) == 3)

        J = self._compute_J(q)
        J_inv = np.linalg.pinv(J)
        dq = np.matmul(J_inv, dx)

        return dq

    def _compute_J(self, q):
        J = np.zeros( (3, len(q)) )

        for i in range(len(q)):
            #dx/dq_i 
            jx = 0
            for j in range(i, len(q)):
                jx += self.L[j] * -np.sin( np.sum(q[0:j+1]) )
            
            #dy/dq_i 
            jy = 0
            for j in range(i, len(q)):
                jy += self.L[j] * np.cos( np.sum(q[0:j+1]) )

            J[0, i] = jx
            J[1, i] = jy
            J[2, i] = 1

        return J