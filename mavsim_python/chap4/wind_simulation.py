"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.aerosonde_parameters as MAV
from IPython.core.debugger import Pdb

class wind_simulation:
    def __init__(self, Ts):
        # steady state wind defined in the inertial frame
        # self._steady_state = np.array([0., 0., 0.])
        self._steady_state = np.array([3., 1., 0.])

        #   Dryden gust model parameters (pg 56 UAV book)
        # HACK:  Setting Va to a constant value is a hack.  We set a nominal airspeed for the gust model.
        # Could pass current Va into the gust function and recalculate A and B matrices.
        self.sigu = 2.12
        self.sigv = self.sigu
        self.sigw = 1.4
        self.Lu = 200.0
        self.Lv = self.Lu
        self.Lw = 50.0

        #   Dryden gust model parameters (pg 56 UAV book)
        # HACK:  Setting Va to a constant value is a hack.  We set a nominal airspeed for the gust model.
        # Could pass current Va into the gust function and recalculate A and C matrices.
        self.Va = MAV.u0
        self._A = self._compute_A()
        self._B = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 0],
                            [0, 0, 1],
                            [0, 0, 0]])
        self._C = self._compute_C()
        self._gust_state = np.zeros(5)
        self._Ts = Ts


    def update(self, Va):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        self.Va = Va
        return np.concatenate(( self._steady_state, self._gust() ))

    def _compute_A(self):
        A = np.array([[-self.Va/self.Lu, 0, 0, 0, 0],
                            [0, -2*(self.Va/self.Lv), -(self.Va/self.Lv)**2, 0, 0],
                            [0, 1, 0, 0, 0],
                            [0, 0, 0, -2*(self.Va/self.Lw), -(self.Va/self.Lw)**2],
                            [0, 0, 0, 1, 0]])
        return A

    def _compute_C(self):
        C = np.array([[self.sigu * np.sqrt((2*self.Va)/self.Lu), 0, 0, 0, 0],
                      [0, self.sigv * np.sqrt((3*self.Va)/self.Lv), np.sqrt((self.Va/self.Lv)**3), 0, 0],
                      [0, 0, 0, self.sigw * np.sqrt((3*self.Va)/self.Lv), np.sqrt((self.Va/self.Lw)**3)]])
        return C

    def _gust(self):
        self._A = self._compute_A()
        self._C = self._compute_C()
        # calculate wind gust using Dryden model.  Gust is defined in the body frame
        w = np.random.randn(3)  # zero mean unit variance Gaussian (white noise)
        # propagate Dryden model (Euler method): x[k+1] = x[k] + Ts*( A x[k] + B w[k] )
        self._gust_state += self._Ts * (self._A @ self._gust_state + self._B @ w)
        # output the current gust: y[k] = C x[k]
        return self._C @ self._gust_state
