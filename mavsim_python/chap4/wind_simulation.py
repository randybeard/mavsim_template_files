"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
import sys
sys.path.append('..')
import numpy as np

class wind_simulation:
    def __init__(self, Ts):
        # steady state wind defined in the inertial frame
        # self._steady_state = np.array([0., 0., 0.])
        self._steady_state = np.array([3., 1., 0.])

        #   Dryden gust model parameters (pg 56 UAV book)
        # HACK:  Setting Va to a constant value is a hack.  We set a nominal airspeed for the gust model.
        # Could pass current Va into the gust function and recalculate A and B matrices.
        Va = 17

        sigu = 2.12
        sigv = sigu
        sigw = 1.4
        Lu = 200
        Lv = Lu
        Lw = 500

        au = 0
        bu = sigu*np.sqrt(2*Va/Lu)
        cu = 1
        du = Va/Lu

        av = sigv*np.sqrt(3*Va/Lv)
        bv = av * Va/(np.sqrt(3)*Lv)
        cv = 2*Va/Lv
        dv = (Va/Lv)**2

        aw = sigw*np.sqrt(3*Va/Lw)
        bw = aw * Va/(np.sqrt(3)*Lw)
        cw = 2*Va/Lw
        dw = (Va/Lw)**2

        self._Au = np.array([[1 - Ts*cu, -Ts*du], [Ts, 1]])
        self._Bu = np.array([Ts, 0])
        self._Cu = np.array([au, bu])

        self._Av = np.array([[1 - Ts*cv, -Ts*dv], [Ts, 1]])
        self._Bv = np.array([Ts, 0])
        self._Cv = np.array([av, bv])

        self._Aw = np.array([[1 - Ts*cw, -Ts*dw], [Ts, 1]])
        self._Bw = np.array([Ts, 0])
        self._Cw = np.array([aw, bw])

        self._gust_state = np.array([1., 2., 3.])
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        return np.concatenate(( self._steady_state, self._gust() ))

    def _gust(self):
        # calculate wind gust using Dryden model.  Gust is defined in the body frame
        w = np.random.randn(3)  # zero mean unit variance Gaussian (white noise)
        # propagate Dryden model (Euler method): x[k+1] = x[k] + Ts*( A x[k] + B w[k] )
        self._gust_state += self._Au @ self._gust_state + self._Bu * w[0]
        self._gust_state += self._Av @ self._gust_state + self._Bv * w[1]
        self._gust_state += self._Aw @ self._gust_state + self._Bw * w[2]
        # output the current gust: y[k] = C x[k]
        return self._C @ self._gust_state
