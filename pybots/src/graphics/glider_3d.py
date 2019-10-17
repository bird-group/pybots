""" glider plotting utilities
"""
import numpy as np
from geometry.quaternion import Quaternion
from geodesy.conversions import enu_to_ned

class Glider3D:
    """ 3-D glider
    """
    def __init__(self, scale_factor=1):
        """ Constructor

        Arguments:
            scale_factor: wingspan is default 1 unit, expand all dimensions by
                this factor.

        Returns:
            no returns
        """
        bWing = 1.0
        lFuse = 0.5
        bStab = 0.25
        hFin = 0.25

        lFuseFwd = lFuse*0.25
        lFuseAft = lFuse*0.65

        cWing = lFuse*0.1
        c2Wing = lFuse*0.08
        c3Wing = lFuse*0.05
        c4Wing = lFuse*0.025

        b1Wing = bWing*0.25
        b2Wing = bWing*0.15 + b1Wing
        b3Wing = bWing*0.1 + b2Wing

        z1Wing = 0.0
        z2Wing = cWing*0.2
        z3Wing = cWing*0.5
        z4Wing = cWing*1.0

        x1Wing = 0.0
        x2Wing = (cWing - c2Wing) + 0.1*cWing + x1Wing
        x3Wing = (c2Wing - c3Wing) + 0.1*cWing + x2Wing
        x4Wing = (c3Wing - c4Wing) + 0.2*cWing + x3Wing

        fFwdTop = lFuseAft + cWing - 0.05*lFuse
        fFwdBot = lFuseAft + cWing - 0.1*lFuse

        fTop = 0.15*lFuse

        cStab = 0.05*lFuse
        c2Stab = 0.5*cStab

        x1Stab = 0.95*(lFuseAft + cWing)
        x2Stab = 0.975*(lFuseAft + cWing)

        b1Stab = 0.2*bStab
        b2Stab = 0.3*bStab + b1Stab

        localX = np.array([-lFuseFwd, x1Wing, x2Wing, x3Wing, x4Wing, x4Wing + c4Wing, x3Wing + c3Wing,
            x2Wing + c2Wing, x1Wing + cWing, x2Wing + c2Wing, x3Wing + c3Wing, x4Wing + c4Wing,
            x4Wing, x3Wing, x2Wing, x1Wing, lFuseAft + cWing, lFuseAft + cWing, x2Stab + c2Stab,
            x2Stab, x1Stab, x1Stab, x2Stab, x2Stab + c2Stab, lFuseAft + cWing, fFwdTop, fFwdBot])

        localY = np.array([0.0, 0.0, b1Wing, b2Wing, b3Wing, b3Wing, b2Wing, b1Wing, 0.0, -b1Wing, -b2Wing,
            -b3Wing, -b3Wing, -b2Wing, -b1Wing, 0.0, 0.0, 0.0, b2Stab, b2Stab, b1Stab, -b1Stab,
            -b2Stab, -b2Stab, 0.0, 0.0, 0.0])

        localZ = np.array([0.0, z1Wing, z2Wing, z3Wing, z4Wing, z4Wing, z3Wing, z2Wing, z1Wing, z2Wing,
            z3Wing, z4Wing, z4Wing, z3Wing, z2Wing, z1Wing, 0.0, fTop, fTop, fTop, fTop,
            fTop, fTop, fTop, fTop, fTop, 0.0])

        self.local_points = -np.vstack((localY,localX,localZ)).T * scale_factor
        self.cg_pos = np.array([[0.0],[0.0],[0.0]])
        self.euler = np.array([0.0,0.0,0.0])

    def points(self, euler=None, cg_pos=None):
        """ Compute the points defining the glider

        Returns points in NED coordinates

        Arguments:
            euler: optionally a numpy array of euler angles defining the
                aircraft orientation. If not specified will use the euler
                property of the class
            cg_pos: optionally a numpy array defining the aircraft position. If
                not specified will use the cg_pos property

        Returns:
            X: 3xn numpy array of points defining the aircraft. in NED
        """
        if euler is not None:
            self.euler = euler
        if cg_pos is not None:
            self.cg_pos = cg_pos
        q = Quaternion()
        q.from_euler(self.euler)
        X_ac = np.array([q.rot(pt) for pt in self.local_points])
        X_ac = enu_to_ned(X_ac)
        return X_ac + self.cg_pos
