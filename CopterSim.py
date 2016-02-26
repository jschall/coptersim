import numpy as np
from scipy.integrate import odeint
from simhelpers import quatderiv
from math import *

class CopterSim:
    def __init__(self, dt):
        self.state = np.array([
            1.,0.,0.,0., # orientation quaternion
            0.,0.,0., #velocity
            0.,0.,-5. #position
            ])

        self.mass = 1.
        ground_contact_freq = 10.
        ground_contact_damping_ratio = 0.2
        self.ground_contact_stiffness = 0.5*self.mass*(ground_contact_freq*2.*pi)**2
        self.ground_contact_damping = 2.*ground_contact_damping_ratio*sqrt(self.ground_contact_stiffness*0.5*self.mass)

        self.omega = np.array([0.0,0.0,0.0])

        self.t = 0.
        self.dt = dt

    def update(self):
        x = np.linspace(self.t, self.t+self.dt, 2.)
        self.t += self.dt
        self.state = odeint(self.dyn, self.state, x)[-1]

    def dyn(self,y,t):
        #these let us call member functions
        self.t = t
        self.state = y

        force = self.getGravityForceNED() + self.getGroundContactForceNED() + self.getThrustForceNED()

        return np.concatenate((quatderiv(self.state[0:4], self.omega), force/self.mass, self.getVelNED()))

    def getGravityForceNED(self):
        return np.array([0.,0.,9.80665])

    def getGroundContactForceNED(self):
        ground_height = 0.
        if self.getPosNED()[2]>ground_height:
            spring_compression = self.getPosNED()[2] - ground_height
            return np.array([0.,0.,-spring_compression*self.ground_contact_stiffness-self.getVelNED()[2]*self.ground_contact_damping])
        else:
            return np.array([0.,0.,0.])

    def getThrustForceNED(self):
        return np.asarray(self.getRotationBodyToNED() * np.matrix([0.,0.,0.]).T).flatten()

    def getRotationBodyToNED(self):
        qr = self.state[0]
        qi = self.state[1]
        qj = self.state[2]
        qk = self.state[3]
        return np.matrix([[1.-2.*(qj**2+qk**2),    2.*(qi*qj-qk*qr),    2.*(qi*qk+qj*qr)],
                          [   2.*(qi*qj+qk*qr), 1.-2.*(qi**2+qk**2),    2.*(qj*qk-qi*qr)],
                          [   2.*(qi*qk-qj*qr),    2.*(qi*qr+qj*qk), 1.-2.*(qi**2+qj**2)]])

    def getQuaternion(self):
        return self.state[0:4]/np.linalg.norm(self.state[0:4])

    def getVelNED(self):
        return self.state[4:7]

    def getPosNED(self):
        return self.state[7:10]

    def setOmega(self, omega):
        self.omega = np.asarray(omega)

    def getUpVecNED(self):
        Tbn = self.getRotationBodyToNED()
        return Tbn * np.matrix([0.,0.,-1.]).T

    def getForwardVecNED(self):
        Tbn = self.getRotationBodyToNED()
        return Tbn * np.matrix([1.,0.,0.]).T

    def getRightVecNED(self):
        Tbn = self.getRotationBodyToNED()
        return Tbn * np.matrix([0.,1.,0.]).T
