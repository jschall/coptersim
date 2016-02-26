#!/usr/bin/python
import numpy as np
from math import *
from visual import *
from CopterSim import CopterSim

def nedtovpy(vec):
    return vector(vec[1], -vec[2], -vec[0])

dt = 1./60.
t = 0.

copter = CopterSim(dt)

theta = radians(30.)
scene.forward=nedtovpy((cos(theta),0.,sin(theta)))

floor = cylinder(pos=(0,0,0), axis=nedtovpy((0.,0.,0.1)), material=materials.wood, radius=10.)

fwdarrow = arrow(pos=(0,0,0), axis=nedtovpy(copter.getForwardVecNED()), color=color.red, opacity=.5)
rgtarrow = arrow(pos=(0,0,0), axis=nedtovpy(copter.getRightVecNED()), color=color.green, opacity=.5)
uparrow =  arrow(pos=(0,0,0), axis=nedtovpy(copter.getUpVecNED()), color=color.blue, opacity=.5)

while True:
    rate(1./dt)
    copter.setThrust(20.)
    copter.setOmega((1.,0.,0.))
    copter.update()
    fwdarrow.pos = nedtovpy(copter.getPosNED()+np.array([0.,0.,-0.2]))
    fwdarrow.axis = nedtovpy(copter.getForwardVecNED())
    fwdarrow.up = nedtovpy(copter.getUpVecNED())
    rgtarrow.pos = nedtovpy(copter.getPosNED()+np.array([0.,0.,-0.2]))
    rgtarrow.axis = nedtovpy(copter.getRightVecNED())
    rgtarrow.up = nedtovpy(copter.getUpVecNED())
    uparrow.pos = nedtovpy(copter.getPosNED()+np.array([0.,0.,-0.2]))
    uparrow.axis = nedtovpy(copter.getUpVecNED())
    uparrow.up = nedtovpy(copter.getForwardVecNED())
