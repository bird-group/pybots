"""
Created on Wed Sep  7 13:02:15 2016

@author: junyi

Contain moment of inertia things
"""
import numpy as np

def calcBox_I(m,l,w,h):
    """Calculate the moment of inertia for a box about its center of mass with x-axis along the length, 
    y-axis along the width, and z-axis along the height
    """
    I_box = np.zeros((3,3))
    I_box[0,0] = m*(w**2+h**2)/12.0
    I_box[1,1] = m*(l**2+h**2)/12.0
    I_box[2,2] = m*(l**2+w**2)/12.0
    
    return I_box
