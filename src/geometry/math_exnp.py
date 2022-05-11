"""
Created on Wed Sep  7 14:00:42 2016

@author: junyi

Several math funcions not included in numpy
"""
import numpy as np

def null(A, eps = 1e-15):
    """Calculate the nullspace of a matrix, get the othonormal basis of the null space
    only works for square matrix (if not in squre, use get_Gnull)
    """
    u,s,vh = np.linalg.svd(A)
    null_mask = (s<=eps)
    null_space= np.compress(null_mask,vh,axis=0)
    
    return np.transpose(null_space)
    
def get_Gnull(G):
    """ For non-square matrix, (ususally number of row < number of column). Make a square matrix first by filling all zeros
    """
    r,c = G.shape
    G_temp = np.vstack((G,np.zeros((c-r,c))))
    G_null = null(G_temp)           # Orthonormal basis for the null space of G
    
    return G_null