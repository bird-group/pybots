import pdb

import copy
import time

import numpy

import approximators.basis
import approximators.basis_functions
import potential_field.potential_field

import matplotlib.pyplot as plt

###############################################################################
# this stuff isn't actually important, I'm just building a potential flow field
# to work with so that this looks vaguely like a wind field(?)
###############################################################################
v = [
    potential_field.potential_field.Vortex(
        numpy.random.randn(), numpy.random.rand(2)*20 - 5) for i in range(10)
    ]
s = [potential_field.potential_field.Source(
        numpy.random.randn(), numpy.random.rand(2)*20 - 5) for i in range(10)
    ]
u = [potential_field.potential_field.UniformFlow(3.0, 0.0),]
d = [
        potential_field.potential_field.Doublet(
            1.0, numpy.ones((2,)) * 5, numpy.array([-1,0])),
    ]

p = u + s + v
pf = potential_field.potential_field.PotentialField(p)

nplot = 20
xx = numpy.linspace(0, 10.0, nplot)
yy = numpy.linspace(0, 10.0, nplot)
X,Y = numpy.meshgrid(xx,yy)
XX = numpy.stack([X.flatten(), Y.flatten()]).T
R = numpy.sqrt(5.0 / numpy.pi / 2.0 / 3.0 / numpy.pi / 2.0)

x_sample = numpy.random.rand(1000,2) * 10.0
x_plot = XX
V = []
for x in x_plot:
    V.append(pf.V(x))
V = numpy.array(V)

###############################################################################
# This is where we do all of the estimation
###############################################################################

# create a set of bases. We'll use 20 bases for each wind field component. The
# first basis will be a uniform bias on the field, the remaining will be
# gaussian radial basis functions to capture perturbations from the uniform
bases = []
basis_centers = []
n_bases = 20
bases.append(approximators.basis_functions.Uniform())
while len(bases) < n_bases:
    # randomly scatter the basis functions
    center = numpy.random.rand(2) * 10.0
    basis_centers.append(center)
    sigma = numpy.eye(2) * 10.0
    bases.append(approximators.basis_functions.GaussianRadialBasis(
        sigma, center))

# use the bases we generated to construct two approximators. One will be a
# kalman filter for iteratively approximating the wind field from observations.
# the other will be just for processing a batch of observations and won't
# include any probabilistic information. Both will have two output dimensions
kf = approximators.basis.NDBasisKalmanFilter(
    bases, 2, R=numpy.eye(2), P0=numpy.eye(40) * 1000.0)
approx = approximators.basis.NDBasisApproximator(
    bases, 2, sigma=numpy.eye(n_bases * 2))

# generate some observations of the wind field, runthe kalman filter as we do
VV = []
for i, x in enumerate(x_sample):
    if i % 100 == 0:
        print('{:0.0f} % complete'.format(i / 10))
    V_obs = pf.V(x)
    VV.append(V_obs)
    kf.measurement_update(x, V_obs)
VV = numpy.array(VV)

# use the batch of data to generate least square weights for the batch
# approximator
C = approx.C(x_sample)
w = numpy.linalg.lstsq(C, VV.T.flatten())[0]
approx.w = w

###############################################################################
# The rest of this is just plotting for visualization
###############################################################################

V2 = kf.value(x_plot)
V3 = approx.value(x_plot)

S = kf.sample(x_plot)
Sa = approx.sample(x_plot)

plt.figure()
plt.quiver(x_plot[:,0], x_plot[:,1], V2[:,0], V2[:,1])
plt.quiver(x_plot[:,0], x_plot[:,1], V3[:,0], V[:,1], color='r')
plt.quiver(x_plot[:,0], x_plot[:,1], V[:,0], V[:,1], color='#00FF00')
basis_centers = numpy.array(basis_centers)
plt.scatter(basis_centers[:,0], basis_centers[:,1])
plt.axis('equal')

plt.figure()
plt.quiver(x_plot[:,0], x_plot[:,1], V2[:,0], V2[:,1], label='kf mean')
plt.quiver(
    x_plot[:,0], x_plot[:,1], S[:,0], S[:,1],
    color='#00FF00', label='kf sample')
plt.axis('equal')
plt.legend()

plt.show()
