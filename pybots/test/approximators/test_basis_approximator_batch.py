import pdb

import copy
import time

import numpy

import approximators.basis
import approximators.basis_functions
import potential_field

import matplotlib.pyplot as plt

###############################################################################
# this stuff isn't actually important, I'm just building a potential flow field
# to work with so that this looks vaguely like a wind field(?)
###############################################################################
xmin = 0.0
xmax = 10.0
ymin = 0.0
ymax = 10.0

v = [
    potential_field.Vortex(
        numpy.random.randn(), numpy.random.rand(2)*20 - 5) for i in range(10)
    ]
s = [potential_field.Source(
        numpy.random.randn(), numpy.random.rand(2)*20 - 5) for i in range(10)
    ]
u = [potential_field.UniformFlow(3.0, 0.0),]
d = [potential_field.Doublet(1.0, numpy.ones((2,)) * 5, numpy.array([-1,0])),]

p = u + s + v
pf = potential_field.PotentialField(p)

nplot = 20
xx = numpy.linspace(xmin, xmax, nplot)
yy = numpy.linspace(ymin, ymax, nplot)
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
n_bases = 50
bases.append(approximators.basis_functions.Uniform())
while len(bases) < n_bases:
    # randomly scatter the basis functions
    center = numpy.random.rand(2) * 10.0
    basis_centers.append(center)
    sigma = numpy.eye(2) * 1.0
    bases.append(approximators.basis_functions.GaussianRadialBasis(
        sigma, center))

# use the bases we generated to construct an approximator which has two output
# dimensions
approx = approximators.basis.NDBasisApproximator(bases, 2)

# generate some observations
wind_observations = []
for i, x in enumerate(x_sample):
    wind = numpy.random.rand(2) + numpy.array([5,1])
    wind_observations.append(wind)
wind_observations = numpy.array(wind_observations)
wind_observations = numpy.array([pf.V(x) for x in x_sample])

# and compute the least-square weight vector for our approximator which fits
# the data
C = approx.C(x_sample)
approx.w = numpy.linalg.lstsq(C, wind_observations.T.flatten())[0]

###############################################################################
# now we'll test the gradient computation
###############################################################################
max_wind_pt = approx.global_extrema(
    (
        approximators.basis.norm_functional,
        approximators.basis.grad_norm_functional),
    gradient_scale=1.0,
    #lim=([xmin, ymin], [xmax, ymax])
    lim=([xmin, xmax], [ymin, ymax])
    )
min_wind_pt = approx.global_extrema(
    (
        approximators.basis.norm_functional,
        approximators.basis.grad_norm_functional),
    gradient_scale=-1.0,
    #lim=([xmin, ymin], [xmax, ymax])
    lim=([xmin, xmax], [ymin, ymax])
    )
min_wind_mag = numpy.linalg.norm(approx.value(min_wind_pt[None]))
max_wind_mag = numpy.linalg.norm(approx.value(max_wind_pt[None]))
print('minimum wind: {:0.2f} at {}'.format(min_wind_mag, min_wind_pt))
print('maximum wind: {:0.2f} at {}'.format(max_wind_mag, max_wind_pt))

###############################################################################
# The rest of this is just plotting for visualization
###############################################################################

V2 = approx.value(x_plot)
i_max = numpy.linalg.norm(V2, axis=1).argmax()
i_min = numpy.linalg.norm(V2, axis=1).argmin()
print('minimum grid wind: {:0.2f} at {}'.format(
    numpy.linalg.norm(V2[i_min]), x_plot[i_min]))
print('maximum grid wind: {:0.2f} at {}'.format(
    numpy.linalg.norm(V2[i_max]), x_plot[i_max]))

plt.figure()
plt.imshow(
    numpy.linalg.norm(V2, axis=1).reshape(20,20),
    origin='lower',
    extent=(xmin, xmax, ymin, ymax))
plt.quiver(x_plot[:,0], x_plot[:,1], V2[:,0], V2[:,1])
plt.quiver(x_plot[:,0], x_plot[:,1], V[:,0], V[:,1], color='#00FF00')
basis_centers = numpy.array(basis_centers)
plt.scatter(basis_centers[:,0], basis_centers[:,1])
plt.scatter(min_wind_pt[0], min_wind_pt[1], marker='x', s=200, color='r')
plt.scatter(max_wind_pt[0], max_wind_pt[1], marker='o', s=200, color='r')
plt.scatter(x_plot[i_min][0], x_plot[i_min][1], marker='x', s=200, color='c')
plt.scatter(x_plot[i_max][0], x_plot[i_max][1], marker='o', s=200, color='c')
plt.axis('equal')

plt.show()

