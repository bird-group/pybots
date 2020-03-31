import pdb

import copy
import time

import numpy

import approximators.basis
import approximators.basis_functions
import potential_field

import matplotlib.pyplot as plt

# this stuff isn't actually important, I'm just building a potential flow field
# to work with so that this looks vaguely like a wind field(?)
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
v = numpy.linalg.norm(V, axis=1)
plt.quiver(x_plot[:,0], x_plot[:,1], V[:,0], V[:,1])
plt.axis('equal')


P = numpy.array([pf.potential(xi) for xi in XX])
#P[r < R] = numpy.nan
#P = numpy.reshape(P, (nplot,nplot))
#plt.figure()
#plt.imshow(P, origin='lower')

bases = []
basis_centers = []
n_bases = 20
bases.append(approximators.basis_functions.Uniform())
while len(bases) < n_bases:
    center = numpy.random.rand(2) * 10.0# - 5.0
    basis_centers.append(center)
    sigma = numpy.eye(2) * 10.0
    bases.append(approximators.basis_functions.GaussianRadialBasis(
        sigma, center))

kf = approximators.basis.NDBasisKalmanFilter(
    bases, 2, R=numpy.eye(2))
#approx = approximators.basis.BasisKalmanFilter(
#    [bases,]*2, [numpy.zeros((n_bases,)),] * 2, [numpy.eye(n_bases) * 10.0,] * 2
#    )

for i, x in enumerate(x_sample):
    if i % 100 == 0:
        print('{:0.0f} % complete'.format(i / 10))
    V_obs = pf.V(x)
    kf.measurement_update(x, V_obs)

V2 = []
for i in range(len(x_plot)):
    V2.append(kf.value(x_plot[i:i+1,:]))

V2 = numpy.array(V2)

plt.figure()
plt.quiver(x_plot[:,0], x_plot[:,1], V2[:,0], V2[:,1])
plt.quiver(x_plot[:,0], x_plot[:,1], V[:,0], V[:,1], color='#00FF00')
#plt.plot(xsave[:,0], xsave[:,1])
basis_centers = numpy.array(basis_centers)
plt.scatter(basis_centers[:,0], basis_centers[:,1])
plt.axis('equal')

#P2 = kf.value(XX)

#P2[r < R] = numpy.nan
#P2 = numpy.reshape(P2, (nplot,nplot))
#P2 -= numpy.nanmean(P2) - numpy.nanmean(P)
#plt.figure()
#plt.imshow(P2, origin='lower')

#dP = P - P2
#plt.figure()
#plt.imshow(dP, origin='lower')

plt.show()
