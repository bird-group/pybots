#!/usr/bin/env python3

import numpy
import pdb
import scipy.spatial
import shapely.geometry
import time
import yaml

import geodesy.conversions
import navigation.obstacle_space
import navigation.obstacle_primitives

import matplotlib.pyplot as plt

if __name__ == '__main__':
    # use the obstacle file with radan degrees or no
    use_radians = True

    if use_radians:
        obstacles_file = 'minneapolis.yaml'
    else:
        obstacles_file = 'minneapolis_deg.yaml'

    with open(obstacles_file, 'r') as yfile:
        obstacle_data = yaml.load(yfile)

    ref_pt = numpy.array(obstacle_data['lla_ref'], ndmin=2)
    obstacles = obstacle_data['obstacles']
    if use_radians:
        ospace = navigation.obstacle_space.PrismaticGeographicObstacleSpace(
            obstacles, ref_pt, is_radians=True)
    else:
        ospace = navigation.obstacle_space.PrismaticGeographicObstacleSpace(
            obstacles, ref_pt, is_radians=False)

    patches = []
    for o in ospace._obstacles:
        p = numpy.array(o._shape.exterior)
        plt.plot(p[:,1], p[:,0])

        x = []
        y = []
        z = []
        for i in range(p.shape[0] - 1):
            x += [p[i,0], p[i+1,0], p[i+1,0] + 0.01, p[i,0] + 0.01, p[i,0]]
            y += [p[i,1], p[i+1,1], p[i+1,1], p[i,1], p[i,1]]
            z += [o._zt, o._zt, 0.0, 0.0, o._zt]
        x += p[:,0].tolist()
        y += p[:,1].tolist()
        z += (numpy.zeros(p[:,0].shape) + o._zt).tolist()
        patches.append((x,y,z))

    ned_paths = numpy.random.rand(100,3)
    ned_paths[:,0] = 2000.0 * ned_paths[:,0] - 1000.0
    ned_paths[:,1] = 2000.0 * ned_paths[:,1] - 1000.0
    ned_paths[:,2] *= -600.0

    # add some interesting test cases. These points will force lines that cross
    # the top and bottom surfaces of some obstacles including lines which begin
    # with no, one, or both points lying in the flat projected area of an
    # obstacle
    ned_paths = numpy.vstack((
        ned_paths,
        numpy.array([-0.0, 0.0, 100.0]),
        numpy.array([-265.0, 0.0, -900.0]),
        numpy.array([-200.0, 0.0, 100.0]),
        numpy.array([-210.0, 0.0, -600.0]),
        numpy.array([-000.0, 0.0, 100.0]),
        ))

    lla_paths = geodesy.conversions.ned_to_lla(ned_paths, ref_pt)

    intersections = []
    start_time = time.time()
    # we can check each segment individually
    for i in range(lla_paths.shape[0] - 1):
        ip = ospace.intersections(lla_paths[i:i+2,:])
        intersections.extend(ip)
    # or check them all as a single path. Interestingly this is about twice as
    # slow as checking each segment individually
    #intersections = ospace.intersections(lla_paths)
    end_time = time.time()
    elapsed_time = end_time - start_time
    print('elapsed time: {:0.2f} s, {:0.5f} s per calculation'.format(
        elapsed_time, elapsed_time / ned_paths.shape[0]))

    plt.plot(ned_paths[:,1], ned_paths[:,0])
    for i in intersections:
        ned_i = geodesy.conversions.lla_to_ned(numpy.array(i, ndmin=2), ref_pt)
        plt.scatter(ned_i[:,1], ned_i[:,0])

    # test computing the direction to nearest obstacle
    for pt in lla_paths:
        if ospace.is_point_obstructed(pt):
            continue
        ned_pt = geodesy.conversions.lla_to_ned(
            numpy.array(pt, ndmin=2), ref_pt)
        plt.scatter(ned_pt[:,1], ned_pt[:,0], marker='x')

        to_nearest_boundary = ospace.nearest_boundary(pt)
        line_to_boundary = numpy.vstack((ned_pt, ned_pt + to_nearest_boundary))
        plt.plot(line_to_boundary[:,1], line_to_boundary[:,0])

    plt.xlabel('east')
    plt.ylabel('north')
    plt.grid()
    plt.axis('equal')
    plt.show()

    o = ospace._obstacles[0]
    b = shapely.geometry.Polygon(o._shape)
    s = navigation.obstacle_primitives.ShapePrimitive(b)
