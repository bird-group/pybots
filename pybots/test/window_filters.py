import pdb
import sys

import numpy

import filters.window
import geodesy.conversions

import matplotlib.pyplot as plt

if __name__ == '__main__':
    #pdb.set_trace()

    r1 = {}
    r10 = {}
    r50 = {}
    r100 = {}

    if sys.argv[1] == 'sample' or sys.argv[1] == 'all':
        # test the sample window filter
        data = numpy.arange(1,101)
        sw_filter = filters.window.Average(10)

        result_1 = []
        result_10 = []
        result_50 = []
        result_100 = []

        for x in data:
            sw_filter.add(x)
            result_1.append(sw_filter.mean(n=1))
            result_10.append(sw_filter.mean())
            result_50.append(sw_filter.mean(n=50))
            result_100.append(sw_filter.mean(n=100))

        result_1 = numpy.array(result_1, ndmin=2).T
        result_10 = numpy.array(result_10, ndmin=2).T
        result_50 = numpy.array(result_50, ndmin=2).T
        result_100 = numpy.array(result_100, ndmin=2).T

        plt.plot(result_1, label='n=1')
        plt.plot(result_10, label='n=10')
        plt.plot(result_50, label='n=50')
        plt.plot(result_100, label='n=100')
        plt.grid()
        plt.legend()
        plt.xlabel('sample')
        plt.ylabel('window average')
        plt.title('sample window')
        if sys.argv[1] != 'all':
            plt.show()

        r1['sample'] = result_1
        r10['sample'] = result_10
        r50['sample'] = result_50
        r100['sample'] = result_100

    if sys.argv[1] == 'time' or sys.argv[1] == 'all':
        # test the time window filter
        time = numpy.arange(1, 101)
        sw_filter = filters.window.TimeAverage(10)

        result_1 = []
        result_10 = []
        result_50 = []
        result_100 = []

        for x, t in zip(data, time):
            sw_filter.add(x, t)
            result_1.append(sw_filter.mean(t_avg=1))
            result_10.append(sw_filter.mean())
            result_50.append(sw_filter.mean(t_avg=50))
            result_100.append(sw_filter.mean(t_avg=100))

        result_1 = numpy.array(result_1, ndmin=2).T
        result_10 = numpy.array(result_10, ndmin=2).T
        result_50 = numpy.array(result_50, ndmin=2).T
        result_100 = numpy.array(result_100, ndmin=2).T

        plt.plot(result_1, label='t=1')
        plt.plot(result_10, label='t=10')
        plt.plot(result_50, label='t=50')
        plt.plot(result_100, label='t=100')
        plt.grid()
        plt.legend()
        plt.xlabel('sample')
        plt.ylabel('window average')
        plt.title('time window')
        if sys.argv[1] != 'all':
            plt.show()

        r1['time'] = result_1
        r10['time'] = result_10
        r50['time'] = result_50
        r100['time'] = result_100

    if sys.argv[1] == 'nd' or sys.argv[1] == 'all':
        # test the sample window filter for n-d data
        sw_filter = filters.window.Average(10)

        r = 1.e3
        theta = numpy.linspace(-numpy.pi, numpy.pi, 100)

        X = numpy.vstack((
            r * numpy.cos(theta),
            r * numpy.sin(theta),
            numpy.zeros(theta.shape),
            numpy.arange(1, theta.shape[0] + 1))).T

        result_1 = []
        result_10 = []
        result_50 = []
        result_100 = []

        for x in X:
            sw_filter.add(x)
            result_1.append(sw_filter.mean(n=1))
            result_10.append(sw_filter.mean())
            result_50.append(sw_filter.mean(n=50))
            result_100.append(sw_filter.mean(n=100))

        result_1 = numpy.array(result_1)
        result_10 = numpy.array(result_10)
        result_50 = numpy.array(result_50)
        result_100 = numpy.array(result_100)

        f, axs = plt.subplots(result_1.shape[1], 1, sharex=True)

        for i, ax in enumerate(axs):
            ax.plot(result_1[:,i], label='n=1')
            ax.plot(result_10[:,i], label='n=10')
            ax.plot(result_50[:,i], label='n=50')
            ax.plot(result_100[:,i], label='n=100')
            ax.grid()
            ax.set_ylabel('window average')
        ax.legend()
        ax.set_xlabel('sample')
        plt.title('sample window, nd-data')

        plt.figure()
        plt.plot(result_1[:,0], result_1[:,1], label='n=1')
        plt.plot(result_10[:,0], result_10[:,1], label='n=10')
        plt.plot(result_50[:,0], result_50[:,1], label='n=50')
        plt.plot(result_100[:,0], result_100[:,1], label='n=100')
        plt.grid()
        plt.legend()
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.axis('equal')
        plt.title('sample window, nd-data')
        if sys.argv[1] != 'all':
            plt.show()

        r1['nd'] = result_1
        r10['nd'] = result_10
        r50['nd'] = result_50
        r100['nd'] = result_100

    if sys.argv[1] == 'nd-time' or sys.argv[1] == 'all':
        # test the sample window filter for n-d time data
        sw_filter = filters.window.TimeAverage(10)

        r = 1.e3
        theta = numpy.linspace(-numpy.pi, numpy.pi, 100)
        time = numpy.arange(1, 101)

        X = numpy.vstack((
            r * numpy.cos(theta),
            r * numpy.sin(theta),
            numpy.zeros(theta.shape),
            numpy.arange(1, theta.shape[0] + 1))).T

        result_1 = []
        result_10 = []
        result_50 = []
        result_100 = []

        for x,t in zip(X, time):
            sw_filter.add(x, t)
            result_1.append(sw_filter.mean(t_avg=1))
            result_10.append(sw_filter.mean())
            result_50.append(sw_filter.mean(t_avg=50))
            result_100.append(sw_filter.mean(t_avg=100))

        result_1 = numpy.array(result_1)
        result_10 = numpy.array(result_10)
        result_50 = numpy.array(result_50)
        result_100 = numpy.array(result_100)

        f, axs = plt.subplots(result_1.shape[1], 1, sharex=True)

        for i, ax in enumerate(axs):
            ax.plot(result_1[:,i], label='t=1')
            ax.plot(result_10[:,i], label='t=10')
            ax.plot(result_50[:,i], label='t=50')
            ax.plot(result_100[:,i], label='t=100')
            ax.grid()
            ax.set_ylabel('window average')
        ax.legend()
        ax.set_xlabel('sample')
        plt.title('time window, nd-data')

        plt.figure()
        plt.plot(result_1[:,0], result_1[:,1], label='t=1')
        plt.plot(result_10[:,0], result_10[:,1], label='t=10')
        plt.plot(result_50[:,0], result_50[:,1], label='t=50')
        plt.plot(result_100[:,0], result_100[:,1], label='t=100')
        plt.grid()
        plt.legend()
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.axis('equal')
        plt.title('time window, nd-data')
        if sys.argv[1] != 'all':
            plt.show()

        r1['nd-time'] = result_1
        r10['nd-time'] = result_10
        r50['nd-time'] = result_50
        r100['nd-time'] = result_100

    if sys.argv[1] == 'geo' or sys.argv[1] == 'all':
        # test the sample window filter for n-d data
        sw_filter = filters.window.GeodesicAverage(10)

        r = 1.e3
        theta = numpy.linspace(-numpy.pi, numpy.pi, 100)
        lla_ref = numpy.deg2rad(
            numpy.array([40.010065, -105.245080, 0.0], ndmin=2))

        X = numpy.vstack((
            r * numpy.cos(theta),
            r * numpy.sin(theta),
            numpy.zeros(theta.shape),
            numpy.arange(1, theta.shape[0] + 1))).T
        X[:,:3] = geodesy.conversions.enu_to_lla(X[:,:3], lla_ref)

        result_1 = []
        result_10 = []
        result_50 = []
        result_100 = []

        for x in X:
            sw_filter.add(x)
            result_1.append(sw_filter.mean(n=1))
            result_10.append(sw_filter.mean())
            result_50.append(sw_filter.mean(n=50))
            result_100.append(sw_filter.mean(n=100))

        result_1 = numpy.array(result_1)
        result_10 = numpy.array(result_10)
        result_50 = numpy.array(result_50)
        result_100 = numpy.array(result_100)
        result_1[:,:3] = geodesy.conversions.lla_to_enu(result_1[:,:3], lla_ref)
        result_10[:,:3] = geodesy.conversions.lla_to_enu(
            result_10[:,:3], lla_ref)
        result_50[:,:3] = geodesy.conversions.lla_to_enu(
            result_50[:,:3], lla_ref)
        result_100[:,:3] = geodesy.conversions.lla_to_enu(
            result_100[:,:3], lla_ref)

        f, axs = plt.subplots(result_1.shape[1], 1, sharex=True)

        for i, ax in enumerate(axs):
            ax.plot(result_1[:,i], label='n=1')
            ax.plot(result_10[:,i], label='n=10')
            ax.plot(result_50[:,i], label='n=50')
            ax.plot(result_100[:,i], label='n=100')
            ax.grid()
            ax.set_ylabel('window average')
        ax.legend()
        ax.set_xlabel('sample')
        plt.title('sample window, geo data')

        plt.figure()
        plt.plot(result_1[:,0], result_1[:,1], label='n=1')
        plt.plot(result_10[:,0], result_10[:,1], label='n=10')
        plt.plot(result_50[:,0], result_50[:,1], label='n=50')
        plt.plot(result_100[:,0], result_100[:,1], label='n=100')
        plt.grid()
        plt.legend()
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.axis('equal')
        plt.title('sample window, geo data')
        if sys.argv[1] != 'all':
            plt.show()

        r1['geo'] = result_1
        r10['geo'] = result_10
        r50['geo'] = result_50
        r100['geo'] = result_100

    if sys.argv[1] == 'geo-time' or sys.argv[1] == 'all':
        # test the sample window filter for n-d time data
        sw_filter = filters.window.GeodesicTimeAverage(10)
        time = numpy.arange(1, 101)

        r = 1.e3
        theta = numpy.linspace(-numpy.pi, numpy.pi, 100)
        lla_ref = numpy.deg2rad(
            numpy.array([40.010065, -105.245080, 0.0], ndmin=2))

        X = numpy.vstack((
            r * numpy.cos(theta),
            r * numpy.sin(theta),
            numpy.zeros(theta.shape),
            numpy.arange(1, theta.shape[0] + 1))).T
        X[:,:3] = geodesy.conversions.enu_to_lla(X[:,:3], lla_ref)

        result_1 = []
        result_10 = []
        result_50 = []
        result_100 = []

        for x, t in zip(X, time):
            sw_filter.add(x, t)
            result_1.append(sw_filter.mean(t_avg=1))
            result_10.append(sw_filter.mean())
            result_50.append(sw_filter.mean(t_avg=50))
            result_100.append(sw_filter.mean(t_avg=100))

        result_1 = numpy.array(result_1)
        result_10 = numpy.array(result_10)
        result_50 = numpy.array(result_50)
        result_100 = numpy.array(result_100)
        result_1[:,:3] = geodesy.conversions.lla_to_enu(result_1[:,:3], lla_ref)
        result_10[:,:3] = geodesy.conversions.lla_to_enu(
            result_10[:,:3], lla_ref)
        result_50[:,:3] = geodesy.conversions.lla_to_enu(
            result_50[:,:3], lla_ref)
        result_100[:,:3] = geodesy.conversions.lla_to_enu(
            result_100[:,:3], lla_ref)

        f, axs = plt.subplots(result_1.shape[1], 1, sharex=True)

        for i, ax in enumerate(axs):
            ax.plot(result_1[:,i], label='t=1')
            ax.plot(result_10[:,i], label='t=10')
            ax.plot(result_50[:,i], label='t=50')
            ax.plot(result_100[:,i], label='t=100')
            ax.grid()
            ax.set_ylabel('window average')
        ax.legend()
        ax.set_xlabel('sample')
        plt.title('time window, geo data')

        plt.figure()
        plt.plot(result_1[:,0], result_1[:,1], label='t=1')
        plt.plot(result_10[:,0], result_10[:,1], label='t=10')
        plt.plot(result_50[:,0], result_50[:,1], label='t=50')
        plt.plot(result_100[:,0], result_100[:,1], label='t=100')
        plt.grid()
        plt.legend()
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.axis('equal')
        plt.title('time window, geo data')
        if sys.argv[1] != 'all':
            plt.show()

        r1['geo-time'] = result_1
        r10['geo-time'] = result_10
        r50['geo-time'] = result_50
        r100['geo-time'] = result_100

if sys.argv[1] == 'all':
    R = {1: r1, 10: r10, 50: r50, 100: r100}
    err = {1: {}, 10: {}, 50: {}, 100: {}}
    position_err = {1: {}, 10: {}, 50: {}, 100: {}}
    max_err = -numpy.inf
    max_position_err = -numpy.inf
    for key_n, r in R.items():
        for key, val in r.items():
            err[key_n][key] = val[:,-1] - r['sample'][:,-1]
            max_err = max(max_err, numpy.amax(numpy.abs(err[key_n][key])))
            if val.shape[1] > 1:
                position_err[key_n][key] = val[:,:3] - r['nd'][:,:3]
                max_position_err= max(
                    max_position_err,
                    numpy.amax(numpy.abs(position_err[key_n][key])))

    print('max_error: {}\nmax_position_err: {}'.format(
        max_err, max_position_err))
    plt.show()
