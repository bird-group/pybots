import numpy

import os

import magpy
import magpy_backend

def magnetic_field(lla, datetime):
    """A wrapper method for the magpy function. Returns magnetic field

    Arguments:
        lla: lat/long/alt (relative to wgs84 ellipsoid)
        datetime: time as a datetime instance

    Returns:
        B: magnetic field vector (nT)
            N: north component
            E: east component
            D: down component
    """
    lat = numpy.rad2deg(lla[0])
    lon = numpy.rad2deg(lla[1])
    alt = lla[2] / 1000.0

    cof_path = os.path.join(
        os.path.split(magpy.__file__)[0], 'WMM.COF')
    if not os.path.isfile(cof_path):
        print('{} not found'.format(cof_path))

    B = magpy_backend.magpy_backend(
        cof_path, lat, lon, alt, datetime.year, datetime.month, datetime.day)
    return numpy.array(B)

