#from distutils.core import setup, Extension
from setuptools import setup, Extension

extension_mod = Extension(
    "magpy_backend",
    [
        "src/magpy/magpy_backend.c",
        "src/magpy/GeomagnetismLibrary.c",
        #"src/magpy/GeomagnetismHeader.h",
        #"src/magpy/EGM9615.h",
        #"src/magpy/WMM.COF",
    ])
setup(
    name = "pybots",
    packages = [
        'approximators',
        'audio',
        'common_packets',
        'communications',
        'device_drivers',
        'environments',
        'filters',
        'geodesy',
        'geometry',
        'graphics',
        'helpers',
        'magpy',
        'parsers',
        'navigation',
        'path_planning',
        'robot_control',
        'simulation',
        'units',
    ],
    ext_modules = [extension_mod],
    package_dir = {'': 'src'},
    include_package_data=True,
    package_data = {'magpy': ['WMM.COF']}
)
