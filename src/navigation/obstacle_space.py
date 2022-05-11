import numpy
import pdb

import scipy.spatial
import shapely.geometry

import geodesy.conversions
import geometry.lines
import geometry.helpers
import navigation.obstacle_primitives

class FlatObstacle(navigation.obstacle_primitives.ShapePrimitive):
    """A class for 2d obstacles

    This is really just a convenience class to make it explicit that the object
    is an obstacle. It is a straight inheritance of
    navigation.obstacle_primitives.ShapePrimitive
    """
    def __init__(self, shape=None, definition=None):
        """Constructor

        Arguments:
            shape: shapely LinearRing, Polygon or numpy (n,2) array defining
                the perimeter of the obstacle in a local North/East coordinate
                frame. optional, if unspecified then definition will be expected
            definition: optional, a dictionary (likely from a yaml file) with
                the perimeter

        Returns:
            class instance
        """
        super(FlatObstacle, self).__init__(shape, definition)

class PrismaticObstacle(navigation.obstacle_primitives.PrismaticShape):
    """A class to hold a prismatic obstacle (2-D with depth)

    This is really just a convenience class to make it explicit that the object
    is an obstacle. It is a straight inheritance of
    navigation.obstacle_primitives.PrismaticShape
    """
    def __init__(self, shape=None, z0=None, zt=None, definition=None):
        """Constructor

        Arguments:
            shape: shapely LinearRing, Polygon or numpy (n,2) array defining
                the perimeter of the obstacle in a local North/East coordinate
                frame. optional, if unspecified then definition will be expected
            z0: Down position of the base of the obstacle. optional, defaults
                to an obstacle of infinite extent
            zt: Down position of the top of the obstacle. optional, defaults to
                an obstacle of infinite extent
            definition: optional, a dictionary (likely from a yaml file) with
                the perimeter, z0, zt values

        Returns:
            class instance
        """
        super(PrismaticObstacle, self).__init__(shape, z0, zt, definition)

class ObstacleSpace(object):
    """A class for an obstacle space in 2-d

    The obstacle space is a collection of obstacles and some methods to check
    collisions and proximity.
    """
    def __init__(self):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        self._obstacles = []

    def is_point_obstructed(self, point):
        """ Check if a point is inside an obstacle

        Arguments:
            point: shapely Point object or 3, element numpy array specifying
                a point in ned coordinates

        Returns:
            is_inside: bool specifying if the point is within the shape
        """
        return numpy.any([o.is_point_inside(point) for o in self._obstacles])

    def nearest_exterior(self, point):
        """ Compute a vector from a point to the nearest boundary of an obstacle

        Arguments:
            point: shapely Point object or 3, numpy array defining the ned
                position of a point

        Returns:
            vector_to_boundary: 1x3 numpy array ned vector to boundary
        """
        # compute a vector to the nearest exterior point on every obstacle and
        # then return the shortest vector
        vectors_to_boundary = numpy.vstack(
            [o.nearest_exterior(point) for o in self._obstacles])
        R = numpy.linalg.norm(vectors_to_boundary, axis=1)
        return vectors_to_boundary[numpy.argmin(R):numpy.argmin(R)+1]

    def nearest_hole(self, point):
        """ Compute a vector from a point to the nearest interior boundary.

        Arguments:
            point: shapely Point object or 3, numpy array defining the ned
                position of a point

        Returns:
            vector_to_boundary: 1x3 numpy array ned vector to interior boundary
        """
        # compute a vector to the nearest hole point on every obstacle and
        # then return the shortest vector
        vectors_to_boundary = numpy.vstack(
            [o.nearest_hole(point) for o in self._obstacles])
        R = numpy.linalg.norm(vectors_to_boundary, axis=1)
        return vectors_to_boundary[numpy.argmin(R):numpy.argmin(R)+1]

    def nearest_boundary(self, point):
        """ Compute a vector from a point to the nearest boundary

        determines vector to the nearest boundary whether hole or exterior

        Arguments:
            point: shapely Point object or numpy array defining the ned
                position of a point

        Returns:
            vector_to_boundary: 1x3 numpy array ned vector to boundary
        """
        # find the nearest exterior and the nearest hole point, return the
        # closest one
        r_exterior = self.nearest_exterior(point)
        r_hole = self.nearest_hole(point)
        if numpy.linalg.norm(r_hole) < numpy.linalg.norm(r_exterior):
            return r_hole
        return r_exterior

    def exterior_intersections(self, path):
        """ Finds all of the points where a path intersects an obstacle.

        Arguments:
            path: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the path

        Returns:
            intersections: tuple of intersection points between the given
                path and the exterior boundary
        """
        intersections = []
        for obstacle in self._obstacles:
            intersections += list(obstacle.exterior_intersections(path))
        return tuple(intersections)

    def hole_intersections(self, path):
        """ Find intersections between a path and holes in the shape.

        Arguments:
            path: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the path

        Returns:
            intersections: tuple of tuples: one tuple each for each hole which
                contains the intersection points between the given path
                and the hole
        """
        intersections = []
        for obstacle in self._obstacles:
            intersections += list(obstacle.hole_intersections(path))
        return tuple(intersections)

    def intersections(self, path):
        """ Finds all intersections between a path and the boundaries.

        finds all of the points where a path intersects any edge of the shape
        whether a boundary or a hole

        Arguments:
            path: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the path

        Returns:
            intersections: tuple of intersection points between the given
                path and the shape
        """
        if type(path) is numpy.ndarray:
            path = navigation.obstacle_primitives._numpy_to_linestring(path)
        intersections = list(self.exterior_intersections(path))
        intersections += list(i for i in self.hole_intersections(path))
        return tuple(intersections)

    @property
    def patches(self):
        """Get patches in NED coordinates to let us plot this obstacle space

        Arguments:
            no arguments:

        Returns:
            list of patches that can be plotted
        """
        patches = []
        for o in self._obstacles:
            p = numpy.array(o.exterior)

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

            X = numpy.stack((x, y, z)).T
            h = scipy.spatial.ConvexHull(X)
            patches.append((x, y, z, h.simplices))

        return patches

class FlatObstacleSpace(ObstacleSpace):
    """A 2-d (Flat) obstacle space

    This inherits almost everything from the ObstacleSpace for computations, it
    serves the correct obstacle definition to the obstacle construction.
    """
    def __init__(self, obstacle_definitions, ref_pt=None, is_radians=True):
        """Constructor

        Arguments:
            obstacle_definitions: dictionaries defining obstacles, each
                dictionary should match that required for FlatObstacle. Can be a
                list of definitions or a dictionary where each entry is the
                definition keyed to the obstacle id
            ref_pt: optional numpy (1,3) array specifying lat/long/alt origin of
                space. If this is not specified then obstacle_definitions is
                assumed to be NED. if it is specified then obstacle_definitions
                is intepreted as lat/long/alt points and converted to NED before
                constructing the obstacle space
            is_radians: optional flag indicating if the obstacle space
                definition is in degrees or radians lat/lon. Defaults to radians

        Returns:
            class instance
        """
        # declare a lambda function to handle the deg/rad conversion if we need
        if not is_radians and ref_pt is not None:
            shape_to_numpy = lambda s: numpy.deg2rad(numpy.array(s))
            ref_pt[0,0:2] = numpy.deg2rad(ref_pt[0,0:2])
        else:
            shape_to_numpy = lambda s: numpy.array(s)

        super(FlatObstacleSpace, self).__init__()
        obstacles = []

        # we have two definition cases. one is a dictionary, in that case we'll
        # take the obstacle name from the dictionary key
        if isinstance(obstacle_definitions, dict):
            for key, val in obstacle_definitions.items():
                obstacle_definition = {}
                if ref_pt is not None:
                    ned_vertices = geodesy.conversions.lla_to_ned(
                        shape_to_numpy(val['shape']), ref_pt)
                else:
                    ned_vertices = shape_to_numpy(val['shape'])

                obstacle_definition['shape'] = ned_vertices
                obstacle_definition['id'] = key

                self._obstacles.append(
                    PrismaticObstacle(definition=obstacle_definition))
            return

        # otherwise its a list and we'll let the obstacle name itself
        for val in obstacle_definitions:
            obstacle_definition = {}
            if ref_pt is not None:
                ned_vertices = geodesy.conversions.lla_to_ned(
                    shape_to_numpy(val['shape']), ref_pt)
            else:
                ned_vertices = shape_to_numpy(val['shape'])

            obstacle_definition['shape'] = ned_vertices

            self._obstacles.append(
                PrismaticObstacle(definition=obstacle_definition))

        self._obstacles = [FlatObstacle(o) for o in obstacles]

class PrismaticObstacleSpace(ObstacleSpace):
    """A 3d obstacle space composed of prismatic obstacles

    This inherits almost everything from the ObstacleSpace for computations, it
    serves the correct obstacle definition to the obstacle construction.
    """
    def __init__(self, obstacle_definitions, ref_pt=None, is_radians=True):
        """Constructor

        Arguments:
            obstacle_definitions: dictionaries defining obstacles, each
                dictionary should match that required for PrismaticObstacle. Can
                be a list of definitions or a dictionary where each entry is the
                definition keyed to the obstacle id
            ref_pt: optional numpy (1,3) array specifying lat/long/alt origin of
                space. If this is not specified then obstacle_definitions is
                assumed to be NED. if it is specified then obstacle_definitions
                is intepreted as lat/long/alt points and converted to NED before
                constructing the obstacle space
            is_radians: optional flag indicating if the obstacle space
                definition is in degrees or radians lat/lon. Defaults to radians

        Returns:
            class instance
        """
        # declare a lambda function to handle the deg/rad conversion if we need
        if ref_pt is not None:
            if not is_radians:
                shape_to_numpy = lambda s: numpy.deg2rad(numpy.array(s))
                ref_pt[0,0:2] = numpy.deg2rad(ref_pt[0,0:2])
            else:
                shape_to_numpy = lambda s: numpy.array(s)
        else:
            ref_pt = numpy.zeros((1,3))

        super(PrismaticObstacleSpace, self).__init__()
        obstacles = []

        # we have two definition cases. one is a dictionary, in that case we'll
        # take the obstacle name from the dictionary key
        if isinstance(obstacle_definitions, dict):
            for key, val in obstacle_definitions.items():
                obstacle_definition = {}
                if ref_pt is not None:
                    ned_vertices = geodesy.conversions.lla_to_ned(
                        shape_to_numpy(val['shape']), ref_pt)
                else:
                    ned_vertices = shape_to_numpy(val['shape'])

                if 'base_alt' in val:
                    z0 = -(val['base_alt'] - ref_pt[0,2])
                else:
                    z0 = 0.0
                zt = -(val['alt'] - ref_pt[0,2])

                obstacle_definition['shape'] = ned_vertices
                obstacle_definition['z0'] = z0
                obstacle_definition['zt'] = zt
                obstacle_definition['id'] = key

                self._obstacles.append(
                    PrismaticObstacle(definition=obstacle_definition))
            return

        # otherwise its a list and we'll let the obstacle name itself
        for val in obstacle_definitions:
            obstacle_definition = {}
            if ref_pt is not None:
                ned_vertices = geodesy.conversions.lla_to_ned(
                    shape_to_numpy(val['shape']), ref_pt)
            else:
                ned_vertices = shape_to_numpy(val['shape'])

            if 'base_alt' in val:
                z0 = -(val['base_alt'] - ref_pt[0,2])
            else:
                z0 = 0.0
            zt = -(val['alt'] - ref_pt[0,2])

            obstacle_definition['shape'] = ned_vertices
            obstacle_definition['z0'] = z0
            obstacle_definition['zt'] = zt

            self._obstacles.append(
                PrismaticObstacle(definition=obstacle_definition))

class GeographicObstacleSpace(ObstacleSpace):
    """A class for an flat obstacle space which is somewhere on earth

    It puts some geodesic conversions on the point and path handling so that
    it can reference an obstacle space in a local NED coordinate frame. This is
    not a true geodesic computation so it should be used only over spaces where
    a local NED frame is a good approximation
    """
    def __init__(self, lla_ref):
        """Constructor

        Arguments:
            lla_ref: numpy (1,3) array giving lat/long/alt of reference point

        Returns:
            class instance
        """
        self._lla_ref = lla_ref
        self._obstacles = []

    def point_lla_to_ned(self, lla_point, shapely_return=True):
        """Convert a lla point to a ned

        Arguments:
            lla_point: shapely Point object or 3, element numpy array specifying
                a point in lla coordinates
            shapely_return: flag to indicate if the return should be a shapely
                Point object

        Returns:
            ned_point: the point in ned coordinates
        """
        if isinstance(lla_point, shapely.geometry.Point):
            lla_point = numpy.array(lla_point, ndmin=2)
        ned_point = geodesy.conversions.lla_to_ned(lla_point, self._lla_ref)[0]
        if shapely_return:
            ned_point = shapely.geometry.Point(ned_point)
        return ned_point

    def point_ned_to_lla(self, ned_point, shapely_return=True):
        """Convert a ned point to lla

        Arguments:
            ned_point: shapely Point object or 3, element numpy array
            shapely_return: flag to indicate if the return should be a shapely
                Point object

        Returns:
            lla_point: the point in lla coordinates
        """
        if isinstance(ned_point, shapely.geometry.Point):
            ned_point = numpy.array(ned_point, ndmin=2)
        lla_point = geodesy.conversions.ned_to_lla(ned_point, self._lla_ref)[0]
        if shapely_return:
            lla_point = shapely.geometry.Point(lla_point)
        return lla_point

    def path_lla_to_ned(self, lla_path, shapely_return=True):
        """Convert an lla path to ned

        Arguments:
            lla_path: shapely LineString object or (n,3) numpy array specifying
                a path in lla coordinates
            shapely_return: flag to indicate if the return should be a shapely
                LineString object

        Returns:
            ned_path: the path in ned coordinates
        """
        if isinstance(lla_path, shapely.geometry.LineString):
            lla_path = numpy.array(lla_path)
        ned_path = geodesy.conversions.lla_to_ned(lla_path, self._lla_ref)
        if shapely_return:
            ned_path = shapely.geometry.LineString(ned_path)
        return ned_path

    def path_ned_to_lla(self, ned_path, shapely_return=True):
        """Convert an ned path to lla

        Arguments:
            ned_path: shapely LineString object or (n,3) numpy array specifying
                a path in ned coordinates
            shapely_return: flag to indicate if the return should be a shapely
                LineString object

        Returns:
            lla_path: the path in lla coordinates
        """
        if isinstance(ned_point, shapely.geometry.LineString):
            ned_path = numpy.array(ned_path)
        lla_path = geodesy.conversions.ned_to_lla(ned_path, self._lla_ref)
        if shapely_return:
            lla_path = shapely.geometry.LineString(lla_path)
        return lla_path

    def is_point_obstructed(self, point):
        """ Check if a point is inside an obstacle

        Arguments:
            point: shapely Point object or 3, element numpy array specifying
                a point in ned coordinates

        Returns:
            is_inside: bool specifying if the point is within the shape
        """
        return super(GeographicObstacleSpace, self).is_point_obstructed(
            self.point_lla_to_ned(point))

    def nearest_exterior(self, point):
        """ Compute a vector from a point to the nearest boundary of an obstacle

        Arguments:
            point: shapely Point object or 3, numpy array defining the ned
                position of a point

        Returns:
            vector_to_boundary: 1x3 numpy array ned vector to boundary
        """
        return super(GeographicObstacleSpace, self).nearest_exterior(
            self.point_lla_to_ned(point))

    def nearest_hole(self, point):
        """ Compute a vector from a point to the nearest interior boundary.

        Arguments:
            point: shapely Point object or 3, numpy array defining the ned
                position of a point

        Returns:
            vector_to_boundary: 1x3 numpy array ned vector to interior boundary
        """
        return super(GeographicObstacleSpace, self).nearest_hole(
            self.point_lla_to_ned(point))

    def exterior_intersections(self, path):
        """ Finds all of the points where a path intersects an obstacle.

        Arguments:
            path: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the path

        Returns:
            intersections: tuple of intersection points between the given
                path and the exterior boundary
        """
        intersections = tuple(
            [self.point_ned_to_lla(p) for p in
                super(GeographicObstacleSpace, self).exterior_intersections(
                    self.path_lla_to_ned(path))])
        return intersections

    def hole_intersections(self, path):
        """ Find intersections between a path and holes in the shape.

        Arguments:
            path: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the path

        Returns:
            intersections: tuple of tuples: one tuple each for each hole which
                contains the intersection points between the given path
                and the hole
        """
        intersections = tuple(
            [self.point_ned_to_lla(p) for p in
                super(GeographicObstacleSpace, self).hole_intersections(
                    self.path_lla_to_ned(path))])
        return intersections

class FlatGeographicObstacleSpace(GeographicObstacleSpace):
    """A 2-d obstacle space located somewhere on earth"""
    def __init__(self, obstacle_definitions, ref_pt, is_radians=True):
        """Constructor

        Arguments:
            definition: dictionaries defining obstacles, the dictionary should
                match that required for FlatObstacle. Can be a list of
                definitions or a dictionary where each entry is the definition
                keyed to the obstacle id
            ref_pt: numpy (1,3) array specifying lat/long/alt origin of space
            is_radians: optional flag indicating if the obstacle space
                definition is in degrees or radians lat/lon. Defaults to radians
                The points to check must *ALWAYS* be in radians regardless of
                this flag.

        Returns:
            class instance
        """
        # declare a lambda function to handle the deg/rad conversion if we need
        if not is_radians:
            shape_to_numpy = lambda s: numpy.deg2rad(numpy.array(s))
            ref_pt[0,0:2] = numpy.deg2rad(ref_pt[0,0:2])
        else:
            shape_to_numpy = lambda s: numpy.array(s)

        super(FlatGeographicObstacleSpace, self).__init__(ref_pt)

        self._obstacles = []

        # we have two definition cases. one is a dictionary, in that case we'll
        # take the obstacle name from the dictionary key
        if isinstance(obstacle_definitions, dict):
            for key, val in obstacle_definitions.items():
                obstacle_definition = {}
                ned_vertices = geodesy.conversions.lla_to_ned(
                    shape_to_numpy(val['shape']), self._lla_ref)
                obstacle_definition['shape'] = ned_vertices
                obstacle_definition['id'] = key
                self._obstacles.append(
                    FlatObstacle(definition=obstacle_definition))
            return

        # otherwise it's a list and we'll let the obstacle name itself
        for val in obstacle_definitions:
            obstacle_definition = {}
            ned_vertices = geodesy.conversions.lla_to_ned(
                shape_to_numpy(val['shape']), self._lla_ref)
            obstacle_definition['shape'] = ned_vertices
            self._obstacles.append(FlatObstacle(definition=obstacle_definition))

class PrismaticGeographicObstacleSpace(GeographicObstacleSpace):
    """A prismatic obstacle space located somewhere on earth"""
    def __init__(self, obstacle_definitions, ref_pt, is_radians=True):
        """Constructor

        Arguments:
            definition: dictionaries defining obstacles, the dictionary should
                match that required for PrismaticObstacle. Can be a list of
                definitions or a dictionary where each entry is the definition
                keyed to the obstacle id
            ref_pt: numpy (1,3) array specifying lat/long/alt origin of space
            is_radians: optional flag indicating if the obstacle space
                definition is in degrees or radians lat/lon. Defaults to radians
                The points to check must *ALWAYS* be in radians regardless of
                this flag.

        Returns:
            class instance
        """
        # declare a lambda function to handle the deg/rad conversion if we need
        if not is_radians:
            shape_to_numpy = lambda s: numpy.deg2rad(numpy.array(s))
            ref_pt[0,0:2] = numpy.deg2rad(ref_pt[0,0:2])
        else:
            shape_to_numpy = lambda s: numpy.array(s)

        super(PrismaticGeographicObstacleSpace, self).__init__(ref_pt)

        self._obstacles = []

        # we have two definition cases. one is a dictionary, in that case we'll
        # take the obstacle name from the dictionary key
        if isinstance(obstacle_definitions, dict):
            for key, val in obstacle_definitions.items():
                obstacle_definition = {}
                ned_vertices = geodesy.conversions.lla_to_ned(
                    shape_to_numpy(val['shape']), self._lla_ref)
                if 'base_alt' in val:
                    z0 = -(val['base_alt'] - self._lla_ref[0,2])
                else:
                    z0 = 0.0
                zt = -(val['alt'] - self._lla_ref[0,2])

                obstacle_definition['shape'] = ned_vertices
                obstacle_definition['z0'] = z0
                obstacle_definition['zt'] = zt
                obstacle_definition['id'] = key
                self._obstacles.append(
                    PrismaticObstacle(definition=obstacle_definition))
            return

        # otherwise its a list and we'll let the obstacle name itself
        for val in obstacle_definitions:
            obstacle_definition = {}
            ned_vertices = geodesy.conversions.lla_to_ned(
                shape_to_numpy(val['shape']), self._lla_ref)
            if 'base_alt' in val:
                z0 = -(val['base_alt'] - self._lla_ref[0,2])
            else:
                z0 = 0.0
            zt = -(val['alt'] - self._lla_ref[0,2])

            obstacle_definition['shape'] = ned_vertices
            obstacle_definition['z0'] = z0
            obstacle_definition['zt'] = zt
            self._obstacles.append(
                PrismaticObstacle(definition=obstacle_definition))

