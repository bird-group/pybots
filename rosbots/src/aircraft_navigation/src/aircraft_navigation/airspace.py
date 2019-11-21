# -*- coding: utf-8 -*-
"""
Created on Tue Jan 20 22:25:51 2015

@author: birdman
"""
import numpy

import shapely.geometry
import spherical_geometry.polygon
import spherical_geometry.vector
import spherical_geometry.great_circle_arc

import rospy

import geometry.lines
import geometry.helpers

import pdb

class FlatEarthAirspace(object):
    """ Class for handling airspace.

    a class for keeping track of an airspace region and computing
    interesting things about it (like whether or not we're in it)

    12 January 2015, John Bird
    """

    def __init__(self, shape, ref_pt):
        """ Constructor for the Airspace class.

        Airspace is defined by a shapely polygon with vertices given in meters
        north/east/down relative to a defined reference lat/lon/alt location. At
        the moment the airspace is 2-d only (ie it has no ceiling/floor).

        Arguments:
            shape: shapely Polygon object defining the airspace area in meters
            ref_pt: numpy 1x3 array of lat/lon/alt defining where the polygon is
                referenced to

        Returns:
            Airspace instance

        12 January 2015, John Bird
        """
        assert (type(shape) is shapely.geometry.Polygon or
            type(shape) is shapely.geometry.MultiPolygon),\
            "airspace geometry must be a shapely polygon object"
        assert type(ref_pt) is numpy.ndarray,\
            "reference point must be a numpy array"
        assert ref_pt.shape == (1, 3), "reference point must be a 1x3 array"

        self.shape = shape
        self.ref_pt = ref_pt

    def is_point_inside(self, waypoint):
        """ Check if a point is inside the airspace.

        determine if a point lies within the airspace region (inside of its
        borders and outside of any holes in it)

        Arguments:
            waypoint: shapely Point object or 3, element numpy array specifying
                a point in ned coordinates

        Returns:
            is_legal: bool specifying if the point is within the airspace

        12 January 2015, John Bird
        """
        if type(waypoint) is numpy.ndarray:
            waypoint = shapely.geometry.Point(waypoint)
        return waypoint.within(self.shape)

    def is_path_inside(self, trajectory):
        """ Determines if a flight path lies entirely within an airspace area.

        Arguments:
            trajectory: shapely LineString object or nx3 numpy array defining
                a flight segment

        Returns:
            is_legal: bool specifying if the flight segment lies within the
                airspace

        12 January 2015, John Bird
        """
        if type(trajectory) is numpy.ndarray:
            trajectory = self._numpy_to_linestring(trajectory)
        return trajectory.within(self.shape)

    def nearest_exterior(self, waypoint):
        """ Compute a vector from a point to the nearest exterior boundary.

        Arguments:
            waypoint: shapely Point object or 3, numpy array defining the ned
                position of a point

        Returns:
            boundary_vect: 1x3 numpy array of ned distance to boundary in meters

        12 January 2015, John Bird
        """
        if type(waypoint) is numpy.ndarray:
            waypoint = shapely.geometry.Point(waypoint)
        return geometry.lines.point_line_distance(
            waypoint, self.shape.exterior)[0]

    def nearest_hole(self, waypoint):
        """ Compute a vector from a point to the nearest interior boundary.

        determines a vector to the nearest point on a hole in the airspace

        Arguments:
            waypoint: shapely Point object or 3, numpy array defining the ned
                position of a point

        Returns:
            boundary_vect: numpy array of ned distance to hole in meters

        12 January 2015, John Bird
        """
        if type(waypoint) is numpy.ndarray:
            waypoint = shapely.geometry.Point(waypoint)
        boundary_vect = numpy.ones((1, 3)) * numpy.inf
        for shape in self.shape.interiors:
            dist = geometry.lines.point_line_distance(waypoint, shape)[0]
	    #pdb.set_trace()
            if numpy.linalg.norm(dist) < numpy.linalg.norm(boundary_vect):
                boundary_vect = dist
        return boundary_vect

    def nearest_boundary(self, waypoint):
        """ Compute a vector from a point to the nearest boundary

        determines vector to the nearest boundary whether hole or exterior

        Arguments:
            waypoint: shapely Point object or numpy array defining the ned
                position of a point

        Returns:
            boundary_vect: numpy array of ned distance to boundary in meters

        12 January 2015, John Bird
        """
        r_exterior = self.nearest_exterior(waypoint)
        r_hole = self.nearest_hole(waypoint)
        if numpy.linalg.norm(r_hole) < numpy.linalg.norm(r_exterior):
            return r_hole
        return r_exterior

    def exterior_intersection(self, trajectory):
        """ Finds all of the points where a path intersects the boundary.

        Arguments:
            trajectory: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the trajectory

        Returns:
            intersections: tuple of intersection points between the given
                trajectory and the airspace

        12 January 2015, John Bird
        """
        if type(trajectory) is numpy.ndarray:
            trajectory = self._numpy_to_linestring(trajectory)
        return tuple(
            i for i in trajectory.intersection(self.shape.exterior))

    def hole_intersection(self, trajectory):
        """ Find intersections between a path and holes in the airspace.

        finds all of the points where a path intersects holes in the airspace

        Arguments:
            trajectory: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the trajectory

        Returns:
            intersections: tuple of tuples: one tuple each for each hole which
                conttains the intersection points between the given trajectory
                and the hole

        12 January 2015, John Bird
        """
        if type(trajectory) is numpy.ndarray:
            trajectory = self._numpy_to_linestring(trajectory)
        intersections = []
        for hole in self.shape.interiors:
            this_hole_intersections = trajectory.intersection(hole)
            if type(this_hole_intersections) is shapely.geometry.Point:
                this_hole_intersections = (this_hole_intersections,)
            if type(this_hole_intersections) is shapely.geometry.LineString:
                # this seems to happen when I instersect on a vertex...
                this_intersection = numpy.array(this_hole_intersections)
                # test if that is the case...save debugging info if not
                err = False
                if len(this_intersection) > 2:
                    err = True
                if (numpy.linalg.norm(this_intersection[0] - this_intersection[1])
                    > 1.0):
                    err = True
                # if I have more than one point or they are not very close
                # together, then something bad happened and I'll save some
                # debugging information...otherwise just ship off a point...
                if err is True:
                    numpy.save('/home/avia/trajectory_dump', trajectory)
                    numpy.save('/home/avia/airspace_dump', self.shape)
                    numpy.save('/home/avia/hole_dump', hole)
                    numpy.save('/home/avia/intersection_dump', this_hole_intersections)
                    continue
                else:
                    this_hole_intersections = (
                        shapely.geometry.Point(this_intersection[0]),)
            intersections.append(tuple(pt for pt in this_hole_intersections))
        return tuple(intersections)

    def any_intersection(self, trajectory):
        """ Finds all intersections between a trajectory and the boundaries.

        finds all of the points where a path intersects any edge of the airspace
        whether a boundary or a hole

        Arguments:
            trajectory: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the trajectory

        Returns:
            intersections: tuple of intersection points between the given
                trajectory and the airspace

        12 January 2015, John Bird
        """
        if type(trajectory) is numpy.ndarray:
            trajectory = self._numpy_to_linestring(trajectory)
        intersections = list(self.exterior_intersection(trajectory))
        intersections.append(list(
            i for i in self.hole_intersection(trajectory)))
        return tuple(intersections)

    def _numpy_to_linestring(self, numpy_array):
        """Convert a number array to a linestring

        Arguments:
            numpy_array: an (n,3) numpy array to convert into a shapely
                linestring instance

        Returns:
            linestring: the shapely linestring with vertices at the points
                given in numpy_array
        """
        trajectory = shapely.geometry.LineString(
            tuple(tuple(i) for i in trajectory))

class SphericalEarthAirspace(object):
    """ Class for handling airspace on a spherical earth

    a class for keeping track of an airspace region and computing
    interesting things about it (like whether or not we're in it)

    6 October 2016, John Bird
    """
    def __init__(self, shape):
        """ Constructor for the Airspace class.

        SphericalEarthAirspace is defined by a spherical_geometry polygon with
        vertices given as lat/lon points. At the moment it does not consider
        altitude floors or ceilints

        Arguments:
            shape: spherical_geometry SphericalPolygon defining the shape

        Returns:
            SphericalEarthAirspace instance
        """
        assert isinstance(shape, spherical_geometry.polygon.SphericalPolygon),\
            "airspace geometry must be a spherical geometry polygon object"
        self._shape = shape

    def is_point_inside(self, waypoint):
        """ Check if a point is inside the airspace.

        determine if a point lies within the airspace region (inside of its
        borders and outside of any holes in it)

        Arguments:
            waypoint: (3,) numpy array specifying a point in lla coordinates

        Returns:
            is_legal: bool specifying if the point is within the airspace
        """
        assert isinstance(waypoint, numpy.ndarray),\
            'waypoint must be numpy array'
        assert waypoint.shape == (3,), 'waypoint must be (3,)'
        waypoint_X = spherical_geometry.vector.radec_to_vector(
            waypoint[1], waypoint[0], degrees=False)
        return self._shape.contains_point(waypoint_X)

    def is_path_inside(self, trajectory):
        """ Determines if a flight path lies entirely within an airspace area.

        Arguments:
            trajectory: (n,3) numpy array defining a flight segment, lla
                waypoints along a great circle route

        Returns:
            is_legal: bool specifying if the flight segment lies within the
                airspace
        """
        assert isinstance(trajectory, numpy.ndarray),\
            'trajectory must be numpy array'
        assert trajectory.ndim == 2, 'trajectory must be (n,3)'
        trajectory = numpy.array(spherical_geometry.vector.radec_to_vector(
            trajectory[:,1], trajectory[:,0], degrees=False)).T
        is_legal = True
        for wp, next_wp in geometry.helpers.iter_next(trajectory, False):
            is_legal &= self._shape.contains_arc(wp, next_wp)
        return is_legal

    def any_intersection(self, trajectory):
        """ Finds all intersections between a trajectory and the boundaries.

        finds all of the points where a path intersects any edge of the airspace
        whether a boundary or a hole

        Arguments:
            trajectory: shapely LineString object or nx3 numpy array defining
                the ned positions of vertices in the trajectory

        Returns:
            intersections: tuple of intersection points between the given
                trajectory and the airspace

        """
        assert isinstance(trajectory, numpy.ndarray),\
            'trajectory must be numpy array'
        assert trajectory.ndim == 2, 'trajectory must be (n,3)'
        trajectory = numpy.array(spherical_geometry.vector.radec_to_vector(
            trajectory[:,1], trajectory[:,0], degrees=False)).T

        intersections = []
        for boundary in self._shape.points:
            iter_bp_and_wp = product(
                geometry.helpers.iter_next(boundary, False),
                geometry.helpers.iter_next(trajectory, False))
            for bp, next_bp, wp, next_wp in iter_bp:
                isect = spherical_geometry.great_circle_arc.intersection(
                    bp, next_bp, wp, next_wp)
                if not numpy.any(numpy.isnan(isect)):
                    intersections.append(isect)

        intersections = [
            spherical_geometry.vector.vector_to_radec(X, degrees=False)
            for X in intersections]
        return tuple(intersections)

