#!/usr/bin/env python3

import pdb
import time
import unittest

import numpy
import scipy.spatial
import shapely.geometry
import yaml

import geodesy.conversions
import navigation.obstacle_space
import navigation.obstacle_primitives

class TestObstacleSpace(unittest.TestCase):

    def setUp(self):
        obstacles_file = 'unit_cube.yaml'

        with open(obstacles_file, 'r') as yfile:
            obstacle_data = yaml.load(yfile)

        ref_pt = numpy.array(obstacle_data['lla_ref'])[None]
        obstacles = obstacle_data['obstacles']
        self.ospace = navigation.obstacle_space.PrismaticObstacleSpace(
            obstacles, ref_pt, is_radians=True)

    def test_no_intersection(self):
        ned_path = numpy.array([
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0]])
        self.assertEqual(len(self.ospace.intersections(ned_path)), 0)

    def test_side_intersection(self):
        ned_path = numpy.array([
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]])
        intersections = self.ospace.intersections(ned_path)
        i1 = numpy.array([-0.5, 0.0, 0.0])
        i2 = numpy.array([0.5, 0.0, 0.0])
        self.assertEqual(len(intersections), 2)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[0]) - i1) < 1.0e-6)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[1]) - i2) < 1.0e-6)

    def test_face_intersection(self):
        ned_path = numpy.array([
            [0.0, 0.0, -1.0],
            [0.0, 0.0, 1.0]])
        intersections = self.ospace.intersections(ned_path)
        i1 = numpy.array([0.0, 0.0, 0.5])
        i2 = numpy.array([0.0, 0.0, -0.5])
        self.assertEqual(len(intersections), 2)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[0]) - i1) < 1.0e-6)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[1]) - i2) < 1.0e-6)

    def test_corner_intersection(self):
        ned_path = numpy.array([
            [-1.0, -1.0, -1.0],
            [1.0, 1.0, 1.0]])
        intersections = self.ospace.intersections(ned_path)
        i1 = numpy.array([-0.5, -0.5, -0.5])
        i2 = numpy.array([0.5, 0.5, 0.5])
        self.assertEqual(len(intersections), 2)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[0]) - i1) < 1.0e-6)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[1]) - i2) < 1.0e-6)

    def test_side_face_intersection(self):
        ned_path = numpy.array([
            [-1.0, 0.0, -0.5],
            [0.5, 0.0, 1.0]])
        intersections = self.ospace.intersections(ned_path)
        i1 = numpy.array([-0.5, 0.0, 0.0])
        i2 = numpy.array([0.0, 0.0, 0.5])
        self.assertEqual(len(intersections), 2)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[1]) - i1) < 1.0e-6)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[0]) - i2) < 1.0e-6)

    def test_one_side_intersection(self):
        ned_path = numpy.array([
            [0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0]])
        intersections = self.ospace.intersections(ned_path)
        i = numpy.array([0.0, -0.5, 0.0])
        self.assertEqual(len(intersections), 1)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[0]) - i) < 1.0e-6)

    def test_one_face_intersection(self):
        ned_path = numpy.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0]])
        intersections = self.ospace.intersections(ned_path)
        i = numpy.array([0.0, 0.0, 0.5])
        self.assertEqual(len(intersections), 1)
        self.assertTrue(
            numpy.linalg.norm(numpy.array(intersections[0]) - i) < 1.0e-6)


if __name__ == '__main__':
    unittest.main()
