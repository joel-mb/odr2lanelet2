#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""OpenDRIVE to Lanelet2 conversor tool"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging

import lanelet2
import opendrive
from bridge import Bridge

import carla

# ==================================================================================================
# -- conversor -------------------------------------------------------------------------------------
# ==================================================================================================


class Odr2Lanelet2Conversor(object):
    """
    Conversor from OpenDRIVE to Lanelet2
    """
    def __init__(self, sampling_distance=2):
        self.sampling_distance = sampling_distance

        self._odr_map = None
        self._lanelet2_map = None

        # dict to save the relation of odr lanes and lanelets.
        self._odr2lanelet = {}  # {(road_id, section_id, lane_id): lanelet_uid, ...}

        self._uid = 0

    def _next_uid(self):
        self._uid += 1
        return self._uid

    def __call__(self, odr_map):
        self._odr_map = odr_map
        self._lanelet2_map = lanelet2.Lanelet2Map()
        self._odr2lanelet = {}

        logging.debug("Processing standard roads")
        map(self._convert_road_to_lanelets, self._odr_map.get_std_roads())
        logging.debug("Processing paths")
        map(self._convert_road_to_lanelets, self._odr_map.get_paths())

        return self._lanelet2_map

    def _create_point(self, waypoint, border):
        uid = self._next_uid()

        location = waypoint.transform.location
        vector = waypoint.transform.get_right_vector()
        if border == "left": vector *= -1
        border_location = location + (waypoint.lane_width / 2.0) * vector

        geolocation = self._odr_map.transform_to_geolocation(border_location)
        lat = geolocation.latitude
        lon = geolocation.longitude
        attributes = {
            "ele": border_location.z,
            "local_x": border_location.x,
            "local_y": -border_location.y # From left-handed to right-handed system
        }

        return lanelet2.Point(uid, lat, lon, attributes)

    def _create_linestring(self, start_waypoint, points, border):
        # assert len(points) > 1
        uid = self._next_uid()

        if border == 'left':
            carla_marking = start_waypoint.left_lane_marking
        else:
            carla_marking = start_waypoint.right_lane_marking

        attributes = Bridge.lanelet2_marking(carla_marking)
        if start_waypoint.is_junction:
            attributes = {'type': 'virtual'}

        return lanelet2.Linestring(uid, points, attributes)

    def _create_lanelet(self, start_waypoint, linestrings):
        uid = self._next_uid()
        attributes = {
            "location": "urban",
            "one_way": "no",
            "region": "de",
            "subtype": "road",
            "type": "lanelet"
        }

        return lanelet2.Lanelet(uid, linestrings, attributes)

    def _is_adjacent(self, lane_id, other_lane_id):
        direction = lane_id * other_lane_id
        difference = abs(lane_id - other_lane_id)
        if direction < 0 and difference > 2:
            return False
        if direction > 0 and difference > 1:
            return False
        return True

    def _convert_road_to_lanelets(self, road_id):
        for section_id in self._odr_map.get_sections(road_id):
            for idx, lane_id in enumerate(self._odr_map.get_lanes(road_id, section_id)):
                logging.debug("Processing {}, {}, {}".format(road_id, section_id, lane_id))

                start_waypoint = self._odr_map.get_waypoint(road_id, section_id, lane_id)
                reference_waypoints = self._create_reference_waypoints(start_waypoint)

                # Check predecessor and successor points.
                predecessors = self._get_predecessor_points(road_id, section_id, lane_id)
                successors = self._get_successor_points(road_id, section_id, lane_id)

                # For the initial (road_id, section_id, lane_id) combination with compute both the
                # right border and the left border.
                if idx == 0 or not self._is_adjacent(lane_id, last_lane_id):
                    left_points = [self._create_point(wp, "left") for wp in reference_waypoints]
                    right_points = [self._create_point(wp, "right") for wp in reference_waypoints]

                    left_edge = map(self._lanelet2_map.add_point, left_points)
                    right_edge = map(self._lanelet2_map.add_point, right_points)
                    edges = (
                        predecessors[0] + left_edge + successors[0],
                        predecessors[1] + right_edge + successors[1]
                    )

                    left_linestring = self._create_linestring(start_waypoint, edges[0], "left")
                    right_linestring = self._create_linestring(start_waypoint, edges[1], "right")
                    linestrings = (
                        self._lanelet2_map.add_linestring(left_linestring),
                        self._lanelet2_map.add_linestring(right_linestring)
                    )

                else:
                    if lane_id < 0:
                        left_points = [self._create_point(wp, "left") for wp in reference_waypoints]

                        left_edge = map(self._lanelet2_map.add_point, left_points)
                        edges = (
                            predecessors[0] + left_edge + successors[0],
                            last_edges[0][:]
                        )

                        left_linestring = self._create_linestring(start_waypoint, edges[0], "left")
                        linestrings = (
                            self._lanelet2_map.add_linestring(left_linestring),
                            last_linestrings[0]
                        )

                    else:
                        right_points = [self._create_point(wp, "right") for wp in reference_waypoints]
                        right_edge = map(self._lanelet2_map.add_point, right_points)
                        edges = (
                            last_edges[0][::-1] if lane_id == 1 else last_edges[1][:],
                            predecessors[1] + right_edge + successors[1]
                        )

                        left_linestring = self._create_linestring(start_waypoint, edges[0], "left")
                        right_linestring = self._create_linestring(start_waypoint, edges[1], "right")
                        linestrings = (
                            self._lanelet2_map.add_linestring(left_linestring) if lane_id == 1 else last_linestrings[1],
                            self._lanelet2_map.add_linestring(right_linestring)
                        )

                lanelet = self._create_lanelet(start_waypoint, linestrings)
                lanelet_uid = self._lanelet2_map.add_lanelet(lanelet)
                self._odr2lanelet[road_id, section_id, lane_id] = lanelet_uid

                last_edges = edges
                last_linestrings = linestrings
                last_lane_id = lane_id

    def _create_reference_waypoints(self, start_waypoint):
        """
        Create reference list of waypoints.

        All the waypoints belonging to the reference list will have, by definition, the same road id,
        section id and lane id.
        """
        waypoints = [start_waypoint]

        next_waypoint = start_waypoint.next(self.sampling_distance)
        while (len(next_waypoint) == 1
               and start_waypoint.road_id == next_waypoint[0].road_id
               and start_waypoint.section_id == next_waypoint[0].section_id):
            waypoints.append(next_waypoint[0])
            next_waypoint = next_waypoint[0].next(self.sampling_distance)

        return waypoints

    def _get_predecessor_points(self, road_id, section_id, lane_id):
        """
        Return predecessors points.
        """
        predecessors = self._odr_map.get_predecessors(road_id, section_id, lane_id)
        if not predecessors:
            return ([], [])

        # TODO(joel): We get only the first predecessor?
        predecessor = predecessors[0]

        if predecessor in self._odr2lanelet:
            assert len(predecessors) == 1

            lanelet_uid = self._odr2lanelet[predecessor]
            borders = self._lanelet2_map.get_lanelet(lanelet_uid).borders
            edges = (self._lanelet2_map.get_linestring(borders[0]).points,
                     self._lanelet2_map.get_linestring(borders[1]).points)

            return ([edges[0][-1]], [edges[1][-1]])

        return ([], [])

    def _get_successor_points(self, road_id, section_id, lane_id):
        """
        Returns successors points.
        """
        successors = self._odr_map.get_successors(road_id, section_id, lane_id)
        if not successors:
            return ([], [])

        # TODO(joel): We get only the first successor?
        successor = successors[0]

        if successor in self._odr2lanelet:
            assert len(successors) == 1

            lanelet_uid = self._odr2lanelet[successor]
            borders = self._lanelet2_map.get_lanelet(lanelet_uid).borders
            edges = (self._lanelet2_map.get_linestring(borders[0]).points,
                     self._lanelet2_map.get_linestring(borders[1]).points)

            return ([edges[0][0]], [edges[1][0]])

        return ([], [])


def odr2lanelet2(xodr_file, output, sampling_distance):
    odr_map = opendrive.load_map(xodr_file)

    conversor = Odr2Lanelet2Conversor(sampling_distance)
    lanelet2_map = conversor(odr_map)
    lanelet2.save(lanelet2_map, output)
    print("Conversion completed!!!")
    print("Total points: {}".format(len(lanelet2_map._points)))
    print("Total linestrings: {}".format(len(lanelet2_map._linestrings)))
    print("Total lanelets: {}".format(len(lanelet2_map._lanelets)))


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('xodr_file', help='opendrive file (*.xodr')
    argparser.add_argument('--output',
                           '-o',
                           default="lanele2.osm",
                           help="output (*.osm)")
    argparser.add_argument('--debug',
                           '-v',
                           action='store_true',
                           help='increase output verbosity')
    args = argparser.parse_args()

    if args.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s',
                            level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s',
                            level=logging.INFO)

    odr2lanelet2(args.xodr_file, args.output, sampling_distance=1.0)
