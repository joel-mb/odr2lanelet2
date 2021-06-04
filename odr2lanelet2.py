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
import sys

import lanelet2_utils
import opendrive
from bridge import Bridge

import carla

# ==================================================================================================
# -- conversor -------------------------------------------------------------------------------------
# ==================================================================================================

###
# REGULATORY ELEMENTS

# El lanelet tiene que tener un member:
#
#     <member type='relation' ref='xxxx' role='regulatory_element' />
#
# Hay una <relation> para cada regulatory element
#
#       <relation id='45234' visible='true' version='1'>
#           <member type='way' ref='43548' role='ref_line' /> --> way defining stop_line
#           <member type='way' ref='77702' role='refers' /> --> way defining traffic light
#           <member type='way' ref='69690' role='refers' /> --> another way defining traffic light?
#           <tag k='subtype' v='traffic_light' />
#           <tag k='type' v='regulatory_element' />
#       </relation>
#
###

# refers: The primitive(s) that are the very origin of the restriction. Traffic lights/signs, et cetera. Most Regulatory Elements need a refers role.
# cancels: The primitive(s) that mark the end of the restriction (if applicable).
# ref_line: The line (usually a LineString) from which a restrictions becomes valid. If not used, that usually means that the whole lanelet/area is affected by the restriction. However, there are exceptions, e.g. for traffic lights the stop line is the end of the lanelet.
# cancel_line: The line (usally a LineString) from which a restriction is no longer in place (if applicable)

# How to create a traffic light

#  1. Create a LineString defining the traffic_light
#  2. Create a LineString defining the stop_line
#  3. Create a RegulatoryElement related to the traffic light linestring and add the stop_line
#  4. Add to the lanelet the regulatory element (add_regulatory_element)

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
        self._lanelet2_map = lanelet2_utils.Lanelet2Map()
        self._odr2lanelet = {}

        logging.debug("Processing standard roads")
        map(self._convert_road_to_lanelets, self._odr_map.get_std_roads())
        logging.debug("Processing paths")
        map(self._convert_road_to_lanelets, self._odr_map.get_paths())
        logging.debug("Processing traffic lights")
        map(self._create_traffic_light, self._odr_map.get_traffic_lights())
        logging.debug("Processing crosswalks")
        map(self._create_crosswalk, self._odr_map.get_crosswalks())

        return self._lanelet2_map

    def _create_crosswalk(self, waypoints):
        right_locations = waypoints[:2]
        left_locations = waypoints[2:]

        right_points = [
            self._create_point_from_location(right_locations[0]),
            self._create_point_from_location(right_locations[1])
        ]
        right_points = [
            self._lanelet2_map.add_point(right_points[0]),
            self._lanelet2_map.add_point(right_points[1])
        ]
        right_linestring = lanelet2_utils.Linestring(self._next_uid(), right_points, attributes={})
        self._lanelet2_map.add_linestring(right_linestring)

        left_points = [
            self._create_point_from_location(left_locations[0]),
            self._create_point_from_location(left_locations[1])
        ]
        left_points = [
            self._lanelet2_map.add_point(left_points[0]),
            self._lanelet2_map.add_point(left_points[1])
        ]
        left_linestring = lanelet2_utils.Linestring(self._next_uid(), left_points, attributes={})
        self._lanelet2_map.add_linestring(left_linestring)

        uid = self._next_uid()
        attributes = {
            "subtype": "crosswalk",
            "type": "lanelet"
        }

        crosswalk = lanelet2_utils.Lanelet(uid, [right_linestring.uid, left_linestring.uid], attributes=attributes)
        return self._lanelet2_map.add_lanelet(crosswalk)

    def _create_traffic_light(self, traffic_light):

        def _create_light_box(light_box):
            # Create light box linestring
            bbox.get_world_vertices()
            right_point = self._create_point_from_location(right_location)
            left_point = self._create_point_from_location(left_location)

            points = [
                self._lanelet2_map.add_point(left_point),
                self._lanelet2_map.add_point(right_point)
            ]
            attributes = {
                'type': 'traffic_light',
                'subtype': 'red_yellow_green',
                'height': 1.4
            }
            light_box = lanelet2_utils.Linestring(self._next_uid(), points, attributes)

            # Create linestring for bulbs
            red_location = location + carla.Location(0.0, 0.0, 1.15)
            yellow_location = location + carla.Location(0.0, 0.0, 0.75)
            green_location = location + carla.Location(0.0, 0.0, 0.35)

            red_point = self._create_point_from_location(red_location)
            yellow_point = self._create_point_from_location(yellow_location)
            green_point = self._create_point_from_location(green_location)

            red_point.add_attribute('color', 'red')
            yellow_point.add_attribute('color', 'yellow')
            green_point.add_attribute('color', 'green')

            bulb_points = [
                self._lanelet2_map.add_point(red_point),
                self._lanelet2_map.add_point(yellow_point),
                self._lanelet2_map.add_point(green_point)
            ]

            attributes_bulb = {
                'traffic_light_id': light_box.uid,
                'type': 'light_bulbs'
            }
            bulbs = lanelet2_utils.Linestring(self._next_uid(), bulb_points, attributes_bulb)
            bulbs = self._lanelet2_map.add_linestring(light_bulbs)

            return (light_box, bulbs)

        # create light boxes and light bulbs
        light_boxes = map(_create_light_box, traffic_light.light_boxes)
        # create regulatory elements
        for landmark in traffic_light.landmarks:
            road_id, s = landmark.road_id, landmark.s
            for from_lane, to_lane in landmark.get_lane_validities():
                for lane_id in range(from_lane, to_lane + 1):
                    waypoint = self._odr_map.carla_map.get_waypoint_xodr(road_id, lane_id, s)

                    # create stop line
                    location = waypoint.transform.location
                    vector = waypoint.transform.get_right_vector()
                    right_location = location + (waypoint.lane_width / 2.0) * vector
                    left_location = location - (waypoint.lane_width / 2.0) * vector

                    right_point = self._create_point_from_location(right_location)
                    left_point = self._create_point_from_location(left_location)

                    points = [
                        self._lanelet2_map.add_point(left_point),
                        self._lanelet2_map.add_point(right_point)
                    ]

                    attributes = {
                        'type': 'stop_line'
                    }
                    stop_line = lanelet2_utils.Linestring(self._next_uid(), points, attributes)
                    self._lanelet2_map.add_linestring(stop_line)

                    # create regulatory element
                    attributes = {
                        'type': 'regulatory_element',
                        'subtype': 'traffic_light'
                    }
                    regulatory_element = lanelet2_utils.RegulatoryElement(self._next_uid(),
                                                                          [traffic_light.uid],
                                                                          stop_line.uid, attributes)
                    regulatory_element.add_member(
                        lanelet2_utils.Lanelet.Member('way', bulbs.uid, 'light_bulbs'))

                    self._lanelet2_map.add_regulatory_element(regulatory_element)

                    # Add regulatory element to the lanelet
                    lanelet_uid = self._odr2lanelet[waypoint.road_id, waypoint.section_id, waypoint.lane_id]
                    lanelet = self._lanelet2_map.get_lanelet(lanelet_uid)
                    lanelet.add_regulatory_element(regulatory_element.uid)

    def _create_point_from_location(self, location):
        uid = self._next_uid()

        geolocation = self._odr_map.transform_to_geolocation(location)
        lat = geolocation.latitude
        lon = geolocation.longitude
        attributes = {
            "ele": location.z,
            "local_x": location.x,
            "local_y": -location.y # From left-handed to right-handed system
        }

        return lanelet2_utils.Point(uid, lat, lon, attributes)

    def _create_point(self, waypoint, border):
        location = waypoint.transform.location
        vector = waypoint.transform.get_right_vector()
        if border == "left": vector *= -1
        border_location = location + (waypoint.lane_width / 2.0) * vector

        return self._create_point_from_location(border_location)

    def _create_linestring(self, start_waypoint, points, border):
        uid = self._next_uid()

        if border == 'left':
            carla_marking = start_waypoint.left_lane_marking
        else:
            carla_marking = start_waypoint.right_lane_marking

        lanelet2_marking = Bridge.lanelet2_marking(carla_marking)
        if start_waypoint.is_junction:
            lanelet2_marking = {'type': 'virtual'}

        attributes = lanelet2_marking
        return lanelet2_utils.Linestring(uid, points, attributes)

    def _create_lanelet(self, start_waypoint, linestrings):
        uid = self._next_uid()
        attributes = {
            # "location": "urban",
            # "one_way": "no",
            # "region": "de",
            "subtype": "road",
            "type": "lanelet"
        }

        return lanelet2_utils.Lanelet(uid, linestrings, attributes=attributes)

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

        while len(waypoints) < 2:

            successors = self._odr_map.get_successors(
                start_waypoint.road_id, start_waypoint.section_id,
                start_waypoint.lane_id)

            if len(successors) >= 1:
                successor = self._odr_map.get_waypoint(*successors[0])
                distance = self._odr_map.euclidean_distance(waypoints[-1], successor)
                print(distance)
                distance *= 0.6
                # TODO(joel): Check epsilon carla
                # distance = sys.float_info.min
            else:
                print("Warning: No tenemos successor")
                distance = sys.float_info.min
            # next_waypoint = waypoints[-1].next(sys.float_info.min)
            next_waypoint = waypoints[-1].next(distance)
            print("Distance applied: {}".format(distance))
            print(start_waypoint.road_id, start_waypoint.section_id, start_waypoint.lane_id)
            print(next_waypoint[0].road_id, next_waypoint[0].section_id, next_waypoint[0].lane_id)
            assert len(next_waypoint) == 1 and next_waypoint[
                0].road_id == start_waypoint.road_id and next_waypoint[
                    0].section_id == start_waypoint.section_id and next_waypoint[
                        0].lane_id == start_waypoint.lane_id
            waypoints.append(next_waypoint[0])

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


def odr2lanelet2(odr_map, world, output, sampling_distance):
    conversor = Odr2Lanelet2Conversor(sampling_distance)
    lanelet2_map = conversor(odr_map)
    lanelet2_utils.save(lanelet2_map, output)
    print("Conversion completed!!!")
    print("Total points: {}".format(len(lanelet2_map._points)))
    print("Total linestrings: {}".format(len(lanelet2_map._linestrings)))
    print("Total lanelets: {}".format(len(lanelet2_map._lanelets)))


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host',
                           '-h',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('--port',
                           '-p',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--xodr_path', '-x', help='opendrive file (*.xodr)')
    argparser.add_argument('--output',
                           '-o',
                           default="lanelet2.osm",
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

    client = carla.Client(args.host, args.port)

    # load the opendrive map if any.
    if args.xodr_path != "":
        with open(args.xodr_path) as odr_file:
            odr_data = odr_file.read()
            client.generate_opendrive_world(odr_data)

    carla_world = client.get_world()
    carla_map = carla_world.get_map()
    odr_map = OdrMap(carla_map, carla_world)

    odr2lanelet2(odr_map, args.output, sampling_distance=1.0)
