#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import collections
import itertools

import carla

def load_map(xodr_file):
    with open(xodr_file, 'r') as f:
        carla_map = carla.Map('odr2lanelet2', str(f.read()))
    return OdrMap(carla_map)


class OdrMap(object):
    def __init__(self, carla_map):
        self.carla_map = carla_map

        carla_topology = self.carla_map.get_topology()
        self._waypoints = self._create_waypoints(carla_topology)
        self._topology = self._create_topology(carla_topology)

    def transform_to_geolocation(self, location):
        return self.carla_map.transform_to_geolocation(location)

    def _create_waypoints(self, carla_topology):
        result = collections.defaultdict(lambda: collections.defaultdict(dict))

        all_waypoints = list(itertools.chain.from_iterable(carla_topology))
        for waypoint in all_waypoints:
            result[waypoint.road_id][waypoint.section_id][waypoint.lane_id] = waypoint

        return result

    def _create_topology(self, carla_topology):
        result = collections.defaultdict(lambda: ([], []))

        for waypoint_pair in carla_topology:
            waypoint, successor = waypoint_pair
            waypoint_segment_id = (waypoint.road_id, waypoint.section_id, waypoint.lane_id)
            successor_segment_id = (successor.road_id, successor.section_id, successor.lane_id)

            result[waypoint_segment_id][1].append(successor_segment_id)
            result[successor_segment_id][0].append(waypoint_segment_id)

        return result

    def get_waypoints(self):
        result = []
        for road_id in self._waypoints:
            for section_id in self._waypoints[road_id]:
                for lane_id in self._waypoints[road_id][section_id]:
                    result.append(self._waypoints[road_id][section_id][lane_id])

        return result

    def get_waypoint(self, road_id, section_id, lane_id):
        if road_id not in self._waypoints:
            return None
        if section_id not in self._waypoints[road_id]:
            return None
        if lane_id not in self._waypoints[road_id][section_id]:
            return None

        return self._waypoints[road_id][section_id][lane_id]

    def get_std_roads(self):
        return set([wp.road_id for wp in filter(lambda wp: not wp.is_junction, self.get_waypoints())])

    def get_paths(self):
        return set([wp.road_id for wp in filter(lambda wp: wp.is_junction, self.get_waypoints())])

    def get_sections(self, road_id):
        if road_id not in self._waypoints:
            return []
        return sorted(self._waypoints[road_id].keys())

    def get_lanes(self, road_id, section_id):
        if road_id not in self._waypoints and section_id not in self._waypoints[road_id]:
            return []
        return sorted(self._waypoints[road_id][section_id].keys())

    def get_predecessors(self, road_id, section_id, lane_id):
        if (road_id, section_id, lane_id) not in self._topology:
            return []
        return self._topology[road_id, section_id, lane_id][0]

    def get_successors(self, road_id, section_id, lane_id):
        if (road_id, section_id, lane_id) not in self._topology:
            return []
        return self._topology[road_id, section_id, lane_id][1]

