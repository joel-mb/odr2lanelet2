#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import collections
import itertools
import math

import carla

# import pyproj
# import lanelet2

# DEFAULT_PROJ_STRING = "+proj=utm +zone=32 +ellps=WGS84"
# DEFAULT_PROJ_STRING = "+proj=utm +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs"

# def load_map(xodr_file):
# with open(xodr_file, 'r') as f:
# carla_map = carla.Map('odr2lanelet2', str(f.read()))
# return OdrMap(carla_map)


class OdrMap(object):

    TL_ODR_ID = "1000001"
    TrafficLight = collections.namedtuple('TrafficLight', 'landmarks boxes')

    def __init__(self, carla_map, carla_world=None):
        self.carla_map = carla_map
        self.carla_world = carla_world

        carla_topology = self.carla_map.get_topology()
        self._waypoints = self._create_waypoints(carla_topology)
        self._topology = self._create_topology(carla_topology)

        # self._proj = lanelet2.projection.UtmProjector(lanelet2.io.Origin(0.0, 0.0, 0.0))
        # self._proj = pyproj.Proj(DEFAULT_PROJ_STRING)
        # self._proj = pyproj.Proj(proj='utm')
        self._crosswalks = self._compute_crosswalks()
        self._traffic_lights = self._compute_traffic_lights()

    @staticmethod
    def euclidean_distance(waypoint, other_waypoint):
        """
        Computes euclidean distance between to locations.
        """
        location = waypoint.transform.location
        other_location = other_waypoint.transform.location
        x1, y1, z1 = location.x, location.y, location.z
        x2, y2, z2 = other_location.x, other_location.y, other_location.z
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

    def transform_to_geolocation(self, location):
        # lon, lat = self._proj(location.x, location.y)
        # return carla.GeoLocation(lat, lon, location.z)
        geolocation = self.carla_map.transform_to_geolocation(location)
        # print (self._proj.forward(lanelet2.core.GPSPoint(lat=geolocation.latitude, lon=geolocation.longitude)))
        gps = self._proj.reverse(lanelet2.core.BasicPoint3d(location.x, -location.y, location.z))
        print("-----------")
        print('Carla: ', geolocation)
        print('Lanelet2: ', carla.GeoLocation(gps.lat, gps.lon, location.z))
        print("-----------")
        return carla.GeoLocation(gps.lat, gps.lon, location.z)

        # return self.carla_map.transform_to_geolocation(location)

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

    def _compute_crosswalks(self):
        crosswalks = []
        locations = self.carla_map.get_crosswalks()
        if not locations:
            return crosswalks

        crosswalks = [[locations[0]]]
        for i, location in enumerate(locations[1:]):
            if location not in crosswalks[-1]:
                crosswalks[-1].append(location)
            else:
                if not i == len(locations) - 2:
                    crosswalks.append([])
        return crosswalks

    def _compute_traffic_lights(self):
        tls = []
        actors = self.carla_world.get_actors()

        tl_actors = filter(lambda actor: isinstance(actor, carla.TrafficLight), actors)
        all_landmarks = self.carla_map.get_all_landmarks_of_type(OdrMap.TL_ODR_ID)
        all_boxes = self.carla_world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
        for tl in tl_actors:
            location = tl.get_transform().location

            landmarks = filter(lambda l: tl.id == self.world.get_traffic_light(l).id, all_landmarks)
            boxes = filter(lambda b: b.transform.location == location, all_boxes)

            tls.append(TrafficLight(actor, list(tl_landmarks), list(boxes)))
        return tls

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
        return set(
            [wp.road_id for wp in filter(lambda wp: not wp.is_junction, self.get_waypoints())])

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

    def get_crosswalks(self):
        return self._crosswalks

    def get_traffic_lights(self):
        return self._traffic_lights

