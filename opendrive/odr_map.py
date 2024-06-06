import collections
import math

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

    def _create_waypoints(self, carla_topology):
        result = collections.defaultdict(lambda: collections.defaultdict(dict))

        # Create dictionary of start waypoints. Do not take into account successors wps as this point.
        for waypoint, _ in carla_topology:
            result[waypoint.road_id][waypoint.section_id][waypoint.lane_id] = waypoint

        return result

    def _create_topology(self, carla_topology):
        topology = collections.defaultdict(lambda: ([], []))

        for wp, succ in carla_topology:

            segment_id = (wp.road_id, wp.section_id, wp.lane_id)
            succ_segment_id = (succ.road_id, succ.section_id, succ.lane_id)

            if segment_id == succ_segment_id:
                topology[segment_id][1].append(succ)

            else:
                topology[segment_id][1].append(succ)
                topology[succ_segment_id][0].append(wp)

        return topology

    def get_waypoint(self, road_id, section_id, lane_id):
        if road_id not in self._waypoints:
            return None
        if section_id not in self._waypoints[road_id]:
            return None
        if lane_id not in self._waypoints[road_id][section_id]:
            return None

        return self._waypoints[road_id][section_id][lane_id]

    def get_waypoints(self):
        waypoints = []
        for road_id in self._waypoints:
            for section_id in self._waypoints[road_id]:
                for lane_id in self._waypoints[road_id][section_id]:
                    waypoints.append(self._waypoints[road_id][section_id][lane_id])

        return waypoints

    def get_roads(self):
        #return set(sorted(self._waypoints.keys()))
        return set([wp.road_id for wp in self.get_waypoints()])

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

    def get_segment_predecessors(self, road_id, section_id, lane_id):
        predecessors = []
        for wp in self.get_waypoint_predecessors(road_id, section_id, lane_id):
            wp_segment = (wp.road_id, wp.section_id, wp.lane_id)
            if wp_segment == (road_id, section_id, lane_id):
                continue
            predecessors.append(wp_segment)
        return predecessors

    def get_segment_successors(self, road_id, section_id, lane_id):
        successors = []
        for wp in self.get_waypoint_successors(road_id, section_id, lane_id):
            wp_segment = (wp.road_id, wp.section_id, wp.lane_id)
            if wp_segment == (road_id, section_id, lane_id):
                continue
            successors.append(wp_segment)
        return successors

    def get_waypoint_predecessors(self, road_id, section_id, lane_id):
        if (road_id, section_id, lane_id) not in self._topology:
            return []
        return self._topology[road_id, section_id, lane_id][0]

    def get_waypoint_successors(self, road_id, section_id, lane_id):
        if (road_id, section_id, lane_id) not in self._topology:
            return []
        return self._topology[road_id, section_id, lane_id][1]

    def get_right(self, road_id, section_id, lane_id):
        if (road_id, section_id, lane_id) not in self._topology:
            return None

        right_lane_id = lane_id + int(math.copysign(1, lane_id))
        right = (road_id, section_id, right_lane_id)
        if right not in self._topology:
            return None

        return right

    def get_left(self, road_id, section_id, lane_id):
        if (road_id, section_id, lane_id) not in self._topology:
            return None

        left_lane_id = lane_id - int(math.copysign(1, lane_id)) if abs(lane_id) != 1 else -1 * lane_id
        left = (road_id, section_id, left_lane_id)
        
        if left not in self._topology:
            return None

        return left

    def transform_to_geolocation(self, location):
        return self.carla_map.transform_to_geolocation(location)

    def get_border(self, waypoint, border="right"):
        location = waypoint.transform.location
        vector = waypoint.transform.get_right_vector()
        if border == "left": vector *= -1
        border_location = location + (waypoint.lane_width / 2.0) * vector
        return border_location

    def get_crosswalks(self):
        crosswalk_points = self.carla_map.get_crosswalks()
        assert len(crosswalk_points) % 5 == 0

        crosswalks = []
        for i in range(0, len(crosswalk_points), 5):
            p1, p2, p3, p4, _ = crosswalk_points[i:i+5]
            crosswalks.append((p1, p2, p3, p4))

        return crosswalks