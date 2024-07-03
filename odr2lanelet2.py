"""OpenDRIVE to Lanelet2 conversor tool"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import math

import lanelet2
import opendrive
from bridge import Bridge

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

        logging.info("Processing standard roads")
        list(map(self._convert_road_to_lanelets, self._odr_map.get_std_roads()))
        logging.info("Processing paths")
        list(map(self._convert_road_to_lanelets, self._odr_map.get_paths()))
        logging.info("Processing crosswalks")
        list(map(self._convert_crosswalk_to_lanelet, self._odr_map.get_crosswalks()))
        logging.info("Processing traffic lights")
        list(map(self._convert_traffic_light_to_regulatory_element, self._odr_map.get_traffic_lights()))

        return self._lanelet2_map

    def _create_point(self, location, extra_attributes={}):
        uid = self._next_uid()
        
        geolocation = self._odr_map.transform_to_geolocation(location)
        lat = geolocation.latitude
        lon = geolocation.longitude
        attributes = {
            "ele": location.z,
            "local_x": location.x,
            "local_y": -location.y # From left-handed to right-handed system
        }

        return lanelet2.Point(uid, lat, lon, {**attributes, **extra_attributes})

    def _is_adjacent(self, road_id, section_id, lane_id, other_lane_id):
        direction = lane_id * other_lane_id
        difference = abs(lane_id - other_lane_id)
        if direction < 0 and difference > 2:
            return False
        if direction > 0 and difference > 1:
            return False

        # Even if lanes are adjacent, check that they do not have same predecessros or successors.
        # Some CARLA maps have wrong lane linkage between section:
        #
        #  *------------------*
        #  |        L2        |
        #  *------------------*------------------*
        #  |        L1        |        L3        |
        #  *------------------*------------------*
        #
        # In the exmaple above, opendrive linkage is L1->L3 amd L2->L3.
        # By definition, L1 and L2 should be adjacent, however they are sharing the same successor.
        # We detect this situation and mark these two lanes as NOT adjacents.
        common_predecessors = set(self._odr_map.get_segment_predecessors(road_id, section_id, lane_id)) & \
                              set(self._odr_map.get_segment_predecessors(road_id, section_id, other_lane_id))
        if common_predecessors:
            logging.warning(
                "Segments {}|{}|{} and {}|{}|{} should be adjacent but sharing at least one predecessor {}|{}|{}".format(
                    road_id, section_id, lane_id,
                    road_id, section_id, other_lane_id,
                    *list(common_predecessors)[0]
                )
            )
            return False

        common_successors = set(self._odr_map.get_segment_successors(road_id, section_id, lane_id)) & \
                            set(self._odr_map.get_segment_successors(road_id, section_id, other_lane_id))
        if common_successors:
            logging.warning(
                "Segment {}|{}|{} and {}|{}|{} should be adjacent but sharing at least one successor {}|{}|{}".format(
                    road_id, section_id, lane_id,
                    road_id, section_id, other_lane_id,
                    *list(common_successors)[0]
                )
            )
            return False

        return True

    def _convert_road_to_lanelets(self, road_id):

        def _create_border_linestring(start_waypoint, points, border):
            uid = self._next_uid()

            if border == "left":
                carla_marking = start_waypoint.left_lane_marking
            else:
                carla_marking = start_waypoint.right_lane_marking

            attributes = Bridge.lanelet2_marking(carla_marking)
            if start_waypoint.is_junction:
                attributes = {"type": "virtual"}

            return lanelet2.Linestring(uid, points, attributes)

        for section_id in self._odr_map.get_sections(road_id):
            # Lane sections of a same road are processed from smaller to higher lane ids.
            #
            #  *--------------*
            #  |  LANE ID  1  |  ^
            #  *--------------*  ^
            #  |  LANE ID -1  |  ^
            #  *--------------*  ^
            #  |  LANE ID -2  |  ^
            #  *--------------*
            #
            for idx, lane_id in enumerate(self._odr_map.get_lanes(road_id, section_id)):
                logging.debug("Processing {}|{}|{}".format(road_id, section_id, lane_id))

                start_waypoint = self._odr_map.get_waypoint(road_id, section_id, lane_id)
                end_waypoint = self._odr_map.get_waypoint_successors(road_id, section_id, lane_id)[0]

                # start_transform = start_waypoint.transform
                # end_transform = end_waypoint.transform if end_waypoint else None

                pre, succ = self._get_link_points(road_id, section_id, lane_id)

                reference_border = self._create_reference_border(start_waypoint, end_waypoint) 

                # For the initial (road_id, section_id, lane_id) combination or lanes that are not adjacent, we compute
                # both the right and the left border
                if idx == 0 or not self._is_adjacent(road_id, section_id, lane_id, last_lane_id):

                    left_points = [self._create_point(self._odr_map.get_border(start_waypoint, "left")) if not pre[0] else pre[0]]
                    left_points += [self._create_point(loc) for loc in reference_border[0]]
                    left_points += [self._create_point(self._odr_map.get_border(end_waypoint, "left")) if not succ[0] else succ[0]]

                    right_points = [self._create_point(self._odr_map.get_border(start_waypoint, "right")) if not pre[1] else pre[1]]
                    right_points += [self._create_point(loc) for loc in reference_border[1]]
                    right_points += [self._create_point(self._odr_map.get_border(end_waypoint, "right")) if not succ[1] else succ[1]]

                    left_edge = list(map(self._lanelet2_map.add_point, left_points))
                    right_edge = list(map(self._lanelet2_map.add_point, right_points))

                    edges = (
                        left_edge,
                        right_edge
                    )

                    left_linestring = _create_border_linestring(start_waypoint, edges[0], "left")
                    right_linestring = _create_border_linestring(start_waypoint, edges[1], "right")
                    linestrings = (
                        self._lanelet2_map.add_linestring(left_linestring),
                        self._lanelet2_map.add_linestring(right_linestring)
                    )

                # For adjacent lanes we share right or left border
                else:
                    
                    # If current lane id is negative, the right border is shared with the left border of the previous 
                    # processed lane:
                    #
                    #  *--------------*
                    #  |  LANE ID -1  |
                    #  |    ---->     |  ^
                    #  *--------------*  ^
                    #  |  LANE ID -2  |  ^
                    #  |    ---->     |  ^
                    #  *--------------*
                    if lane_id < 0:

                        left_points = [self._create_point(self._odr_map.get_border(start_waypoint, "left")) if not pre[0] else pre[0]]
                        left_points += [self._create_point(loc) for loc in reference_border[0]]
                        left_points += [self._create_point(self._odr_map.get_border(end_waypoint, "left")) if not succ[0] else succ[0]]

                        left_edge = list(map(self._lanelet2_map.add_point, left_points))
                        edges = (
                            left_edge,
                            last_edges[0][:]
                        )

                        left_linestring = _create_border_linestring(start_waypoint, edges[0], "left")
                        linestrings = (
                            self._lanelet2_map.add_linestring(left_linestring),
                            last_linestrings[0]
                        )

                    # If current lane id is positive:
                    #
                    #    * If current lane id is equal to 1, the left border is shared with the left border of the
                    #      previous processed lane in reversed order.
                    #
                    #        *--------------*
                    #        |  LANE ID  1  |
                    #        |    <----     |  ^
                    #        *--------------*  ^
                    #        |  LANE ID -1  |  ^
                    #        |    ---->     |  ^
                    #        *--------------*
                    #
                    #    * If current lane id is higher than 1, the left border is shared with the right border of the
                    #      previous processed lane.
                    #
                    #        *--------------*
                    #        |  LANE ID  2  |
                    #        |    <----     |  ^
                    #        *--------------*  ^
                    #        |  LANE ID  1  |  ^
                    #        |    <----     |  ^
                    #        *--------------*
                    else:
                        right_points = [self._create_point(self._odr_map.get_border(start_waypoint, "right")) if not pre[1] else pre[1]]
                        right_points += [self._create_point(loc) for loc in reference_border[1]]
                        right_points += [self._create_point(self._odr_map.get_border(end_waypoint, "right")) if not succ[1] else succ[1]]

                        right_edge = list(map(self._lanelet2_map.add_point, right_points))
                        edges = (
                            last_edges[0][::-1] if lane_id == 1 else last_edges[1][:],
                            right_edge
                        )

                        left_linestring = _create_border_linestring(start_waypoint, edges[0], "left")
                        right_linestring = _create_border_linestring(start_waypoint, edges[1], "right")
                        linestrings = (
                            self._lanelet2_map.add_linestring(left_linestring) if lane_id == 1 else last_linestrings[1],
                            self._lanelet2_map.add_linestring(right_linestring)
                        )

                lanelet = lanelet2.Lanelet(
                    uid=self._next_uid(),
                    left=linestrings[0],
                    right=linestrings[1],
                    regulatory_elements=[],
                    attributes = {
                        "type": "lanelet",
                        "subtype": "road",
                        "location": "urban",
                        "one_way": "yes",
                        "speed_limit": "30",
                        "turn_direction": "" # TODO
                    }
                )
                lanelet_uid = self._lanelet2_map.add_lanelet(lanelet)
                self._odr2lanelet[road_id, section_id, lane_id] = lanelet_uid

                last_edges = edges
                last_linestrings = linestrings
                last_lane_id = lane_id

                #print("{}: Real points->".format(road_id), [(edges[0][0], edges[1][0]), (edges[0][-1], edges[1][-1])])

    def _create_reference_border(self, start_waypoint, end_waypoint):
        """
        Create reference list of transforms.

        All the transforms belonging to the reference list will have, by definition, the same road id,
        section id and lane id.
        """
        ltransforms = []
        rtransforms = []

        def _is_aligned(loc1, loc2, loc3):

            dx1, dy1 = loc1.x - loc2.x, loc1.y - loc2.y
            dx2, dy2 = loc2.x - loc3.x, loc2.y - loc3.y

            angle1 = math.atan2(dx1, dy1)
            angle2 = math.atan2(dx2, dy2)

            THRESHOLD_ANGLE = 0.01
            diff_angle = abs(angle1 - angle2)
            if diff_angle < THRESHOLD_ANGLE:
                return True
            return False

        # Create a buffer for both left and right border:
        #
        #    * Buffer: [current, candidate, look_ahead]
        #
        # The buffer consist of:
        #    - `current`: last point added (start waypoint border at the begining)
        #    - `candidate`: candidate point to be added.
        #    - `look_ahead`: look ahead point to decide if candidate is added.
        #
        # Given diff = ( angle(current->candidate) - angle(candidate->look_ahead) ):
        #
        #    * If diff < THRESHOLD candidate is NOT added:
        #        current -> current
        #        candidate -> look_ahead
        #        look_ahead -> None
        #
        #    * If diff > THRESHOLD candidate is added:
        #        current -> candidate
        #        candidate -> look_ahead
        #        look_ahead -> None

        lbuffer = [self._odr_map.get_border(start_waypoint, "left"), None, None]
        rbuffer = [self._odr_map.get_border(start_waypoint, "right"), None, None]

        next_waypoint = start_waypoint.next(self.sampling_distance)
        while (len(next_waypoint) == 1
               and start_waypoint.road_id == next_waypoint[0].road_id
               and start_waypoint.section_id == next_waypoint[0].section_id):

            # If end_waypoint is provided check that the distance between next waypoint and end waypoint is higher
            # than the sampling distance. Otherwise break the loop.
            if end_waypoint:
                next_location = next_waypoint[0].transform.location
                end_location = end_waypoint.transform.location

                distance = next_location.distance(end_location)
                if distance < self.sampling_distance:
                    break

            # No candidate. This only happens during the first iteration
            if lbuffer[1] == None and rbuffer[1] == None:
                lbuffer[1] = self._odr_map.get_border(next_waypoint[0], "left")
                rbuffer[1] = self._odr_map.get_border(next_waypoint[0], "right")
  
            else:
                # Compute look ahead point
                lbuffer[2] = self._odr_map.get_border(next_waypoint[0], "left")
                rbuffer[2] = self._odr_map.get_border(next_waypoint[0], "right")

                # Left border
                if _is_aligned(*lbuffer):
                    lbuffer[1] = lbuffer[2]
                    lbuffer[2] = None
                else:
                    ltransforms.append(lbuffer[1])

                    lbuffer[0] = lbuffer[1]
                    lbuffer[1] = lbuffer[2]
                    lbuffer[2] = None

                # Right border
                if _is_aligned(*rbuffer):
                    rbuffer[1] = rbuffer[2]
                    rbuffer[2] = None
                else:
                    rtransforms.append(rbuffer[1])

                    rbuffer[0] = rbuffer[1]
                    rbuffer[1] = rbuffer[2]
                    rbuffer[2] = None

            next_waypoint = next_waypoint[0].next(self.sampling_distance)

        # Check last candidate with the end waypoint
        if end_waypoint and (lbuffer[1] != None and rbuffer[1] != None):
            lbuffer[2] = self._odr_map.get_border(end_waypoint, "left")
            rbuffer[2] = self._odr_map.get_border(end_waypoint, "right")

            if not _is_aligned(*lbuffer):
                ltransforms.append(lbuffer[1])
            if not _is_aligned(*rbuffer):
                rtransforms.append(rbuffer[1])

        return ltransforms, rtransforms

    def _get_link_points(self, road_id, section_id, lane_id):

        # Keep track of the visited segments so we don't revisit the same segment twice when searching a point link
        # candidate
        visited_segments = set()

        def _is_segment_visited(road_id, section_id, lane_id, clear=False):
            if clear:
                visited_segments.clear()

            if (road_id, section_id, lane_id) in visited_segments:
                return True
            else:
                visited_segments.add((road_id, section_id, lane_id))
                return False

        # Search start-left link point.
        #
        #                           *--------------------*
        #                           |    LEFT LANELET    |
        #                           |       ---->        |
        #                    point  |     ( <---- )      |
        #  *-----------------------(*)-------------------*
        #  |  PREDESSOR(s) LANELET  |  CURRENT LANELET   |
        #  |          ---->         |       ---->        |
        #  *------------------------*--------------------*
        #
        # When searching for the start-left link point:
        #
        #    * Check direct predecessors. It any direct predecessors has already been processed both left and right
        #      points must exists. If no predecessors have been processed look for the end-left point of each
        #      pedecessor.
        #
        #    * Check left neighbbour. Taking into account left neighbour lane direction and current lane direction:
        #
        #        - If SAME direction: Search start-right of the left neighbour.
        #        - If DIFFERENT direction: Search end-left of the left neighbour.

        def _get_start_left(road_id, section_id, lane_id, clear=False):
            if _is_segment_visited(road_id, section_id, lane_id, clear):
                return None

            # Check direct predecessors
            predecessors = self._odr_map.get_segment_predecessors(road_id, section_id, lane_id)
            
            # Check if any predecessor has already been processed
            processed_predecessors = [p for p in predecessors if p in self._odr2lanelet]
            
            # If a direct predecessor has been processed both left and right points must exist
            if processed_predecessors:
                lpoint, _ = self._lanelet2_map.get_lanelet_end_points(self._odr2lanelet[processed_predecessors[0]])
                return lpoint

            # If no predecessors have been processed look for the end-left point of each pedecessor
            else:
                for predecessor in predecessors:
                    point = _get_end_left(*predecessor)
                    if point: return point

            # Check left neighbbour.
            left = self._odr_map.get_left(road_id, section_id, lane_id)
            if left:
                _, _, left_lane_id = left
                if (lane_id * left_lane_id > 0):  # same direction
                    point = _get_start_right(*left)
                    if point: return point
                else:  # different direction
                    point = _get_end_left(*left)
                    if point: return point

        # Search start-right link point.
        #
        #  *------------------------*--------------------*
        #  |  PREDESSOR(s) LANELET  |  CURRENT LANELET   |
        #  |          ---->         |       ---->        |
        #  *-----------------------(*)-------------------*
        #                     point |   RIGHT LANELET    |
        #                           |       ---->        |
        #                           *--------------------*
        #
        # When searching for the start-right link point:
        #
        #    * Check direct predecessors. It any direct predecessors has already been processed both left and right
        #      points must exists. If no predecessors have been processed look for the end-right point of each
        #      pedecessor.
        #
        #    * Check right neighbbour and search for the start-left point. By denifition right neighbour lane direction
        #      and current lane direction is always the smae.

        def _get_start_right(road_id, section_id, lane_id, clear=False):
            if _is_segment_visited(road_id, section_id, lane_id, clear):
                return None

            # Check direct predecessors
            predecessors = self._odr_map.get_segment_predecessors(road_id, section_id, lane_id)

            # Check if any predecessor has already been processed
            processed_predecessors = [p for p in predecessors if p in self._odr2lanelet]

            # If a direct predecessor has been processed both left and right points must exist
            if processed_predecessors:
                _, rpoint = self._lanelet2_map.get_lanelet_end_points(self._odr2lanelet[processed_predecessors[0]])
                return rpoint

            # If no predecessors have been processed look for the end-right point of each pedecessor
            else:
                for predecessor in predecessors:
                    point = _get_end_right(*predecessor)
                    if point: return point

            # Check right neighbbour
            right = self._odr_map.get_right(road_id, section_id, lane_id)
            if right:
                _, _, right_lane_id = right
                assert lane_id * right_lane_id > 0
                point = _get_start_left(*right)
                if point: return point

        # Search end-left link point.
        #
        #  *------------------ *
        #  |    LEFT LANELET   |
        #  |       ---->       |
        #  |     ( <---- )     | point
        #  *------------------(*)------------------------*
        #  |  CURRENT LANELET  |  SUCCESSORS(s) LANELET  |
        #  |       ---->       |          ---->          |
        #  *-------------------*-------------------------*
        #
        # When searching for the end-left link point:
        #
        #    * Check direct successors. It any direct successor has already been processed both left and right points
        #      must exists. If no successors have been processed yet, look for the start-left point of each successor.
        #
        #    * Check left neighbbour. Taking into account left neighbour lane direction and current lane direction:
        #
        #        - If SAME direction: Search end-right of the left neighbour.
        #        - If DIFFERENT direction: Search start-left of the left neighbour.

        def _get_end_left(road_id, section_id, lane_id, clear=False):
            if _is_segment_visited(road_id, section_id, lane_id, clear):
                return None

            # Check direct successors
            successors = self._odr_map.get_segment_successors(road_id, section_id, lane_id)

            # Check if any successor has already been processed
            processed_successors = [s for s in successors if s in self._odr2lanelet]

            # If a direct successor has been processed both left and right points must exist
            if processed_successors:
                lpoint, _ = self._lanelet2_map.get_lanelet_start_points(self._odr2lanelet[processed_successors[0]])
                return lpoint

            # If no successors have been processed look for the start-left point of each pedecessor
            else:
                for successor in successors:
                    point = _get_start_left(*successor)
                    if point: return point

            # Check left neighbour
            left = self._odr_map.get_left(road_id, section_id, lane_id)
            if left:
                _, _, left_lane_id = left
                if (lane_id * left_lane_id > 0):  # same direction
                    point = _get_end_right(*left)
                    if point: return point
                else:  # different direction
                    point = _get_start_left(*left)
                    if point: return point

        # Search end-right link point.
        #
        #  *-------------------*-------------------------*
        #  |  CURRENT LANELET  |  SUCCESSORS(s) LANELET  |
        #  |       ---->       |          ---->          |
        #  *------------------(*)-------------------------*
        #  |   RIGHT LANELET   | point
        #  |       ---->       |
        #  *-------------------*
        #
        # When searching for the end-right link point:
        #
        #    * Check direct successors. It any direct successor has already been processed both left and right points
        #      must exists. If no successors have been processed yet, look for the start-right point of each successor.
        #
        #    * Check right neighbbour and search for the end-left point. By denifition right neighbour lane direction
        #      and current lane direction is always the smae.

        def _get_end_right(road_id, section_id, lane_id, clear=False):
            if _is_segment_visited(road_id, section_id, lane_id, clear):
                return None

            # Check direct successors
            successors = self._odr_map.get_segment_successors(road_id, section_id, lane_id)

            # Check if any successor has already been processed
            processed_successors = [s for s in successors if s in self._odr2lanelet]

            # If a direct successor has been processed both left and right points must exist
            if processed_successors:
                _, rpoint = self._lanelet2_map.get_lanelet_start_points(self._odr2lanelet[processed_successors[0]])
                return rpoint

            # If no successors have been processed look for the start-right point of each pedecessor
            else:
                for successor in successors:
                    point = _get_start_right(*successor)
                    if point: return point

            # Check right neighbour
            right = self._odr_map.get_right(road_id, section_id, lane_id)
            if right:
                _, _, right_lane_id = right
                assert lane_id * right_lane_id > 0
                point = _get_end_left(*right)
                if point: return point

        lstart = _get_start_left(road_id, section_id, lane_id, True)
        rstart = _get_start_right(road_id, section_id, lane_id, True)
        lend = _get_end_left(road_id, section_id, lane_id, True)
        rend = _get_end_right(road_id, section_id, lane_id, True)

        points = [
            (self._lanelet2_map.get_point(lstart), self._lanelet2_map.get_point(rstart)),
            (self._lanelet2_map.get_point(lend), self._lanelet2_map.get_point(rend))
        ]

        #print("{}: Linked points->".format(road_id), points)

        return points

    def _convert_crosswalk_to_lanelet(self, crosswalk):
        
        # A crosswalk is defined in the following way:
        #
        #            left border
        #    (p1)----------------->(p4)
        #     |                     |
        #     |      CROSSWLAK      |
        #     |                     |
        #    (p2)----------------->(p3)
        #           right border
        #
        # TODO: https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md#crosswalk

        p1, p2, p3, p4 = crosswalk

        left = [
            self._lanelet2_map.add_point(self._create_point(p1)),
            self._lanelet2_map.add_point(self._create_point(p4))
        ]
        right = [
            self._lanelet2_map.add_point(self._create_point(p2)),
            self._lanelet2_map.add_point(self._create_point(p3))
        ]

        linestrings = [
            self._lanelet2_map.add_linestring(lanelet2.Linestring(self._next_uid(), left, attributes={"type": "pedestrian_marking"})),
            self._lanelet2_map.add_linestring(lanelet2.Linestring(self._next_uid(), right, attributes={"type": "pedestrian_marking"}))
        ]

        self._lanelet2_map.add_lanelet(
            lanelet2.Lanelet(
                uid=self._next_uid(),
                left=linestrings[0],
                right=linestrings[1],
                attributes={
                    "type": "lanelet",
                    "subtype": "crosswalk",
                    "location": "urban",
                    "one_way": "no",
                    "speed_limit": "10",
                    "participant:pedestrian": "yes"}
            )
        )

    def _convert_traffic_light_to_regulatory_element(self, traffic_light):
        # https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md#trafficlights
        # https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md#light-bulbs-in-traffic-lights

        boxes, bulbs = [], []
        for box in traffic_light["boxes"]:

            # Linestring defining the traffic light box
            light_box = self._lanelet2_map.add_linestring(lanelet2.Linestring(
                uid=self._next_uid(),
                points=[
                    self._lanelet2_map.add_point(self._create_point(box["left"])),
                    self._lanelet2_map.add_point(self._create_point(box["right"]))
                ],
                attributes={
                    "type": "traffic_light",
                    "subtype": "red_yellow_green",
                    "height": box["height"]
                }
            ))
            boxes.append(light_box)

            # Linestring defining traffic light bulbs
            light_bulbs = self._lanelet2_map.add_linestring(lanelet2.Linestring(
                uid=self._next_uid(),
                points=[
                    self._lanelet2_map.add_point(self._create_point(box["bulbs"]["green"], extra_attributes={"color": "green"})),
                    self._lanelet2_map.add_point(self._create_point(box["bulbs"]["yellow"], extra_attributes={"color": "yellow"})),
                    self._lanelet2_map.add_point(self._create_point(box["bulbs"]["red"], extra_attributes={"color": "red"})),
                ],
                attributes={
                    "type": "light_bulbs",
                    "traffic_light_id": light_box
                }
            ))
            bulbs.append(light_bulbs)

        # For each landmark associated to this traffic light
        for waypoint in traffic_light["landmarks"]:
            segment = waypoint.road_id, waypoint.section_id, waypoint.lane_id

            # Linestring defining stop line.
            stop_line_waypoint = waypoint
            stop_line = self._lanelet2_map.add_linestring(lanelet2.Linestring(
                uid=self._next_uid(),
                points=[
                    self._lanelet2_map.add_point(self._create_point(self._odr_map.get_border(stop_line_waypoint, "left"))),
                    self._lanelet2_map.add_point(self._create_point(self._odr_map.get_border(stop_line_waypoint, "point"))),

                ],
                attributes={
                    "type": "stop_line"
                }
            ))

            # Regulatory element defining the traffic light
            regulatory_element = self._lanelet2_map.add_regulatory_element(lanelet2.RegulatoryElement(
                uid=self._next_uid(),
                parameters={
                    "refers": boxes,
                    "ref_line": [stop_line],
                    "light_bulbs": bulbs
                },
                attributes={
                    "type": "regulatory_element",
                    "subtype": "traffic_light"
                }
            ))

            # Add the regulatory element to the affected road lanelet
            lanelet = self._lanelet2_map.get_lanelet(self._odr2lanelet[segment])
            lanelet.add_regulatory_element(regulatory_element)            
            #print(lanelet.regulatory_elements)

    def validate(self):
        for road_id in self._odr_map.get_roads():
            for section_id in self._odr_map.get_sections(road_id):
                for lane_id in self._odr_map.get_lanes(road_id, section_id):

                    predecessors = self._odr_map.get_segment_predecessors(road_id, section_id, lane_id)
                    successors = self._odr_map.get_segment_successors(road_id, section_id, lane_id)

                    predecessors_points = []
                    for predecessor in predecessors:
                        predecessors_points += [self._lanelet2_map.get_lanelet_end_points(self._odr2lanelet[predecessor])]
                
                    successors_points = []
                    for successor in successors:
                        successors_points += [self._lanelet2_map.get_lanelet_start_points(self._odr2lanelet[successor])]

                    if not all(p == predecessors_points[0] for p in predecessors_points) or \
                       not all(s == successors_points[0] for s in successors_points):

                        logging.warning(
                            "Segment {}|{}|{} do not share the same points with all predecessors and/or successors.\n Predecessors: {}, Successors: {}".format(
                                road_id, section_id, lane_id,
                                predecessors_points,
                                successors_points
                            )
                        )

def odr2lanelet2(xodr_file, lanelet2_file, sampling_distance, use_carla_server):
    logging.info("Loading opendrive...")
    odr_map = opendrive.load_map(xodr_file, use_carla_server)

    conversor = Odr2Lanelet2Conversor(sampling_distance)
    lanelet2_map = conversor(odr_map)

    logging.info("Saving...")
    lanelet2.save(lanelet2_map, lanelet2_file)

    logging.info("""Conversion completed:
      * Total points {}
      * Total linestrings: {}
      * Total lanelets: {}""".format(
        len(lanelet2_map._points),
        len(lanelet2_map._linestrings),
        len(lanelet2_map._lanelets))
    )

    logging.info("Validating...")
    conversor.validate()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--input',
                           '-i',
                           help="input (*.xodr)")
    argparser.add_argument('--output',
                           '-o',
                           default="lanelet2.osm",
                           help="output (*.osm)")
    argparser.add_argument('--carla',
                           action='store_true',
                           help='use carla server')
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

    odr2lanelet2(args.input, args.output, sampling_distance=1.0, use_carla_server=args.carla)
