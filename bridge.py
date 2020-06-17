#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla

class Bridge(object):

    @staticmethod
    def lanelet2_marking(carla_marking):
        _type = carla_marking.type
        if _type == carla.LaneMarkingType.NONE:
            return {'type': 'road_border'}

        elif _type == carla.LaneMarkingType.Other:
            return {'type': 'road_border'}

        elif _type == carla.LaneMarkingType.Broken:
            return {'type': 'line_thin', 'subtype': 'dashed'}

        elif _type == carla.LaneMarkingType.Solid:
            return {'type': 'line_thin', 'subtype': 'solid'}

        elif _type == carla.LaneMarkingType.SolidSolid:
            return {'type': 'line_thin', 'subtype': 'solid_solid'}

        elif _type == carla.LaneMarkingType.SolidBroken:
            return {'type': 'line_thin', 'subtype': 'solid_dashed'}

        elif _type == carla.LaneMarkingType.BrokenSolid:
            return {'type': 'line_thin', 'subtype': 'dashed_solid'}

        elif _type == carla.LaneMarkingType.BrokenBroken:
            return {'type': 'line_thin', 'subtype': 'dashed'}

        elif _type == carla.LaneMarkingType.BottsDots:
            return {'type': 'line_thin', 'subtype': 'dashed'}

        elif _type == carla.LaneMarkingType.Grass:
            return {'type': 'line_thin', 'subtype': 'dashed'}

        elif _type == carla.LaneMarkingType.Curb:
            return {'type': 'line_thin', 'subtype': 'dashed'}


    @staticmethod
    def lanelet2_lane(carla_lane):
        pass
