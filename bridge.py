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
        """
        https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LinestringTagging.md
        """
        _type = carla_marking.type
        _color = str(carla_marking.color).lower()
        if _type == carla.LaneMarkingType.NONE:
            return {'type': 'road_border'}

        elif _type == carla.LaneMarkingType.Other:
            return {'type': 'road_border'}

        elif _type == carla.LaneMarkingType.Broken:
            return {'type': 'line_thin', 'subtype': 'dashed', 'color': _color}

        elif _type == carla.LaneMarkingType.Solid:
            return {'type': 'line_thin', 'subtype': 'solid', 'color': _color}

        elif _type == carla.LaneMarkingType.SolidSolid:
            return {'type': 'line_thin', 'subtype': 'solid_solid', 'color': _color}

        # TODO(joel): Check SolidBroken or BrokenSolid
        elif _type == carla.LaneMarkingType.SolidBroken:
            return {'type': 'line_thin', 'subtype': 'solid_dashed', 'color': _color}

        # TODO(joel): Check BrokenSolid or SolidBroken
        elif _type == carla.LaneMarkingType.BrokenSolid:
            return {'type': 'line_thin', 'subtype': 'dashed_solid', 'color': _color}

        elif _type == carla.LaneMarkingType.BrokenBroken:
            return {'type': 'line_thin', 'subtype': 'dashed', 'color': _color}

        # TODO(joel): Is this the best translation. Autoware is using any convention?
        elif _type == carla.LaneMarkingType.BottsDots:
            return {'type': 'line_thin', 'subtype': 'solid', 'color': _color}

        # TODO(joel): It is necessary to add the color.
        elif _type == carla.LaneMarkingType.Grass:
            return {'type': 'road_border'}

        # TODO(joel): Check whether we use low or high.
        # TODO(joel): It is necessary to add the color.
        elif _type == carla.LaneMarkingType.Curb:
            return {'type': 'curbstone', 'subtype': 'low'}


    @staticmethod
    def lanelet2_lane(carla_lane):
        pass
