import carla

class Bridge(object):

    @staticmethod
    def lanelet2_marking(carla_marking, has_neighbour=False):
        _type = carla_marking.type

        if _type == carla.LaneMarkingType.NONE:
            return {
                'type': 'road_border'
            }

        elif _type == carla.LaneMarkingType.Other:
            return {
                'type': 'road_border'
            }

        elif _type == carla.LaneMarkingType.Broken:
            return {
                'type': 'line_thin',
                'subtype': 'dashed',
                "lane_change": 'yes' if has_neighbour else 'no'
            }

        elif _type == carla.LaneMarkingType.Solid:
            return {
                'type': 'line_thin',
                'subtype': 'solid'
            }

        elif _type == carla.LaneMarkingType.SolidSolid:
            return {
                'type': 'line_thin',
                'subtype': 'solid_solid'
            }

        elif _type == carla.LaneMarkingType.SolidBroken:
            return {
                'type': 'line_thin',
                'subtype': 'solid_dashed',
                'lane_change:right': 'no',
                'lane_change:left': 'yes' if has_neighbour else 'no'
            }

        elif _type == carla.LaneMarkingType.BrokenSolid:
            return {
                'type': 'line_thin',
                'subtype': 'dashed_solid',
                'lane_change:right': 'yes' if has_neighbour else 'no',
                'lane_change:left': 'no'
            }

        elif _type == carla.LaneMarkingType.BrokenBroken:
            return {
                'type': 'line_thin',
                'subtype': 'dashed',
                "lane_change": 'yes' if has_neighbour else 'no'
            }

        elif _type == carla.LaneMarkingType.BottsDots:
            return {
                'type': 'line_thin',
                'subtype': 'dashed',
                "lane_change": 'yes' if has_neighbour else 'no'
            }

        elif _type == carla.LaneMarkingType.Grass:
            return {
                'type': 'line_thin',
                'subtype': 'solid',
            }

        elif _type == carla.LaneMarkingType.Curb:
            return {
                'type': 'line_thin',
                'subtype': 'solid'
            }

    @staticmethod
    def lanelet2_lane(carla_lane):
        pass
