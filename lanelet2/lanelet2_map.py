#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import datetime
import os

import lxml.etree as ET


class Lanelet2Map(object):
    def __init__(self):
        self._points = {}
        self._linestrings = {}
        self._lanelets = {}

    def get_point(self, uid):
        return self._points.get(uid, None)

    def get_linestring(self, uid):
        return self._linestrings.get(uid, None)

    def get_lanelet(self, uid):
        return self._lanelets.get(uid, None)

    def get_points(self):
        return self._points.values()

    def get_linestrings(self):
        return self._linestrings.values()

    def get_lanelets(self):
        return self._lanelets.values()

    def add_point(self, point):
        self._points[point.uid] = point
        return point.uid

    def add_linestring(self, linestring):
        self._linestrings[linestring.uid] = linestring
        return linestring.uid

    def add_lanelet(self, lanelet):
        self._lanelets[lanelet.uid] = lanelet
        return lanelet.uid


def save(lanelet2_map, filename):
    def _create_attributes_tags(attributes, parent):
        for key, value in attributes.items():
            ET.SubElement(parent, 'tag', {'k': key, 'v': str(value)})

    def _create_node_tag(point):
        node_tag = ET.SubElement(
            root, 'node', {
                'id': str(point.uid),
                'visible': str(point.visible),
                'version': '1',
                'lat': str(point.lat),
                'lon': str(point.lon)
            })

        _create_attributes_tags(point.attributes, node_tag)

    def _create_way_tag(linestring):
        way_tag = ET.SubElement(root, 'way', {
            'id': str(linestring.uid),
            'visible': 'true',
            'version': '1'
        })
        for point in linestring.points:
            nd_tag = ET.SubElement(way_tag, 'nd', {'ref': str(point)})

        _create_attributes_tags(linestring.attributes, way_tag)

    def _create_relation_tag(lanelet):
        relation_tag = ET.SubElement(root, 'relation', {
            'id': str(lanelet.uid),
            'visible': 'true',
            'version': '1'
        })

        left_tag = ET.SubElement(relation_tag, 'member', {
            'type': 'way',
            'ref': str(lanelet.borders[0]),
            'role': 'left'
        })
        right_tag = ET.SubElement(relation_tag, 'member', {
            'type': 'way',
            'ref': str(lanelet.borders[1]),
            'role': 'right'
        })

        _create_attributes_tags(lanelet.attributes, relation_tag)

    root = ET.Element('osm', {"version": "0.6"})
    root.addprevious(
        ET.Comment('generated on {date:%Y-%m-%d %H:%M:%S} by {script:}'.format(
            date=datetime.datetime.now(), script=os.path.basename(__file__))))

    map(_create_node_tag, lanelet2_map.get_points())
    map(_create_way_tag, lanelet2_map.get_linestrings())
    map(_create_relation_tag, lanelet2_map.get_lanelets())

    tree = ET.ElementTree(root)
    tree.write(filename,
               pretty_print=True,
               encoding='UTF-8',
               xml_declaration=True)
