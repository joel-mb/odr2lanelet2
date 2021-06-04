#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


class Primitive(object):
    def __init__(self, uid, attributes={}):
        self.uid = uid
        self.attributes = attributes

    def add_attribute(self, key, value):
        self.attributes[key] = value

    def get_attribute(self, key):
        return self.attributes.get(key, None)


class Point(Primitive):
    def __init__(self, uid, lat, lon, attributes={}):
        super(Point, self).__init__(uid, attributes)

        self.lat = lat
        self.lon = lon

        self.version = "1"
        self.visible = True


class Linestring(Primitive):
    def __init__(self, uid, points, attributes={}):
        super(Linestring, self).__init__(uid, attributes)

        self.points = points


class Lanelet(Primitive):

    class Member(object):
        def __init__(self, type_, ref, role):
            self.type_ = type_
            self.ref = ref
            self.role = role

    def __init__(self, uid, borders, attributes={}):
        super(Lanelet, self).__init__(uid, attributes)

        self.borders = borders
        self.regulatory_elements = []
        self.members = []

    def get_border(self, border):
        return self.borders[0] if border == "left" else self.borders[1]

    def add_regulatory_element(self, regulatory_element):
        self.regulatory_elements.append(regulatory_element)

    def add_member(self, member):
        self.members.append(member)


# TODO(joel): Should inherit from Lanelet? Or from Relation (general one)
class RegulatoryElement(Primitive):
    def __init__(self, uid, refers=[], ref_line=None, attributes={}):
        super(RegulatoryElement, self).__init__(uid, attributes)

        self.refers = refers
        self.ref_line = ref_line

        self.members = []

    def add_member(self, member):
        self.members.append(member)


class TrafficLight(RegulatoryElement):
    def __init__(self, uid, refers=[], ref_line=None):
        pass
