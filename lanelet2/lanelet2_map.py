import datetime
import os

import lxml.etree as ET


class Lanelet2Map(object):
    def __init__(self):
        self._points = {}
        self._linestrings = {}
        self._lanelets = {}
        self._regulatory_elements = {}

        self._primitives = {}

    def get_point(self, uid):
        return self._points.get(uid, None)

    def get_linestring(self, uid):
        return self._linestrings.get(uid, None)

    def get_regulatory_element(self, uid):
        return self._regulatory_elements.get(uid, None)

    def get_lanelet(self, uid):
        return self._lanelets.get(uid, None)

    def get_lanelet_start_points(self, uid):
        lanelet = self.get_lanelet(uid)
        edges = (self.get_linestring(lanelet.left).points,
                 self.get_linestring(lanelet.right).points)
        return (edges[0][0], edges[1][0])

    def get_lanelet_end_points(self, uid):
        lanelet = self.get_lanelet(uid)
        edges = (self.get_linestring(lanelet.left).points,
                 self.get_linestring(lanelet.right).points)
        return (edges[0][-1], edges[1][-1])

    def get_points(self):
        return self._points.values()

    def get_linestrings(self):
        return self._linestrings.values()

    def get_lanelets(self):
        return self._lanelets.values()

    def get_regulatory_elements(self):
        return self._regulatory_elements.values()

    def add_point(self, point):
        if not point.uid in self._points:
            self._points[point.uid] = point
        return point.uid

    def add_linestring(self, linestring):
        self._linestrings[linestring.uid] = linestring
        return linestring.uid

    def add_lanelet(self, lanelet):
        self._lanelets[lanelet.uid] = lanelet
        return lanelet.uid

    def add_regulatory_element(self, regulatory_element):
        self._regulatory_elements[regulatory_element.uid] = regulatory_element
        return regulatory_element.uid


def save(lanelet2_map, filename):
    def _serialize_attributes(attributes, parent):
        for key, value in attributes.items():
            ET.SubElement(parent, "tag", {"k": key, "v": str(value)})

    def _serialize_point(point):
        node_tag = ET.SubElement(
            root, "node", {
                "id": str(point.uid),
                "visible": "true" if point.visible else "false",
                "version": str(point.version),
                "lat": str(point.lat),
                "lon": str(point.lon)
            })

        _serialize_attributes(point.attributes, node_tag)

    def _serialize_linestring(linestring):
        way_tag = ET.SubElement(root, "way", {
            "id": str(linestring.uid),
            "visible": "true" if linestring.visible else "false",
            "version": str(linestring.version),
        })
        for point in linestring.points:
            _ = ET.SubElement(way_tag, "nd", {"ref": str(point)})

        _serialize_attributes(linestring.attributes, way_tag)

    def _serialize_lanelet(lanelet):

        relation_tag = ET.SubElement(root, "relation", {
            "id": str(lanelet.uid),
            "visible": "true" if lanelet.visible else "false",
            "version": str(lanelet.version),
        })

        _ = ET.SubElement(relation_tag, "member", {
            "type": "way",
            "ref": str(lanelet.left),
            "role": "left"
        })
        _ = ET.SubElement(relation_tag, "member", {
            "type": "way",
            "ref": str(lanelet.right),
            "role": "right"
        })

        for regulatory_element in lanelet.regulatory_elements:
            _ = ET.SubElement(relation_tag, "member", {
                "type": "relation",
                "ref": str(regulatory_element),
                "role": "regulatory_element"
            })

        _serialize_attributes(lanelet.attributes, relation_tag)

    def _serialize_regulatory_element(regulatory_element):
        relation_tag = ET.SubElement(root, "relation", {
            "id": str(regulatory_element.uid),
            "visible": "true" if regulatory_element.visible else "false",
            "version": str(regulatory_element.version),
        })

        for role in regulatory_element.parameters.keys():
            for ref in regulatory_element.parameters[role]:
                _ = ET.SubElement(relation_tag, "member", {
                    "type": "way" if str(ref) in lanelet2_map._linestrings else "relation",
                    "ref": str(ref),
                    "role": role
                })

        _serialize_attributes(regulatory_element.attributes, relation_tag)

    root = ET.Element("osm", {"version": "0.6"})
    root.addprevious(
        ET.Comment("generated on {date:%Y-%m-%d %H:%M:%S} by {script:}".format(
            date=datetime.datetime.now(), script=os.path.basename(__file__))))

    list(map(_serialize_point, lanelet2_map.get_points()))
    list(map(_serialize_linestring, lanelet2_map.get_linestrings()))
    list(map(_serialize_lanelet, lanelet2_map.get_lanelets()))
    list(map(_serialize_regulatory_element, lanelet2_map.get_regulatory_elements()))

    tree = ET.ElementTree(root)
    tree.write(filename,
               pretty_print=True,
               encoding="UTF-8",
               xml_declaration=True)
