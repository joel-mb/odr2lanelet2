
class Primitive(object):
    def __init__(self, uid, attributes={}):
        self.uid = uid

        self.version = "1"
        self.visible = True

        self.attributes = attributes

    def get_attribute(self, key):
        return self.attributes.get(key, None)


class Point(Primitive):
    def __init__(self, uid, lat, lon, attributes={}):
        super(Point, self).__init__(uid, attributes)

        self.lat = lat
        self.lon = lon


class Linestring(Primitive):
    def __init__(self, uid, points, attributes={}):
        super(Linestring, self).__init__(uid, attributes)

        self.points = points


class Lanelet(Primitive):
    def __init__(self, uid, left, right, regulatory_elements=[], attributes={}):
        super(Lanelet, self).__init__(uid, attributes)

        # lanelet borders
        self.left = left
        self.right = right

        self.regulatory_elements = regulatory_elements

    def add_regulatory_element(self, regulatory_element):
        self.regulatory_elements.append(regulatory_element)


class RegulatoryElement(Primitive):

    def __init__(self, uid, parameters={}, attributes={}):
        super(RegulatoryElement, self).__init__(uid, attributes)

        self.parameters = parameters
