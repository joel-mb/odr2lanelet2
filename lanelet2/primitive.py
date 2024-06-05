

class Primitive(object):
    def __init__(self, uid, attributes={}):
        self.uid = uid
        self.attributes = attributes

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
    def __init__(self, uid, borders, attributes={}):
        super(Lanelet, self).__init__(uid, attributes)

        self.borders = borders

    def get_border(self, border):
        return self.borders[0] if border == "left" else self.borders[1]
