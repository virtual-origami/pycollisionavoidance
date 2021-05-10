import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/walkgen.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class Point:
    """
    This class implements Coordinate points
    """
    def __init__(self, x, y):
        """
        Initializes coordinate points
        :param x: x coordinate
        :param y: y coordinate
        """
        try:
            self.x = x
            self.y = y
        except AssertionError as e:
            logging.critical(e)
            exit()
        except ValueError as e:
            logging.critical(e)
            exit()
        except Exception as e:
            logging.critical(e)
            exit()


class LineSegment:
    """
    This class implements Line segment
    """
    def __init__(self, point1, point2, description=""):
        """
        Initialize Line segment. A line segment needs two points
        :param point1: coordinates of first point
        :param point2: coordinate of second point
        :param description: description about line segment
        """
        try:
            assert type(point1) == Point, "point1 must be of class Point"
            assert type(point2) == Point, "point2 must be of class Point"
            self.a = point1
            self.b = point2
            self.description = description
        except AssertionError as e:
            logging.critical(e)
            exit()
        except ValueError as e:
            logging.critical(e)
            exit()
        except Exception as e:
            logging.critical(e)
            exit()


class Dot:
    def __init__(self, point, description=""):
        """
        Initialize Dot. A Dot needs a point
        :param point: coordinates of point
        :param description: Description about a Dot
        """
        try:
            assert type(point) == Point, "point1 must be of class Point"
            self.a = point
            self.description = description
        except AssertionError as e:
            logging.critical(e)
            exit()
        except ValueError as e:
            logging.critical(e)
            exit()
        except Exception as e:
            logging.critical(e)
            exit()
