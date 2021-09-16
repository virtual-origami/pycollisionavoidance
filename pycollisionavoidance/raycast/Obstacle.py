import logging
import sys
import traceback
from pycollisionavoidance.raycast.Point import LineSegment, Dot

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/walkgen.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class Obstacle:
    """
    This class implements the defining Obstacles
    """

    def __init__(self, id, corner_points, obstacle_shape, obstacle_type, description=""):
        """
        Initialization of obstacle
        :param id: obstacle ID
        :param corner_points: corner coordinate points of obstacle mentioned in a cyclic order
        :param obstacle_shape: shape of the obstacle
        :param obstacle_type: type of the obstacle. Obstacle can be line , polygon
        :param description: Description about the obstacle
        """
        try:
            assert type(corner_points) == tuple, "Corner point list must be a list of Points"
            self.id = id
            self.num_of_points = len(corner_points)
            self.corner_points = corner_points
            self.description = description
            self.shape = obstacle_shape
            self.type = obstacle_type
            self.line_segments = []

            # add segments of the obstacle based on the shape of the obstacle
            if obstacle_shape == 'polygon':
                for i in range(0, self.num_of_points - 1):
                    self.line_segments.append(LineSegment(point1=corner_points[i],
                                                          point2=corner_points[i + 1],
                                                          description=self.description))
                self.line_segments.append(LineSegment(point1=corner_points[self.num_of_points - 1],
                                                      point2=corner_points[0],
                                                      description=self.description))

            if obstacle_shape == 'line':
                self.line_segments.append(LineSegment(point1=corner_points[0],
                                                      point2=corner_points[1],
                                                      description=self.description))

        except AssertionError as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()
        except ValueError as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()
        except Exception as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()

    def update(self, corner_points, shape=None):
        """
        Update coordinate of the obstacle.
        :param corner_points: corner coordinate points of obstacle mentioned in a cyclic order
        :param shape: shape of the obstacle (optional). No need to mention if shape of the obstacle has not changed
        :return:
        """
        try:
            assert type(corner_points) == tuple, "Corner point list must be a list of Points"
            self.corner_points = corner_points
            self.num_of_points = len(corner_points)
            if shape is not None:
                self.shape = shape
            self.line_segments.clear()
            if self.shape == 'polygon':
                for i in range(0, self.num_of_points - 1):
                    self.line_segments.append(LineSegment(point1=corner_points[i], point2=corner_points[i + 1]))
                self.line_segments.append(
                    LineSegment(point1=corner_points[self.num_of_points - 1], point2=corner_points[0],
                                description=self.description))
            elif self.shape == 'line':
                self.line_segments.append(
                    LineSegment(point1=corner_points[0], point2=corner_points[1], description=self.description))
            else:
                self.line_segments.append(
                    LineSegment(point1=corner_points[0], point2=corner_points[0], description=self.description))
        except AssertionError as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()
        except ValueError as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()
        except Exception as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()
