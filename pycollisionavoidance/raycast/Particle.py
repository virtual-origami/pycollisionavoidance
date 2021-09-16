import math
import logging
from pycollisionavoidance.raycast.Point import Point
from pycollisionavoidance.raycast.Ray import Ray

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/walkgen.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class Particle:
    """
    This class implements the Particle representing a personnel as point
    """
    def __init__(self,particle_id,x,y,size_in_pixel=1):
        """
        Initializes the particle
        :param particle_id: particle id
        :param x: x cordinates of the particle
        :param y: y coordinate of the particle
        :param size_in_pixel: size of the particle in pixels
        """
        self.id = particle_id
        self.pos = Point(x=x,y=y)
        self.size_in_pixel = size_in_pixel
        self.rays = []

        # creates a rays in 360 degree with 1 degree resolution
        for i in range(0,360):
            self.rays.append(Ray(origin=self.pos,angle=i))

    def update(self,x,y):
        """
        Update poistion coordinate of the particle
        :param x: x coordinate of the particle
        :param y: y coordinate of the particle
        :return:
        """
        if x is not None and y is not None:
            self.pos.x = x
            self.pos.y = y

    def look(self,segments):
        """
        look the world around for the obstacles and do distance ranging
        :param segments: list of obstacle segments from the world
        :return:
        """
        result = []
        if self.pos.x is not None and self.pos.y is not None:
            for ray in self.rays:
                closest_obstacle = None
                closest_distance = None
                contact_point = None
                for obstacle in segments:
                    pt = ray.cast(obstacle)
                    if pt is not None:
                        distance = abs(math.sqrt(((self.pos.x - pt.x) ** 2) + ((self.pos.y - pt.y) ** 2)))
                        if closest_obstacle is None:
                            closest_obstacle = obstacle
                            closest_distance = distance
                            contact_point = pt
                        else:
                            if distance < closest_distance:
                                closest_obstacle = obstacle
                                closest_distance = distance
                                contact_point = pt
                result.append({
                    'contact_point':[contact_point.x,contact_point.y]if contact_point is not None else None,
                    "angle":ray.angle,
                    "obstacle":closest_obstacle.description if closest_obstacle is not None else None,
                    "distance":closest_distance
                })
        return result

    def look_at_angle(self,segments,start_angle,stop_angle):
        """
        look the world around for the obstacles between start and stop angle and do distance ranging
        :param segments: list of obstacle segments from the world
        :param start_angle: start angle
        :param stop_angle: stop angle
        :return:
        """
        result = []
        if self.pos.x is not None and self.pos.y is not None:
            for ray in self.rays:
                if start_angle <= ray.angle <= stop_angle:
                    closest_obstacle = None
                    closest_distance = None
                    for obstacle in segments:
                        pt = ray.cast(obstacle)
                        if pt is not None:
                            distance = abs(math.sqrt(((self.pos.x - pt.x) ** 2) + ((self.pos.y - pt.y) ** 2)))
                            if closest_obstacle is None:
                                closest_obstacle = obstacle
                                closest_distance = distance
                            else:
                                if distance < closest_distance:
                                    closest_obstacle = obstacle
                                    closest_distance = distance
                result.append({
                    "angle":ray.angle,
                    "obstacle":closest_obstacle.description if closest_obstacle is not None else None,
                    "distance":closest_distance
                })
        return result
