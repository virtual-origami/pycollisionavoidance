import sys
import time
from pycollisionavoidance.raycast.Point import Point, LineSegment
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/walkgen.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class ParticleCollisionDetection:
    """
    This class ranges the distance between following using 2D ray cast algorithm
    1. particle (human worker) and environment (static obstacles like walls etc)
    2. particle (human worker) and robot (dynamic obstacles)
    """

    def __init__(self, scene, particle, env_collision_distance, robot_collision_distance):
        """
        Initializes collision detection
        :param scene: scene object
        :param particle: particle object
        :param env_collision_distance: environment obstacle collision distance
        :param robot_collision_distance: robot (obstacle) collision distance
        """
        self.scene = scene
        self.id = particle.id
        self.particle = particle
        self.views = None
        self.env_collision_distance = env_collision_distance
        self.robot_collision_distance = robot_collision_distance
        self.time_now = 0
        self.time_past = 0
        self.robot_collision = []
        self.env_collision = []

    def update_particles(self, x, y):
        """
        update particles position
        :param x: x axis value
        :param y: y axis value
        :return:
        """
        self.particle.update(x=x, y=y)

    def update_scene(self, obstacle_id, points, shape=None):
        """
        update scene with obstacle coordinates
        :param obstacle_id: obstacle id
        :param points: coordinate points
        :param shape: shape of the obstacle
        :return:
        """
        if shape == "line" and len(points) == 2:
            self.scene.update(obstacle_id=obstacle_id,
                              corner_points=(Point(x=points[0][0], y=points[0][1]),
                                             Point(x=points[1][0], y=points[1][1])),
                              shape=shape)
        elif shape == "polygon" and len(points) > 2:
            corner_points = []
            for point in points:
                corner_points.append(Point(x=point[0], y=point[1]))
            self.scene.update(obstacle_id=obstacle_id,
                              corner_points=corner_points,
                              shape=shape)

    def get_environmental_collision_distance(self):
        """
        get environmental collision distance between static obstacles and particle (human worker)
        :return: array consisting of map {obstacle id ,distance from the particle}
        """
        result = []
        for item in self.views:
            if item['distance'] is not None:
                if item['distance'] > self.env_collision_distance:
                    result.append(item)
        return result

    def get_robot_collision_distance(self):
        """
        get robot collision distance between robot and particle (human worker)
        :return:  array consisting of {robot id,control message}
        """
        robot_control_msg = []
        for item in self.views:
            if item['distance'] is not None:
                if item['distance'] < self.robot_collision_distance:
                    view_substring = item['obstacle'].split("_")
                    if "robot" in view_substring:
                        robot_control_msg.append({"id": view_substring[1], "control": "stop"})
        return robot_control_msg

    def ranging(self):
        """
        range (measure distances) from the obstacles.
        Ranging is done 360 degree with 1 degree increment about a particle
        and ranging result is added to an distance
        :return:
        """
        result = []
        robot_control_msg = []

        # ranging about the particle
        self.views = self.particle.look(self.scene.get_segments())

        # get environment collision distance
        env_collision_distance = self.get_environmental_collision_distance()

        # get robot collision distance
        robot_collision_msg = self.get_robot_collision_distance()

        return env_collision_distance, robot_collision_msg

    def get_view(self):
        """
        get view of the scene around
        :return:
        """
        return self.views

    async def update(self, tdelta=-1):
        """
        update walker position in 3D
        :param tdelta: time duration between successive updates
        :return:
        """
        try:
            # calculate loop time
            if tdelta > 0:
                # valid time delta received as input paramter
                timedelta = tdelta
            elif self.time_now == 0 and self.time_past == 0:
                # time delta calculation for first update cycle
                self.time_now = time.time()
                timedelta = self.interval
            else:
                # time delta calculation based on run time
                self.time_now = time.time()
                timedelta = self.time_now - self.time_past
                self.time_past = self.time_now

            assert (timedelta >= 0), f"Time delta: {timedelta},  can't be negative"

            # Calculate Walk angle for next step, and also check if walker is in collision course
            self.env_collision, self.robot_collision = self.ranging()

        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)
