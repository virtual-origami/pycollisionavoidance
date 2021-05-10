from pycollisionavoidance.raycast.Point import Point, LineSegment
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/walkgen.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class Detection:
    def __init__(self, scene, particle, env_collision_distance, robot_collision_distance):
        """
        Initializes collision detection
        :param scene: scene object
        :param particle: particle object
        :param env_collision_distance: environment obstacle collision distance
        :param robot_collision_distance: robot (obstacle) collision distance
        """
        self.scene = scene
        self.particle = particle
        self.views = None
        self.env_collision_distance = env_collision_distance
        self.robot_collision_distance = robot_collision_distance

    def update_particles(self, x, y):
        """
        update particles position
        :param x: x axis value
        :param y: y axis value
        :return:
        """
        self.particle.update(x=x, y=y)

    def get_view(self):
        """
        get view of the scene around
        :return:
        """
        return self.views

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

    def ranging(self):
        """
        range (measure distances) from the obstacles
        :return:
        """
        result = []
        robot_control_msg = []

        self.views = self.particle.look(self.scene.get_segments())

        for item in self.views:
            if item['distance'] is not None:
                if item['distance'] > self.env_collision_distance:
                    result.append(item)

        for item in self.views:
            if item['distance'] < self.robot_collision_distance:
                view_substring = item['obstacle'].split("_")
                if "robot" in view_substring:
                    robot_control_msg.append({"id": view_substring[1], "control": "stop"})
        return result, robot_control_msg


