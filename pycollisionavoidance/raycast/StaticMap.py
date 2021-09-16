import logging
import sys
from pycollisionavoidance.raycast.Obstacle import Obstacle
from pycollisionavoidance.raycast.Point import Point

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/collision avoidance.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class StaticMap:
    """
    Static obstacle map implementation
    """
    def __init__(self, config_file):
        """
        Initialization of static obstacle map
        :param config_file: configuration file
        """
        try:
            self.obstacles = []
            obstacles = config_file["obstacles"]
            robots = config_file["robots"]
            for obstacle in obstacles:
                points = []
                for point in obstacle["points"]:
                    points.append(Point(x=point[0], y=point[1]))
                self.obstacles.append(Obstacle(id=obstacle["id"],
                                               corner_points=tuple(points),
                                               obstacle_shape=obstacle["render"]["shape"],
                                               obstacle_type=obstacle["render"]["type"],
                                               description=obstacle["description"]))
            for robot in robots:
                center_x = robot["base"]['x']
                center_y = robot["base"]['y']
                points = [Point(x=center_x - 2, y=center_y - 2),
                          Point(x=center_x + 2, y=center_y - 2),
                          Point(x=center_x + 2, y=center_y + 2),
                          Point(x=center_x + 2, y=center_y - 2)]
                arm = points = [Point(x=center_x - 2, y=center_y - 2), Point(x=center_x + 2, y=center_y - 2)]
                self.obstacles.append(Obstacle(id=obstacle["id"],
                                               corner_points=tuple(points),
                                               obstacle_shape='polygon',
                                               obstacle_type='static',
                                               description="robot_" + robot['id']))
                self.obstacles.append(Obstacle(id="robot_" + robot['id']+"_base_shoulder",
                                               corner_points=tuple(arm),
                                               obstacle_shape='line',
                                               obstacle_type='dynamic',
                                               description="robot_" + robot['id']+"_base_shoulder"))
                self.obstacles.append(Obstacle(id="robot_" + robot['id'] + "_shoulder_elbow",
                                               corner_points=tuple(arm),
                                               obstacle_shape='line',
                                               obstacle_type='dynamic',
                                               description="robot_" + robot['id'] + "_shoulder_elbow"))
                self.obstacles.append(Obstacle(id="robot_" + robot['id'] + "_elbow_wrist",
                                               corner_points=tuple(arm),
                                               obstacle_shape='line',
                                               obstacle_type='dynamic',
                                               description="robot_" + robot['id'] + "_elbow_wrist"))
        except AssertionError as e:
            logging.critical(e)
            sys.exit()
        except Exception as e:
            logging.critical(e)
            sys.exit()

    def update(self, obstacle_id, corner_points, shape=None):
        """
        Update obstacle information in the world
        :param obstacle_id:Obstacle ID
        :param corner_points: Coordinate points of the obstacle
        :param shape: shape of the obstacle (Shape is optional and is mentioned only when shape is changed)
        :return:
        """
        try:
            assert type(corner_points) == tuple, "Corner points must be tuple of Points"
            for idx, obstacle in enumerate(self.obstacles):
                if obstacle.id == obstacle_id:
                    obstacle.update(corner_points=corner_points, shape=shape)
                    break
        except Exception as e:
            logging.critical(e)
            sys.exit()

    def get_segments(self):
        """
        get all segments in world view
        :return: segments
        """
        segments = []
        for obstacle in self.obstacles:
            for segment in obstacle.line_segments:
                segments.append(segment)
        return segments
