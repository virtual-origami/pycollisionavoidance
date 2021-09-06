# Python code for 2D random walk.
import json
import sys
import random
import time
import math
import logging
import asyncio
from pycollisionavoidance.pub_sub.AMQP import PubSubAMQP
from pycollisionavoidance.raycast.Particle import Particle
from pycollisionavoidance.raycast.StaticMap import StaticMap
from pycollisionavoidance.collision.Detection import ParticleCollisionDetection

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/walkgen.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


# ========================================= WALK PATTERN GENERATOR ===================================================

class CollisionAvoidance:
    def __init__(self, eventloop, config_file, personnel_id):
        """
        Initialize Collision Avoidance
        :param eventloop: event loop for amqp pub sub
        :param config_file: configuration file
        """
        try:
            self.workspace_attributes = config_file["workspace"]
            self.walkers_in_ws = []
            self.interval = self.workspace_attributes["update_interval"]
            self.publishers = []
            self.subscribers = []

            protocol = config_file["protocol"]
            # check for protocol key
            if "protocol" not in config_file:
                logger.critical("no 'protocol' key found.")
                sys.exit(-1)

            # Personnel instantiation
            for each_walker in config_file["personnels"]:
                # create walker
                walker_id = personnel_id
                # walker_id = each_walker["id"]
                # pos = {'x': each_walker["start_coordinates"]["x"],
                #        'y': each_walker["start_coordinates"]["y"],
                #        'z': each_walker["start_coordinates"]["z"]}
                pos = {'x': None, 'y': None, 'z': None}
                env_collision_distance = each_walker["attribute"]["collision"]["distance"]["environment"]
                robot_collision_distance = each_walker["attribute"]["collision"]["distance"]["robot"]

                # Collision detection
                walker = ParticleCollisionDetection(scene=StaticMap(config_file=self.workspace_attributes),
                                                    particle=Particle(particle_id=walker_id, x=pos["x"], y=pos["y"]),
                                                    env_collision_distance=env_collision_distance,
                                                    robot_collision_distance=robot_collision_distance)

                self.walkers_in_ws.append(walker)

            # Publisher
            if protocol["publishers"] is not None:
                for publisher in protocol["publishers"]:
                    if publisher["type"] == "amq":
                        logger.debug('Setting Up AMQP Publisher for Robot')
                        self.publishers.append(
                            PubSubAMQP(
                                eventloop=eventloop,
                                config_file=publisher,
                                binding_suffix=""
                            )
                        )
                    else:
                        logger.error("Provide protocol amq config")
                        raise AssertionError("Provide protocol amq config")

            # Subscriber
            if protocol["subscribers"] is not None:
                for subscriber in protocol["subscribers"]:
                    if subscriber["type"] == "amq":
                        logger.debug('Setting Up AMQP Subcriber for Robot')
                        self.subscribers.append(
                            PubSubAMQP(
                                eventloop=eventloop,
                                config_file=subscriber,
                                binding_suffix="",
                                app_callback=self._consume_telemetry_msg
                            )
                        )

                    else:
                        logger.error("Provide protocol amq config")
                        raise AssertionError("Provide protocol amq config")

        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)

    async def update(self):
        """
        update walk generator.
        Note This function need to be called in a loop every update cycle
        :param binding_key: binding key name (optional) used when other than default binding key
        :return:
        """
        try:
            for walker in self.walkers_in_ws:
                if self.interval >= 0:
                    await walker.update(tdelta=self.interval)
                else:
                    await walker.update()

                # collision avoidance
                if len(walker.robot_collision) > 0:
                    for msg in walker.robot_collision:
                        await self.publish(exchange_name="control_robot",
                                           msg=json.dumps(msg).encode(),
                                           external_binding_suffix=msg["id"])

            # sleep until its time for next sample
            if self.interval >= 0:
                await asyncio.sleep(delay=self.interval)
            else:
                await asyncio.sleep(delay=0)
        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)

    def get_states(self):
        return {"x_ref_pos": self.pos['x'], "y_ref_pos ": self.pos['y'], "z_ref_pos": self.pos['z']}

    async def publish(self, exchange_name, msg, external_binding_suffix=None):
        '''
        publishes amqp message
        :param exchange_name: name of amqp exchange
        :param msg: message to be published
        :param external_binding_suffix: binding suffix. suffix is appended to the end of binding namedd
        :return:
        '''
        try:
            for publisher in self.publishers:
                if exchange_name == publisher.exchange_name:
                    await publisher.publish(message_content=msg, external_binding_suffix=external_binding_suffix)
                    logger.debug(f'pub: {msg}')
        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)

    async def connect(self):
        """
        connects amqp publishers and subscribers
        :return:
        """
        try:
            for publisher in self.publishers:
                await publisher.connect()

            for subscriber in self.subscribers:
                await subscriber.connect(mode="subscriber")
        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)

    def _consume_telemetry_msg(self, **kwargs):
        """
        consume telemetry messages
        :param kwargs: must contain following information
                       1.   exchange_name
                       2.   binding_name
                       3.   message_body
        :return: none
        """
        # extract message attributes from message
        exchange_name = kwargs["exchange_name"]
        binding_name = kwargs["binding_name"]
        message_body = json.loads(kwargs["message_body"])

        # check for matching subscriber with exchange and binding name in all subscribers
        for subscriber in self.subscribers:
            if subscriber.exchange_name == exchange_name:
                if "rmt.robot." in binding_name:
                    # extract robot id from binding name
                    binding_delimited_array = binding_name.split(".")
                    robot_id = binding_delimited_array[len(binding_delimited_array) - 1]
                    msg_attributes = message_body.keys()

                    # check for must fields in the message attributes
                    if ("id" in msg_attributes) and ("base" in msg_attributes) \
                            and ("shoulder" in msg_attributes) and ("elbow" in msg_attributes):

                        # check if robot id matches with 'id' field in the message
                        if robot_id == message_body["id"]:
                            # extract information from message body
                            base_shoulder = [message_body["base"], message_body["shoulder"]]
                            shoulder_elbow = [message_body["shoulder"], message_body["elbow"]]
                            elbow_wrist = [message_body["elbow"], message_body["wrist"]]
                            prefix = "robot_" + message_body["id"]
                            for walker in self.walkers_in_ws:
                                # update robot in scene for collision detection
                                walker.update_scene(obstacle_id=prefix + "_base_shoulder",
                                                    points=base_shoulder,
                                                    shape="line")
                                walker.update_scene(obstacle_id=prefix + "_shoulder_elbow",
                                                    points=shoulder_elbow,
                                                    shape="line")
                                walker.update_scene(obstacle_id=prefix + "_elbow_wrist",
                                                    points=elbow_wrist,
                                                    shape="line")
                elif "plm.walker." in binding_name:
                    # extract walker id
                    binding_delimited_array = binding_name.split(".")
                    walker_id = binding_delimited_array[len(binding_delimited_array) - 1]
                    msg_attributes = message_body.keys()
                    if ("id" in msg_attributes) and \
                            ("x_est_pos" in msg_attributes) and \
                            ("y_est_pos" in msg_attributes) and \
                            ("z_est_pos" in msg_attributes) and \
                            ("timestamp" in msg_attributes):
                        for walker in self.walkers_in_ws:
                            if walker.id == walker_id and walker_id == message_body["id"]:
                                logger.debug(f'sub: exchange {exchange_name}: msg {message_body}')
                                walker.update_particles(x=message_body["x_est_pos"],
                                                        y=message_body["y_est_pos"])
                                break
                        else:
                            return False  # robot id in binding name and message body does not match
                    else:
                        return False  # invalid message body format
