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
from pycollisionavoidance.collision.Detection import Detection

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/walkgen.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


# ========================================= WALK PATTERN GENERATOR ===================================================

class Avoidance:

    def __init__(self, eventloop, config_file):
        """
        Initialize Avoidance
        :param eventloop: event loop for amqp pub sub
        :param config_file: configuration file
        """
        try:
            # walker id
            self.walker_id = config_file["id"]

            # initialize the start coordinates
            self.pos = {'x': config_file["start_coordinates"]["x"],
                        'y': config_file["start_coordinates"]["y"],
                        'z': config_file["start_coordinates"]["z"]}

            self.interval = config_file["attribute"]["interval"]

            # time stamp information
            self.time_now = 0
            self.time_past = 0

            # Collision detection
            self.collision = Detection(scene=StaticMap(config_file=config_file["workspace"]),
                                       particle=Particle(particle_id=self.walker_id,
                                                         x=self.pos["x"],
                                                         y=self.pos["y"]),
                                       env_collision_distance=0,
                                       robot_collision_distance=config_file["attribute"]["collision"][
                                           "distance"]["robot"])

            # Publisher
            protocol = config_file["protocol"]
            self.publishers = []
            if protocol["publishers"] is not None:
                for publisher in protocol["publishers"]:
                    if publisher["type"] == "amq":
                        logger.debug('Setting Up AMQP Publisher for Robot')
                        self.publishers.append(
                            PubSubAMQP(
                                eventloop=eventloop,
                                config_file=publisher,
                                binding_suffix=self.walker_id
                            )
                        )
                    else:
                        logger.error("Provide protocol amq config")
                        raise AssertionError("Provide protocol amq config")

            # Subscriber
            self.subscribers = []
            if protocol["subscribers"] is not None:
                for subscriber in protocol["subscribers"]:
                    if subscriber["type"] == "amq":
                        logger.debug('Setting Up AMQP Subcriber for Robot')
                        if subscriber["exchange"] == "control_exchange":
                            self.subscribers.append(
                                PubSubAMQP(
                                    eventloop=eventloop,
                                    config_file=subscriber,
                                    binding_suffix="",
                                    app_callback=self._consume_telemetry_msg
                                )
                            )
                        else:
                            self.subscribers.append(
                                PubSubAMQP(
                                    eventloop=eventloop,
                                    config_file=subscriber,
                                    binding_suffix=self.walker_id,
                                    app_callback=self._consume_telemetry_msg
                                )
                            )
                    else:
                        logger.error("Provide protocol amq config")
                        raise AssertionError("Provide protocol amq config")

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
                if "telemetry.robot" in binding_name:
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

                            # update robot in scene for collision detection
                            self.collision.update_scene(obstacle_id=prefix + "_base_shoulder",
                                                        points=base_shoulder,
                                                        shape="line")
                            self.collision.update_scene(obstacle_id=prefix + "_shoulder_elbow",
                                                        points=shoulder_elbow,
                                                        shape="line")
                            self.collision.update_scene(obstacle_id=prefix + "_elbow_wrist",
                                                        points=elbow_wrist,
                                                        shape="line")
                elif "telemetry.pls" in binding_name:
                    # extract walker id
                    binding_delimited_array = binding_name.split(".")
                    walker_id = binding_delimited_array[len(binding_delimited_array) - 1]
                    msg_attributes = message_body.keys()
                    if ("id" in msg_attributes) and \
                            ("x_est_pos" in msg_attributes) and \
                            ("y_est_pos" in msg_attributes) and \
                            ("z_est_pos" in msg_attributes) and \
                            ("timestamp" in msg_attributes):
                        if walker_id == message_body["id"]:
                            msg_log = {'id': message_body["id"],
                                       "x_ref_pos": message_body['x_ref_pos'],
                                       "y_ref_pos": message_body['y_ref_pos'],
                                       "x_est_pos": message_body['x_est_pos'],
                                       "y_est_pos": message_body['y_est_pos'],
                                       "z_est_pos": message_body['z_est_pos'],
                                       "timestamp": message_body['timestamp']}
                            logger.debug(msg_log)
                            self.collision.update_particles(x=message_body["x_est_pos"], y=message_body["y_est_pos"])
                        else:
                            return False  # robot id in binding name and message body does not match
                    else:
                        return False  # invalid message body format

    async def _update3d(self, tdelta=-1):
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
            ranging, collision_avoidance_msg = self.collision.ranging()

            # collision avoidance
            if len(collision_avoidance_msg) > 0:
                for msg in collision_avoidance_msg:
                    await self.publish(exchange_name="control_exchange",
                                       msg=json.dumps(msg).encode(),
                                       external_binding_suffix=msg["id"])

        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)

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
                    logger.debug(msg)
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

    async def update(self, binding_key=None):
        """
        update walk generator.
        Note This function need to be called in a loop every update cycle
        :param binding_key: binding key name (optional) used when other than default binding key
        :return:
        """
        try:
            if self.interval >= 0:
                await self._update3d(tdelta=self.interval)
            else:
                await self._update3d()

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
