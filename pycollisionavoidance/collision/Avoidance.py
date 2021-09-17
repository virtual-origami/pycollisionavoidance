# Python code for 2D random walk.
import json
import sys
import random
import time
import math
import queue
import logging
import asyncio
import traceback

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
    def __init__(self, eventloop, config_file):
        """
        Initialize Collision Avoidance
        :param eventloop: event loop for amqp pub sub
        :param config_file: configuration file
        """
        try:
            self.workspace_attributes = config_file["workspace"]
            self.publishers = []
            self.subscribers = []
            self.robot_msg_queue = queue.SimpleQueue()
            self.personnel_msg_queue = queue.SimpleQueue()

            protocol = config_file["protocol"]
            # check for protocol key
            if "protocol" not in config_file:
                logger.critical("no 'protocol' key found.")
                sys.exit(-1)

            # Personnel instantiation
            pos = {'x': None, 'y': None, 'z': None}
            env_collision_distance = config_file["personnel"]["attribute"]["collision"]["distance"]["environment"]
            robot_collision_distance = config_file["personnel"]["attribute"]["collision"]["distance"]["robot"]

            # Collision detection
            self.walker_in_ws = ParticleCollisionDetection(scene=StaticMap(config_file=self.workspace_attributes),
                                                           particle=Particle(x=pos["x"], y=pos["y"]),
                                                           env_collision_distance=env_collision_distance,
                                                           robot_collision_distance=robot_collision_distance)

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
            await self.robot_update()
            await self.personnel_update()
        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)

    async def personnel_update(self):
        """
        update walk generator.
        Note This function need to be called in a loop every update cycle
        :return:
        """
        try:
            if not self.personnel_msg_queue.empty():
                new_message = self.personnel_msg_queue.get_nowait()
                self.walker_in_ws.update_particles(x=new_message["x_est_pos"],
                                                   y=new_message["y_est_pos"])
                await self.walker_in_ws.update()

                # collision avoidance
                if len(self.walker_in_ws.robot_collision) > 0:
                    collision_robot_list = list({v['id']: v for v in self.walker_in_ws.robot_collision}.values())
                    for msg in collision_robot_list:
                        await self.publish(exchange_name="control_robot",
                                           msg=json.dumps(msg).encode())
        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)

    async def robot_update(self):
        try:
            if not self.robot_msg_queue.empty():
                new_message = self.robot_msg_queue.get_nowait()

                # extract information from message body
                base_shoulder = [new_message["base"], new_message["shoulder"]]
                shoulder_elbow = [new_message["shoulder"], new_message["elbow"]]
                elbow_wrist = [new_message["elbow"], new_message["wrist"]]
                prefix = "robot_" + new_message["id"]

                # update robot in scene for collision detection
                # logger.debug(f'sub: exchange {exchange_name}: msg {message_body}')

                # center_x = new_message["base"][0]
                # center_y = new_message["base"][1]
                # base_points = [[center_x - 2, center_y - 2],
                #           [center_x + 2, center_y - 2],
                #           [center_x + 2, center_y + 2],
                #           [center_x + 2, center_y - 2]]
                # self.walker_in_ws.update_scene(obstacle_id=prefix + "_base",
                #                                points=base_points,
                #                                shape="polygon")

                self.walker_in_ws.update_scene(obstacle_id=prefix + "_base_shoulder",
                                               points=base_shoulder,
                                               shape="line")
                self.walker_in_ws.update_scene(obstacle_id=prefix + "_shoulder_elbow",
                                               points=shoulder_elbow,
                                               shape="line")
                self.walker_in_ws.update_scene(obstacle_id=prefix + "_elbow_wrist",
                                               points=elbow_wrist,
                                               shape="line")
        except Exception as e:
            logger.critical("unhandled exception", e)
            sys.exit(-1)

    def get_states(self):
        return {"x_ref_pos": self.pos['x'], "y_ref_pos ": self.pos['y'], "z_ref_pos": self.pos['z']}

    async def publish(self, exchange_name, msg):
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
                    await publisher.publish(message_content=msg)
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

    async def robot_msg_handler(self, exchange_name, binding_name, message_body):

        msg_attributes = message_body.keys()

        # check for must fields in the message attributes
        if ("id" in msg_attributes) and ("base" in msg_attributes) \
                and ("shoulder" in msg_attributes) and ("elbow" in msg_attributes):
            self.robot_msg_queue.put_nowait(item=message_body)

    async def personnel_msg_handler(self, exchange_name, binding_name, message_body):

        msg_attributes = message_body.keys()
        if ("id" in msg_attributes) and \
                ("x_est_pos" in msg_attributes) and \
                ("y_est_pos" in msg_attributes) and \
                ("z_est_pos" in msg_attributes) and \
                ("timestamp" in msg_attributes):

            # logger.debug(f'sub: exchange {exchange_name}: msg {message_body}')
            self.personnel_msg_queue.put_nowait(item=message_body)

    async def _consume_telemetry_msg(self, **kwargs):
        """
        consume telemetry messages
        :param kwargs: must contain following information
                       1.   exchange_name
                       2.   binding_name
                       3.   message_body
        :return: none
        """
        try:
            # extract message attributes from message
            exchange_name = kwargs["exchange_name"]
            binding_name = kwargs["binding_name"]
            message_body = json.loads(kwargs["message_body"])

            # check for matching subscriber with exchange and binding name in all subscribers
            for subscriber in self.subscribers:
                # if subscriber.exchange_name == exchange_name:
                cb_str = subscriber.get_callback_handler_name()
                if cb_str is not None:
                    try:
                        cb = getattr(self, cb_str)
                    except:
                        logging.critical(f'No Matching handler found for {cb_str}')
                        continue
                    if cb is not None:
                        await cb(exchange_name=exchange_name, binding_name=binding_name, message_body=message_body)
        except AssertionError as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()
        except Exception as e:
            logging.critical(e)
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logging.critical(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
            sys.exit()