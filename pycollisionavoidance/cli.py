import argparse
import asyncio
import logging
import os
import sys
import signal
import functools
import yaml
from pycollisionavoidance.collision.Avoidance import CollisionAvoidance
from pycollisionavoidance.health import HealthServer

logging.basicConfig(level=logging.WARNING, format='%(levelname)-8s [%(filename)s:%(lineno)d] %(message)s')

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('/tmp/walkgen.log')
handler.setLevel(logging.ERROR)
formatter = logging.Formatter('%(levelname)-8s-[%(filename)s:%(lineno)d]-%(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

is_sighup_received = False


def parse_arguments():
    """Arguments to run the script"""
    parser = argparse.ArgumentParser(description='Collision Avoidance')
    parser.add_argument('--config', '-c', required=True,
                        help='YAML Configuration File for Collision Avoidance with path')
    return parser.parse_args()


def signal_handler(name):
    global is_sighup_received
    is_sighup_received = True


async def app(eventloop, config):
    """Main application for Personnel Generator"""
    workspace_collection = []
    global is_sighup_received

    while True:
        # Read configuration
        try:
            walk_config = read_config(yaml_file=config, rootkey='collision_avoidance')
        except Exception as e:
            logger.error(f'Error while reading configuration: {e}')
            break

        logger.debug("Collision Avoidance Version: %s", walk_config['version'])

        # health server
        health_server = HealthServer(config=walk_config["health_server"], event_loop=eventloop)
        eventloop.create_task(health_server.server_loop())

        try:
            update_interval = walk_config["update_interval"]
            assert type(update_interval) is int or type(update_interval) is float
        except Exception as e:
            logger.error(f'Update interval need to be a number. Check config file {e}')
            break

        # check if amq or mqtt key description present in configuration
        if ("amq" not in walk_config) and ("mqtt" not in walk_config):
            logger.critical("Please provide either 'amq' or 'mqtt' configuration")
            sys.exit(-1)

        # Personnel instantiation
        for workspace in walk_config["workareas"]:
            ws = CollisionAvoidance(eventloop=eventloop, config_file=workspace)
            await ws.connect()
            workspace_collection.append(ws)

        # continuously monitor signal handle and update walker
        while not is_sighup_received:
            for ws in workspace_collection:
                await ws.update()

            await asyncio.sleep(delay=update_interval)
        # If SIGHUP Occurs, Delete the instances
        for entry in workspace_collection:
            del entry

        # reset sighup handler flag
        is_sighup_received = False


def read_config(yaml_file, rootkey):
    """Parse the given Configuration File"""
    if os.path.exists(yaml_file):
        with open(yaml_file, 'r') as config_file:
            yaml_as_dict = yaml.load(config_file, Loader=yaml.FullLoader)
        return yaml_as_dict[rootkey]
    else:
        raise FileNotFoundError
        logger.error('YAML Configuration File not Found.')


def app_main():
    """Initialization"""
    args = parse_arguments()
    if not os.path.isfile(args.config):
        logger.error("configuration file not readable. Check path to configuration file")
        sys.exit(-1)

    event_loop = asyncio.get_event_loop()
    event_loop.add_signal_handler(signal.SIGHUP, functools.partial(signal_handler, name='SIGHUP'))
    event_loop.run_until_complete(app(eventloop=event_loop, config=args.config))
