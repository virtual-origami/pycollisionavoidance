import argparse
import asyncio
import logging
import os
import sys
import signal
import functools
import yaml
from collision.Avoidance import Avoidance

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
    parser.add_argument('--config', '-c', required=True, help='YAML Configuration File for Collision Avoidance with path')
    return parser.parse_args()


def signal_handler(name):
    global is_sighup_received
    is_sighup_received = True


async def app(eventloop, config):
    """Main application for Personnel Generator"""
    walkers_in_ws = []
    global is_sighup_received

    while True:
        # Read configuration
        try:
            walk_config = read_config(yaml_file=config, rootkey='walk_generator')
        except Exception as e:
            logger.error(f'Error while reading configuration: {e}')
            break

        logger.debug("Collision Avoidance Version: %s", walk_config['version'])

        # check if amq or mqtt key description present in configuration
        if ("amq" not in walk_config) and ("mqtt" not in walk_config):
            logger.critical("Please provide either 'amq' or 'mqtt' configuration")
            sys.exit(-1)

        # Personnel instantiation
        for each_walker in walk_config["personnel"]:

            # check for protocol key
            if "protocol" not in each_walker:
                logger.critical("no 'protocol' key found.")
                sys.exit(-1)

            # create walker
            walker = Avoidance(eventloop=eventloop, config_file=each_walker)
            await walker.connect()
            walkers_in_ws.append(walker)

        # continuously monitor signal handle and update walker
        while not is_sighup_received:
            for each_walker in walkers_in_ws:
                await each_walker.update(binding_key="telemetry.walker")

        # If SIGHUP Occurs, Delete the instances
        for entry in walkers_in_ws:
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


def main():
    """Initialization"""
    args = parse_arguments()
    if not os.path.isfile(args.config):
        logger.error("configuration file not readable. Check path to configuration file")
        sys.exit(-1)

    event_loop = asyncio.get_event_loop()
    event_loop.add_signal_handler(signal.SIGHUP, functools.partial(signal_handler, name='SIGHUP'))
    event_loop.run_until_complete(app(event_loop, args.config))


if __name__ == "__main__":
    main()
