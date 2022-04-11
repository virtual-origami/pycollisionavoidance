import asyncio, socket
import logging
import sys

logging.basicConfig(stream=sys.stdout, level=logging.DEBUG,
                    format='%(levelname)-8s [%(filename)s:%(lineno)d] %(message)s')
logger = logging.getLogger("Robot:Model")


class HealthServer:
    def __init__(self, config, event_loop):
        self.host = ""
        self.port = int(config["port"])
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(8)
        self.server.setblocking(False)
        self.event_loop = event_loop

    @staticmethod
    async def handle_client(client):
        loop = asyncio.get_event_loop()
        response_body = 'collision avoidance is healthy'
        response_headers = {
            'Content-Type': 'text/plain; encoding=utf-8',
            'Content-Length': len(response_body),
            'Connection:': 'close',
        }
        response_headers_raw = ''.join(f'{k}: {v}\r\n' for k, v in response_headers.items())
        response_protocol = 'HTTP/1.1'
        response_status = '200'
        response_status_text = 'OK'
        response = f'{response_protocol} {response_status} {response_status_text}\r\n{response_headers_raw}\r\n{response_body}'.encode('utf-8')
        await loop.sock_sendall(client, response)
        logger.debug("sent health status")
        client.close()

    async def server_loop(self):
        try:
            while True:
                self.event_loop = asyncio.get_event_loop()
                client, _ = await self.event_loop.sock_accept(self.server)
                self.event_loop.create_task(self.handle_client(client))
        except Exception as e:
            return


