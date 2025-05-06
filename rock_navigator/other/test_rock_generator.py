import asyncio
import websockets
import json
import random
import uuid
import argparse
import logging
import socket
import os
import sys

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Rock data generation logic (copied from rock_navigator.py)
ROCK_TYPES = ['igneous', 'metamorphic', 'sedimentary', '?']
MAX_ROCKS = 4

def generate_rock_data():
    num_rocks = random.randint(1, MAX_ROCKS)
    rocks = []
    for _ in range(num_rocks):
        rock = {
            "id": str(uuid.uuid4())[:8],
            "x": random.uniform(0.0, 1.0),
            "y": random.uniform(0.0, 1.0),
            "radius": random.uniform(20.0, 50.0),
            "type": [random.choice(ROCK_TYPES), random.uniform(0.0, 1.0)]
        }
        rocks.append(rock)
    return rocks

async def send_rock_data(websocket, path=None):
    try:
        logger.info(f"Client connected from path: {path or 'unknown'}")
        while True:
            # Generate random rock data
            rocks = generate_rock_data()
            
            # Send the data
            await websocket.send(json.dumps(rocks))
            logger.info(f"Sent rock data: {json.dumps(rocks, indent=2)}")
            await asyncio.sleep(2)  # Send data every 2 seconds
    except websockets.exceptions.ConnectionClosed as e:
        logger.info(f"Client disconnected: {e}")
    except Exception as e:
        logger.error(f"Unexpected error in send_rock_data: {str(e)}", exc_info=True)
        raise  # Re-raise to ensure the server logs the internal error

async def main(ip, port):
    try:
        # Validate IP address
        if ip not in ["localhost", "127.0.0.1", "0.0.0.0"]:
            try:
                socket.inet_aton(ip)  # Check if IP is valid IPv4
                # Check if IP is assigned to an interface
                interfaces = os.popen("ip addr | grep inet | awk '{print $2}' | cut -d'/' -f1").read().split()
                if ip not in interfaces:
                    logger.error(f"IP {ip} not assigned to any network interface. Available IPs: {interfaces}")
                    logger.error("Use --ip 0.0.0.0 to bind to all interfaces, --ip localhost, or a valid IP from the list above.")
                    sys.exit(1)
            except socket.error:
                logger.error(f"Invalid IP address: {ip}. Use a numeric IPv4 address, localhost, or 0.0.0.0.")
                sys.exit(1)
        server = await websockets.serve(send_rock_data, ip, port)
        logger.info(f"WebSocket server running on ws://{ip}:{port}")
        await server.wait_closed()
    except Exception as e:
        logger.error(f"Failed to start WebSocket server: {str(e)}", exc_info=True)
        raise

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simple WebSocket Server for Rock Data")
    parser.add_argument('--ip', type=str, default="localhost", help="IP address to bind (default: localhost, use 0.0.0.0 for all interfaces)")
    parser.add_argument('--port', type=int, default=8080, help="Port to bind (default: 8080)")
    args = parser.parse_args()

    asyncio.run(main(args.ip, args.port))
