import asyncio
import websockets
import json
import argparse
import logging
import socket
import os
import sys

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

async def handle_client(websocket, path=None):
    logger.info(f"Client connected from path: {path or 'unknown'}")
    try:
        while True:
            message = await websocket.recv()
            try:
                data = json.loads(message)
                logger.info(f"Received JSON data: {json.dumps(data, indent=2)}")
                # Add custom processing of received JSON data here if needed
            except json.JSONDecodeError:
                logger.error(f"Received invalid JSON: {message}")
    except websockets.exceptions.ConnectionClosed as e:
        logger.info(f"Client disconnected: {e}")
    except Exception as e:
        logger.error(f"Error in handle_client: {str(e)}", exc_info=True)

async def main(ip, port):
    try:
        # Validate IP address
        if ip not in ["localhost", "127.0.0.1", "0.0.0.0"]:
            try:
                socket.inet_aton(ip)  # Check if IP is valid IPv4
                interfaces = os.popen("ip addr | grep inet | awk '{print $2}' | cut -d'/' -f1").read().split()
                if ip not in interfaces:
                    logger.error(f"IP {ip} not assigned to any network interface. Available IPs: {interfaces}")
                    logger.error("Use --ip 0.0.0.0 to bind to all interfaces, --ip localhost, or a valid IP from the list above.")
                    sys.exit(1)
            except socket.error:
                logger.error(f"Invalid IP address: {ip}. Use a numeric IPv4 address, localhost, or 0.0.0.0.")
                sys.exit(1)
        server = await websockets.serve(handle_client, ip, port)
        logger.info(f"WebSocket server running on ws://{ip}:{port}")
        await server.wait_closed()
    except Exception as e:
        logger.error(f"Failed to start WebSocket server: {str(e)}", exc_info=True)
        raise

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebSocket Server for Receiving JSON Data")
    parser.add_argument('--ip', type=str, default="localhost", help="IP address to bind (default: localhost, use 0.0.0.0 for all interfaces)")
    parser.add_argument('--port', type=int, default=8080, help="Port to bind (default: 8080)")
    args = parser.parse_args()

    asyncio.run(main(args.ip, args.port))
