import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import json
import numpy as np
import random
import uuid
from turtlesim.srv import TeleportAbsolute, SetPen
import math
import time
import asyncio
import websockets
import threading
import argparse

class RockNavigator(Node):
    UPDATE_INTERVAL = 2.0
    ROCK_TYPES = ['igneous', 'metamorphic', 'sedimentary', '?']
    MAX_ROCKS = 4
    TRIANGLE_SIDE = 8.0  # Side length of the equilateral triangle
    TRIANGLE_CENTER_X = 5.5  # Center of TurtleSim (0-11)
    TRIANGLE_CENTER_Y = 5.5
    CONFIDENCE_THRESHOLD = 0.1  # Lowered to process more rocks
    MOVE_SPEED = 1.0  # Increased for faster movement
    ANGULAR_SPEED = 10.0  # Proportional gain for turning
    MIN_ANGULAR_VELOCITY = 5e-3  # Minimum angular velocity
    ANGLE_DEADBAND = 1e-4  # Deadband to prevent oscillation
    DISTANCE_THRESHOLD = 0.05  # Tighter threshold to stop precisely
    ANGLE_THRESHOLD = 0.01  # Finer alignment threshold
    TURNING_TIMEOUT = 4.0  # Timeout for turning
    MAX_TURNING_ITERATIONS = 20  # Iteration limit
    TARGET_CHANGE_THRESHOLD = 0.5  # Distance threshold for new targets
    CONTROL_PERIOD = 0.05  # Control loop period in seconds (20 Hz)
    EPSILON = 1e-2  # Tolerance for boundary checks
    SERVICE_TIMEOUT = 30.0  # Increased timeout for waiting for TurtleSim services

    def __init__(self, use_mock_data=False, ws_ip="localhost", ws_port=8080):
        super().__init__('rock_navigator')
        # Data source configuration
        self.USE_MOCK_DATA = use_mock_data
        self.WS_IP = f"ws://{ws_ip}:{ws_port}"  # Dynamic WebSocket server IP/port
        self.WS_RECONNECT_INTERVAL = 5.0  # Initial reconnect interval if WebSocket fails
        self.MAX_RECONNECT_ATTEMPTS = 5  # Maximum number of reconnect attempts before increasing delay

        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.context_initialized = False

        # Wait for services with increased timeout
        try:
            self.wait_for_service_with_timeout(self.teleport_client, "teleport", timeout=self.SERVICE_TIMEOUT)
            self.wait_for_service_with_timeout(self.set_pen_client, "set_pen", timeout=self.SERVICE_TIMEOUT)
        except RuntimeError as e:
            self.get_logger().error(f"Failed to initialize services: {e}")
            raise

        # Initialize current position
        self.current_pose = Pose()
        self.current_pose.x = 5.5  # Initial position
        self.current_pose.y = 5.5
        
        # Define triangle vertices for an equilateral triangle
        self.IGNEOUS_POS = np.array([1.5, 3.19059892])
        self.METAMORPHIC_POS = np.array([9.5, 3.19059892])
        # Correct sedimentary position: height = sqrt(3)/2 * 8 ≈ 6.9282 above base
        self.SEDIMENTARY_POS = np.array([5.5, 3.19059892 + (math.sqrt(3) / 2 * 8)])

        # Movement state: "turning" or "moving"
        self.movement_state = "turning"
        self.target_pos = None
        self.turning_start_time = None
        self.turning_iterations = 0  # Counter for turning iterations

        # WebSocket data
        self.websocket_data = None
        self.websocket_connected = False
        if not self.USE_MOCK_DATA:
            self.get_logger().info(f"Connecting to WebSocket server at {self.WS_IP}")
            self.ws_thread = threading.Thread(target=self.run_websocket_client, daemon=True)
            self.ws_thread.start()

        # Draw the triangle at startup
        self.draw_triangle()
        
        # Start periodic rock data generation
        self.create_timer(self.UPDATE_INTERVAL, self.generate_and_process_rock_data)
        
        # Start control loop at 20 Hz
        self.control_timer = self.create_timer(self.CONTROL_PERIOD, self.control_loop)
        
        self.get_logger().info("Rock Navigator started")

    def pose_callback(self, msg):
        """Callback to update the turtle's current pose."""
        self.current_pose = msg
        self.get_logger().debug(f"Updated pose: x={msg.x}, y={msg.y}, theta={msg.theta}")

    def wait_for_service_with_timeout(self, client, service_name, timeout):
        """Wait for a service to become available with a timeout."""
        start_time = time.time()
        self.get_logger().info(f"Waiting for service {service_name}...")
        while not client.service_is_ready():
            if time.time() - start_time > timeout:
                raise RuntimeError(f"Timed out waiting for service {service_name}")
            self.get_logger().warn(f"Service {service_name} not available, retrying...")
            time.sleep(1.0)
        self.get_logger().info(f"Service {service_name} is available")

    async def websocket_client(self):
        attempt = 0
        reconnect_delay = self.WS_RECONNECT_INTERVAL
        while rclpy.ok():
            try:
                async with websockets.connect(self.WS_IP) as websocket:
                    self.get_logger().info("WebSocket connection established")
                    self.websocket_connected = True
                    attempt = 0  # Reset attempts on successful connection
                    reconnect_delay = self.WS_RECONNECT_INTERVAL  # Reset delay
                    while rclpy.ok():
                        message = await websocket.recv()
                        try:
                            rocks = json.loads(message)
                            self.websocket_data = rocks
                            self.get_logger().info(f"Received WebSocket data: {json.dumps(rocks, indent=2)}")
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f"Failed to parse WebSocket JSON: {e}")
                        except websockets.exceptions.ConnectionClosed as e:
                            self.get_logger().info(f"WebSocket connection closed: {e}")
                            break
            except Exception as e:
                self.get_logger().error(f"Websocket connection failed: {str(e)}")
                self.websocket_connected = False
                attempt += 1
                if attempt >= self.MAX_RECONNECT_ATTEMPTS:
                    reconnect_delay *= 2  # Exponential backoff
                    attempt = 0
                    self.get_logger().warn(f"Increasing reconnect delay to {reconnect_delay} seconds after {self.MAX_RECONNECT_ATTEMPTS} failed attempts")
                self.get_logger().info(f"Reconnecting in {reconnect_delay} seconds...")
                await asyncio.sleep(reconnect_delay)

    def run_websocket_client(self):
        asyncio.run(self.websocket_client())

    def draw_triangle(self):
        # Set pen to blue for triangle (RGB: 0, 0, 255)
        self.set_pen(0, 0, 255, 2, 0)
        time.sleep(0.1)  # Ensure pen color applies
        
        # Draw triangle: igneous -> metamorphic -> sedimentary -> igneous
        vertices = [self.IGNEOUS_POS, self.METAMORPHIC_POS, self.SEDIMENTARY_POS, self.IGNEOUS_POS]
        for vertex in vertices:
            self.get_logger().info(f"Teleporting to vertex: [{vertex[0]}, {vertex[1]}]")
            self.teleport_to(vertex[0], vertex[1], 0.0)
            time.sleep(0.1)  # Small delay to ensure teleportation
        
        # Set pen to red for path (RGB: 255, 0, 0)
        self.set_pen(255, 0, 0, 1, 0)
        time.sleep(0.1)  # Ensure pen color applies
        # Move back to center to start
        self.get_logger().info(f"Teleporting to center: [{self.TRIANGLE_CENTER_X}, {self.TRIANGLE_CENTER_Y}]")
        self.teleport_to(self.TRIANGLE_CENTER_X, self.TRIANGLE_CENTER_Y, 0.0)

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = self.set_pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def teleport_to(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def generate_rock_data(self):
        num_rocks = random.randint(1, self.MAX_ROCKS)
        rocks = []
        for _ in range(num_rocks):
            rock = {
                "id": str(uuid.uuid4())[:8],
                "x": random.uniform(0.0, 1.0),
                "y": random.uniform(0.0, 1.0),
                "radius": random.uniform(20.0, 50.0),
                "type": [random.choice(self.ROCK_TYPES), random.uniform(0.0, 1.0)]
            }
            rocks.append(rock)
        return rocks

    def generate_and_process_rock_data(self):
        if self.USE_MOCK_DATA:
            rocks = self.generate_rock_data()
        else:
            rocks = self.websocket_data
            if rocks is None:
                self.get_logger().warn("No WebSocket data received yet, skipping update")
                return

        self.get_logger().info(f"Generated rock data: {json.dumps(rocks, indent=2)}")
        new_target_pos = self.compute_target_position(rocks)
        if new_target_pos is not None:
            if self.target_pos is None or self.is_target_reached() or self.is_target_significantly_different(new_target_pos):
                self.target_pos = new_target_pos
                self.movement_state = "turning"  # Reset to turning state for new target
                self.turning_start_time = time.time()
                self.turning_iterations = 0
                self.get_logger().info(f"New target set: {new_target_pos}")

    def control_loop(self):
        if self.target_pos is not None:
            self.move_to_target(self.target_pos)

    def is_target_reached(self):
        if self.target_pos is None:
            return True
        dx = self.target_pos[0] - self.current_pose.x
        dy = self.target_pos[1] - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < self.DISTANCE_THRESHOLD:
            self.get_logger().info(f"Reached target: {self.target_pos}")
        return distance < self.DISTANCE_THRESHOLD

    def is_target_significantly_different(self, new_target):
        if self.target_pos is None:
            return True
        dx = new_target[0] - self.target_pos[0]
        dy = new_target[1] - self.target_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        return distance > self.TARGET_CHANGE_THRESHOLD

    def compute_target_position(self, rocks):
        counts = {'igneous': 0, 'metamorphic': 0, 'sedimentary': 0}
        for rock in rocks:
            rock_type, confidence = rock.get('type', ['?', 0.0])
            if confidence >= self.CONFIDENCE_THRESHOLD and rock_type in counts:
                counts[rock_type] += 1

        total = sum(counts.values())
        if total == 0:
            self.get_logger().warn("No valid rocks detected, staying in place")
            return None

        self.get_logger().info(f"Recognized rocks: {counts['igneous']} igneous, {counts['metamorphic']} metamorphic, {counts['sedimentary']} sedimentary")

        weights = {k: v / total for k, v in counts.items()}
        self.get_logger().info(f"Rock weights: {weights}")

        target = (
            weights['igneous'] * self.IGNEOUS_POS +
            weights['metamorphic'] * self.METAMORPHIC_POS +
            weights['sedimentary'] * self.SEDIMENTARY_POS
        )

        # Determine leaning direction
        distances = {
            'igneous': np.linalg.norm(target - self.IGNEOUS_POS),
            'metamorphic': np.linalg.norm(target - self.METAMORPHIC_POS),
            'sedimentary': np.linalg.norm(target - self.SEDIMENTARY_POS)
        }
        closest_vertex = min(distances, key=distances.get)
        self.get_logger().info(f"Leaning towards {closest_vertex} vertex")

        return target

    def is_inside_triangle(self, x, y):
        """Check if a point is inside or on the triangle using barycentric coordinates with tolerance."""
        def sign(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

        pt = (x, y)
        v1 = (self.IGNEOUS_POS[0], self.IGNEOUS_POS[1])
        v2 = (self.METAMORPHIC_POS[0], self.METAMORPHIC_POS[1])
        v3 = (self.SEDIMENTARY_POS[0], self.SEDIMENTARY_POS[1])

        # Include boundary with tolerance for floating-point errors
        b1 = sign(pt, v1, v2) <= self.EPSILON
        b2 = sign(pt, v2, v3) <= self.EPSILON
        b3 = sign(pt, v3, v1) <= self.EPSILON

        return (b1 == b2) and (b2 == b3)

    def move_to_target(self, target_pos):
        # Calculate distance to target
        dx = target_pos[0] - self.current_pose.x
        dy = target_pos[1] - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        # Stop if close to target
        if distance < self.DISTANCE_THRESHOLD:
            self.get_logger().info(f"Reached target: {self.target_pos}")
            twist = Twist()  # Stop moving
            self.cmd_vel_publisher.publish(twist)
            return

        # Calculate desired angle
        desired_angle = math.atan2(dy, dx)
        current_angle = self.current_pose.theta
        angle_diff = desired_angle - current_angle

        # Normalize angle difference to [-pi, pi]
        angle_diff = ((angle_diff + math.pi) % (2 * math.pi)) - math.pi

        # Create twist message
        twist = Twist()

        # State machine: turn first, then move
        if self.movement_state == "turning" or abs(angle_diff) > self.ANGLE_THRESHOLD:
            if self.movement_state != "turning":
                self.movement_state = "turning"
                self.turning_start_time = time.time()
                self.turning_iterations = 0
            self.turning_iterations += 1
            # Check for timeout or iteration limit to prevent getting stuck
            if self.turning_start_time is not None:
                elapsed_time = time.time() - self.turning_start_time
                if elapsed_time > self.TURNING_TIMEOUT or self.turning_iterations > self.MAX_TURNING_ITERATIONS:
                    self.get_logger().warn("Turning timeout or iteration limit reached, forcing transition to moving state")
                    self.movement_state = "moving"
            
            # Proportional control with minimum angular velocity and deadband
            if abs(angle_diff) < self.ANGLE_DEADBAND:
                self.movement_state = "moving"
                self.get_logger().info("Angle within deadband, switching to moving state")
            else:
                angular_velocity = self.ANGULAR_SPEED * angle_diff
                if abs(angular_velocity) < self.MIN_ANGULAR_VELOCITY:
                    angular_velocity = self.MIN_ANGULAR_VELOCITY * (1 if angle_diff > 0 else -1)
                twist.angular.z = angular_velocity
                self.get_logger().info(f"Turning towards target, desired angle: {desired_angle}, current angle: {current_angle}, angle difference: {angle_diff}, iterations: {self.turning_iterations}")
        else:
            self.movement_state = "moving"
            # Adjust speed based on distance to prevent overshoot
            speed = min(self.MOVE_SPEED, distance / (32 * self.CONTROL_PERIOD))
            twist.linear.x = speed

            # Predict next position based on control period
            prediction_dt = self.CONTROL_PERIOD
            next_x = self.current_pose.x + twist.linear.x * math.cos(self.current_pose.theta) * prediction_dt
            next_y = self.current_pose.y + twist.linear.x * math.sin(self.current_pose.theta) * prediction_dt
            
            # Stop if next position is outside the triangle
            if not self.is_inside_triangle(next_x, next_y):
                self.get_logger().warn("Attempting to move outside triangle, stopping")
                twist.linear.x = 0.0

            self.get_logger().info("Moving linearly")

        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Moving to target: {self.target_pos}, current: [{self.current_pose.x}, {self.current_pose.y}], angle: {self.current_pose.theta}")

def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Rock Navigator for TurtleSim")
    parser.add_argument('--mock', action='store_true', help="Run with mock data instead of WebSocket")
    parser.add_argument('--ws-ip', type=str, default="localhost", help="WebSocket server IP (default: localhost)")
    parser.add_argument('--ws-port', type=int, default=8080, help="WebSocket server port (default: 8080)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    node = None
    try:
        node = RockNavigator(
            use_mock_data=parsed_args.mock,
            ws_ip=parsed_args.ws_ip,
            ws_port=parsed_args.ws_port
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    except Exception as e:
        if str(e):  # Suppress empty error messages during shutdown
            print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
