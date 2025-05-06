#!/bin/bash
set -e

# Rock Navigator Setup Script
# Orchestrates TurtleSim navigation with mock data, local WebSocket, or real-time IP/port modes.
# Optimizes Docker image usage by rebuilding only when necessary.

# Ensure script runs from rock_navigator/ directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$SCRIPT_DIR" != "$(pwd)" ]; then
    echo "Error: Please run this script from the rock_navigator/ directory."
    echo "Run: cd rock_navigator && ./setup.sh"
    exit 1
fi

# Default IP and port for WebSocket
DEFAULT_IP="localhost"
DEFAULT_PORT=8080

# Initialize flags and variables
MOCK=false
TEST=false
IP=""
PORT=""
IMAGE_NAME="rock_navigator:latest"

# Parse command-line arguments
while [ "$1" != "" ]; do
    case $1 in
        --mock )    MOCK=true
                    ;;
        --test )    TEST=true
                    ;;
        --ip )      shift
                    IP=$1
                    ;;
        --port )    shift
                    PORT=$1
                    ;;
        --help )    echo "Usage: ./setup.sh [--mock] [--test] [--ip IP] [--port PORT]"
                    echo "  --mock: Run TurtleSim with mock rock data in Docker"
                    echo "  --test: Run TurtleSim with WebSocket data from a local server"
                    echo "  --ip IP: Specify WebSocket IP (default: localhost, use 0.0.0.0 for all interfaces)"
                    echo "  --port PORT: Specify WebSocket port (default: 8080)"
                    exit 0
                    ;;
        * )         echo "Invalid option: $1"
                    echo "Use --help for usage information."
                    exit 1
    esac
    shift
done

# Check mutual exclusivity of --mock and --test
if [ "$MOCK" = true ] && [ "$TEST" = true ]; then
    echo "Error: Cannot specify both --mock and --test."
    exit 1
fi

# Set defaults if not provided
IP=${IP:-$DEFAULT_IP}
PORT=${PORT:-$DEFAULT_PORT}

# Check prerequisites
echo "Checking prerequisites..."

# Verify ROS2 Humble
if ! command -v ros2 &> /dev/null; then
    echo "Sourcing ROS2 environment..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "Error: ROS2 Humble setup.bash not found at /opt/ros/humble/setup.bash."
        exit 1
    fi
    if ! command -v ros2 &> /dev/null; then
        echo "Error: ROS2 not found after sourcing."
        exit 1
    fi
fi

# Verify Python3
if ! command -v python3 &> /dev/null; then
    echo "Error: Python3 not installed. Install with: sudo apt install python3 python3-pip python3-venv"
    exit 1
fi

# Verify Docker
if ! command -v docker &> /dev/null; then
    echo "Error: Docker not installed. Install with: sudo apt install docker.io"
    exit 1
fi

# Verify X11 display
if [ -z "$DISPLAY" ]; then
    echo "Error: DISPLAY environment variable not set. Ensure a graphical environment is available."
    exit 1
fi

# Verify xhost
if ! command -v xhost &> /dev/null; then
    echo "Error: xhost not found. Install with: sudo apt install x11-xserver-utils"
    exit 1
fi

echo "Granting X11 access to Docker..."
xhost +local:docker || {
    echo "Error: Failed to set X11 permissions."
    exit 1
}

# Check GPU device (optional)
if [ ! -d "/dev/dri" ]; then
    echo "Warning: GPU device (/dev/dri) not found. TurtleSim may fail to render."
else
    echo "GPU device found: /dev/dri"
fi

# Install fuser
if ! command -v fuser &> /dev/null; then
    echo "Installing psmisc for port conflict resolution..."
    sudo apt update && sudo apt install -y psmisc || {
        echo "Error: Failed to install psmisc."
        exit 1
    }
fi

# Set up virtual environment
VENV_DIR="$SCRIPT_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "Setting up virtual environment in $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"

# Install Python dependencies
echo "Installing Python dependencies..."
pip install websockets numpy || {
    echo "Error: Failed to install dependencies."
    deactivate
    exit 1
}

PYTHON_EXEC="$VENV_DIR/bin/python3"

# Function to clean up processes
cleanup_processes() {
    echo "Cleaning up processes..."
    pkill -f "turtlesim_node" 2>/dev/null || true
    pkill -f "rock_navigator.py" 2>/dev/null || true
    pkill -f "test_rock_server.py" 2>/dev/null || true
    docker stop $(docker ps -q --filter ancestor="$IMAGE_NAME") 2>/dev/null || true
}

# Trap interrupts
trap cleanup_processes EXIT INT TERM

# Check if Docker image needs rebuilding
build_docker_image() {
    echo "Checking if Docker image needs rebuilding..."
    # Calculate hash of Dockerfile and scripts
    HASH_FILE="$SCRIPT_DIR/.docker_hash"
    CURRENT_HASH=$(cat Dockerfile scripts/rock_navigator.py scripts/test_rock_server.py 2>/dev/null | sha256sum | cut -d' ' -f1)
    if [ -f "$HASH_FILE" ]; then
        PREV_HASH=$(cat "$HASH_FILE")
    else
        PREV_HASH=""
    fi

    # Check if image exists
    if docker inspect "$IMAGE_NAME" &>/dev/null && [ "$CURRENT_HASH" = "$PREV_HASH" ]; then
        echo "Docker image $IMAGE_NAME is up to date."
    else
        echo "Building Docker image $IMAGE_NAME..."
        docker build -t "$IMAGE_NAME" -f Dockerfile . || {
            echo "Build failed. Running diagnostics..."
            docker run -it --rm -v "$(pwd):/mnt" osrf/ros:humble-desktop bash -c "
                echo 'Listing workspace directory...' &&
                ls -l /mnt/scripts/ &&
                echo 'Checking rock_navigator.py...' &&
                [ -f /mnt/scripts/rock_navigator.py ] && cat /mnt/scripts/rock_navigator.py || echo 'File not found!' &&
                echo 'Checking ROS2 environment...' &&
                source /opt/ros/humble/setup.bash &&
                ros2 pkg list | grep turtlesim || echo 'TurtleSim not found!' &&
                echo 'Checking Python dependencies...' &&
                pip3 list | grep -E 'numpy|websocket-client|websockets' || echo 'Dependencies missing!'
            "
            cleanup_processes
            deactivate
            exit 1
        }
        echo "$CURRENT_HASH" > "$HASH_FILE"
        echo "Docker image built successfully."
    fi
}

# Validate IP address
if [ "$MOCK" != true ] && [ "$TEST" != true ] && [ "$IP" != "localhost" ] && [ "$IP" != "127.0.0.1" ] && [ "$IP" != "0.0.0.0" ]; then
    echo "Validating IP address $IP..."
    ip addr | grep -w inet | grep -w "$IP" >/dev/null || {
        echo "Error: IP $IP is not assigned to any network interface."
        echo "Available IPs:"
        ip addr | grep -w inet | awk '{print $2}' | cut -d'/' -f1
        echo "Use --ip 0.0.0.0 to bind to all interfaces, --ip localhost, or a valid IP from the list above."
        deactivate

        exit 1
    }
fi

# Check and free port
echo "Checking if port $PORT is in use..."
if lsof -i :$PORT >/dev/null 2>&1; then
    echo "Warning: Port $PORT is in use. Attempting to free port..."
    fuser -k $PORT/tcp || {
        echo "Error: Failed to free port $PORT. Free it manually with: kill -9 \$(lsof -t -i :$PORT)"
        deactivate
        exit 1
    }
fi

if [ "$TEST" = true ]; then
    # Test mode: Run TurtleSim with local WebSocket server
    echo "Starting TurtleSim for test..."
    ros2 run turtlesim turtlesim_node &
    TURTLESIM_PID=$!
    sleep 5
    if ! ps -p $TURTLESIM_PID > /dev/null; then
        echo "Error: TurtleSim failed to start. Checking display..."
        echo "DISPLAY=$DISPLAY"
        glxinfo | grep "OpenGL" || echo "OpenGL not found!"
        cleanup_processes
        deactivate
        exit 1
    fi
    echo "TurtleSim PID: $TURTLESIM_PID"

    # Verify ROS2 services
    echo "Checking ROS2 services..."
    ros2 service list | grep -E "teleport_absolute|set_pen" || {
        echo "Warning: TurtleSim services not found. Retrying after 5 seconds..."
        sleep 5
        ros2 service list | grep -E "teleport_absolute|set_pen" || {
            echo "Error: TurtleSim services not found. Ensure TurtleSim is running."
            cleanup_processes
            deactivate
            exit 1
        }
    }
    echo "TurtleSim services found."

    # Start WebSocket server
    echo "Starting WebSocket test server on $IP:$PORT..."
    TEST_SERVER_PATH=$(realpath scripts/test_rock_server.py)
    "$PYTHON_EXEC" "$TEST_SERVER_PATH" --ip "$IP" --port "$PORT" &
    TEST_SERVER_PID=$!
    sleep 5
    if ! ps -p $TEST_SERVER_PID > /dev/null; then
        echo "Error: WebSocket test server failed to start."
        cleanup_processes
        deactivate
        exit 1
    fi
    echo "Test Server PID: $TEST_SERVER_PID"

    # Run navigation node
    echo "Running rock_navigator.py to connect to the test server..."
    "$PYTHON_EXEC" scripts/rock_navigator.py --ws-ip "$IP" --ws-port "$PORT" &
    NAVIGATOR_PID=$!
    echo "Navigator PID: $NAVIGATOR_PID"

    # Run for 30 seconds
    sleep 30

    # Clean up
    echo "Terminating test server and navigator..."
    kill $TEST_SERVER_PID 2>/dev/null
    wait $TEST_SERVER_PID 2>/dev/null
    kill $NAVIGATOR_PID 2>/dev/null
    wait $NAVIGATOR_PID 2>/dev/null
    kill $TURTLESIM_PID 2>/dev/null
    wait $TURTLESIM_PID 2>/dev/null
else
    # Mock or real-time mode: Run TurtleSim in Docker
    build_docker_image

    DOCKER_RUN_CMD="docker run -it --rm \
        --network=host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e XDG_RUNTIME_DIR=/tmp \
        --device=/dev/dri:/dev/dri \
        $IMAGE_NAME \
        bash -c \"source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node &"

    if [ "$MOCK" = true ]; then
        echo "Running TurtleSim container with mock data..."
        MOCK_COMMAND="python3 /root/ros2_ws/rock_navigator.py --mock"
        eval "$DOCKER_RUN_CMD $MOCK_COMMAND\"" || {
            echo "Container failed. Run diagnostics with: docker run -it --rm -v $(pwd):/mnt $IMAGE_NAME bash"
            cleanup_processes
            deactivate
            exit 1
        }
    else
        echo "Running TurtleSim container with real-time data on $IP:$PORT..."
        REALTIME_COMMAND="python3 /root/ros2_ws/rock_navigator.py --ws-ip $IP --ws-port $PORT"
        eval "$DOCKER_RUN_CMD $REALTIME_COMMAND\"" || {
            echo "Container failed. Run diagnostics with: docker run -it --rm -v $(pwd):/mnt $IMAGE_NAME bash"
            cleanup_processes
            deactivate
            exit 1
        }
    fi
fi

deactivate
echo "Revoking X11 access..."
xhost -local:docker || {
    echo "Warning: Failed to revoke X11 permissions. Run manually: xhost -local:docker"
}
echo "Setup complete."
