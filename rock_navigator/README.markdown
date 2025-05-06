# Smart Geological Turtle Navigation

```
   ____          _ _       _   _       _ _       
  |  _ \        | | |     | \ | |     (_) |      
  | |_) | __ _  | | |__   |  \| | __ _ _| |__    
  |  _ < / _` | | | '_ \  | . ` |/ _` | | '_ \   
  | |_) | (_| | | | |_) | | |\  | (_| | | |_) |  
  |____/ \__,_| |_|_.__/  |_| \_| \__,_|_|_.__/ 
```

The **Smart Geological Turtle Navigation** system is an intelligent robotic navigation platform built with ROS2 Humble and TurtleSim. Imagine a rover exploring a rocky planet, guided by data about different rock types (igneous, metamorphic, sedimentary). The turtle moves within a triangular map, where each corner represents a rock type, deciding its path based on rock data it receives. Using clever math and a robust control system, it navigates smoothly, stays within bounds, and adapts to data from mock sources or real-time WebSocket servers. This project showcases autonomous navigation for geological exploration, with applications in planetary science and industrial robotics.

## Project Overview
The system delivers:
- **Geological Exploration**: The turtle navigates a triangle, with each corner tied to a rock type, based on rock data inputs.
- **Smart Path Planning**: Combines rock data to pick target points and uses a two-step process (turn, then move) to get there.
- **Flexible Data**: Works with mock data, a local WebSocket server, or a remote server via IP/port.
- **Robust Design**: Runs reliably in a Docker container, reconnects to data sources if they drop, and handles errors gracefully.

## Directory Structure
```
rock_navigator/
├── scripts/
│   ├── rock_navigator.py      # Navigation brain for TurtleSim
│   ├── test_rock_server.py    # WebSocket server for rock data
├── Dockerfile                 # Docker setup for consistent runs
├── setup.sh                   # Script to launch the system
└── README.md                  # This guide
```

## Prerequisites
- **OS**: Ubuntu 22.04 (Jammy)
- **ROS2**: Humble Hawksbill
  - Install: `sudo apt install ros-humble-desktop`
  - Source: `source /opt/ros/humble/setup.bash`
- **Docker**: For containerized execution
  - Install: `sudo apt install docker.io`
  - Add user: `sudo usermod -aG docker $USER` (relogin required)
- **Python3**: For scripts and dependencies
  - Install: `sudo apt install python3 python3-pip python3-venv`
- **X11**: For TurtleSim’s graphical display
  - Install: `sudo apt install x11-xserver-utils`
  - Enable: `xhost +local:docker`

## Setup Instructions
1. **Go to Project Folder**:
   ```bash
   cd rock_navigator
   ```
2. **Make Script Executable**:
   ```bash
   chmod +x setup.sh
   ```
3. **Run the System**:
   Use `./setup.sh` with:
   - `--mock`: Use fake rock data.
   - `--test`: Use a local WebSocket server.
   - `--ip IP --port PORT`: Connect to a remote WebSocket server.

## Example Commands
1. **Mock Mode**:
   ```bash
   ./setup.sh --mock
   ```
   - **What Happens**: TurtleSim opens, draws a blue triangle, and moves with a red path based on fake rock data.
   - **Logs**:
     ```
     [INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
     [INFO] [rock_navigator]: Teleporting to vertex: [1.5, 3.19059892]
     [INFO] [rock_navigator]: Generated rock data: [...]
     [INFO] [rock_navigator]: Moving to target: [5.5, 6.65470054]
     ```

2. **Test Mode**:
   ```bash
   ./setup.sh --test
   ```
   - **What Happens**: TurtleSim navigates using data from a local WebSocket server for 30 seconds.
   - **Logs**:
     ```
     [INFO] [test_rock_server]: WebSocket server running on ws://localhost:8080
     [INFO] [rock_navigator]: Received WebSocket data: [...]
     [INFO] [rock_navigator]: Moving to target: [8.16666667, 5.5]
     ```

3. **Real-Time IP/Port**:
   ```bash
   source .venv/bin/activate
   python3 scripts/test_rock_server.py --ip 192.168.1.163 --port 8081 &
   ./setup.sh --ip 192.168.1.163 --port 8081
   ```
   - **What Happens**: TurtleSim navigates using data from a remote server.
   - **Logs**:
     ```
     [INFO] [test_rock_server]: WebSocket server running on ws://192.168.1.163:8081
     [INFO] [rock_navigator]: Moving to target: [9.5, 3.19059892]
     ```

## Navigation Logic
The turtle navigates a triangular map, like a rover exploring a rocky terrain. It uses rock data to pick a target spot, turns to face it, and moves forward, always staying inside the triangle. Here’s how it works, with visuals and simplified math.

### Logic Flow
```
+---------------------+       +---------------------+       +---------------------+
|   Rock Data Input   | ----> |  Process Rock Data  | ----> |   State Machine     |
| (Mock/WebSocket)    |       |  - Count rock types |       |   - Turning         |
| - Type, Confidence  |       |  - Pick target spot |       |   - Moving          |
| - x, y, radius      |       |  - Check triangle   |       |   - Stay in bounds  |
+---------------------+       +---------------------+       +---------------------+
          |                           |                           |
          v                           v                           v
+---------------------+       +---------------------+       +---------------------+
|   WebSocket Server  |       |   Target Spot       |       |   TurtleSim Output  |
|   - Sends rock data |       |   - Inside triangle |       |   - Blue triangle   |
|   - IP:Port         |       |   - Nearest corner  |       |   - Red path        |
+---------------------+       +---------------------+       +---------------------+
```

### Triangle Map
The turtle moves in a triangle with sides 8 units long, centered at (5.5, 5.5) in TurtleSim’s 11x11 grid. The corners are:
- **Igneous**: (1.5, 3.19)
- **Metamorphic**: (9.5, 3.19)
- **Sedimentary**: (5.5, 10.12)

```
              Sedimentary (5.5, 10.12)
                     / \
                    /   \
                   /     \
                  /       \
                 /  Turtle \
                / (x, y, θ) \
               /_____________\
Igneous (1.5, 3.19)     Metamorphic (9.5, 3.19)
```

### 1. Picking a Target Spot
The system gets rock data (like a list of rocks with types and confidence scores) and decides where to go next. Think of it like a chef mixing ingredients: each rock type (igneous, metamorphic, sedimentary) votes for its corner of the triangle, and the turtle picks a spot based on those votes.

- **How It Works**:
  - Each rock has a type and a confidence score (0 to 1). We only count rocks with confidence ≥ 0.1.
  - Count how many rocks of each type: \( c_i \) (igneous), \( c_m \) (metamorphic), \( c_s \) (sedimentary).
  - Calculate weights, like percentages of the total votes:
    \[
    w_k = \frac{c_k}{c_i + c_m + c_s}, \quad \text{where } k \text{ is igneous, metamorphic, or sedimentary}
    \]
    The weights add up to 1: \( w_i + w_m + w_s = 1 \).
  - Mix the corner positions using these weights to get the target spot \( (x_t, y_t) \):
    \[
    (x_t, y_t) = w_i \cdot (1.5, 3.19) + w_m \cdot (9.5, 3.19) + w_s \cdot (5.5, 10.12)
    \]
- **Example**:
  - Suppose we have 1 igneous, 1 metamorphic, and 2 sedimentary rocks.
  - Total rocks: \( 1 + 1 + 2 = 4 \).
  - Weights: \( w_i = \frac{1}{4} = 0.25 \), \( w_m = \frac{1}{4} = 0.25 \), \( w_s = \frac{2}{4} = 0.5 \).
  - Target spot:
    \[
    x_t = 0.25 \cdot 1.5 + 0.25 \cdot 9.5 + 0.5 \cdot 5.5 = 0.375 + 2.375 + 2.75 = 5.5
    \]
    \[
    y_t = 0.25 \cdot 3.19 + 0.25 \cdot 3.19 + 0.5 \cdot 10.12 = 0.7975 + 0.7975 + 5.06 \approx 6.655
    \]
    So, the target is \( (5.5, 6.655) \), leaning toward the sedimentary corner.

### 2. Staying Inside the Triangle (Barycentric Coordinates)
To keep the turtle inside the triangle, we check if its next position is within the boundaries, like making sure a toy car stays on a triangular track. We use **barycentric coordinates**, which is a fancy way of saying we check if a point is inside by comparing areas.

- **How It Works**:
  - Imagine the triangle as a balance board with the corners (igneous, metamorphic, sedimentary) as weights. A point is inside if it balances correctly.
  - For a point \( (x, y) \), we calculate three “sign” values to see if it’s on the right side of each triangle edge:
    \[
    \sigma_1 = (x - x_m)(y_i - y_m) - (x_i - x_m)(y - y_m)
    \]
    \[
    \sigma_2 = (x - x_s)(y_m - y_s) - (x_m - x_s)(y - y_s)
    \]
    \[
    \sigma_3 = (x - x_i)(y_s - y_i) - (x_s - x_i)(y - y_i)
    \]
    Here, \( (x_i, y_i) = (1.5, 3.19) \), \( (x_m, y_m) = (9.5, 3.19) \), \( (x_s, y_s) = (5.5, 10.12) \).
  - The point is inside if all signs are small (≤ 0.01) and consistent:
    \[
    \sigma_1 \leq 0.01 \text{ and } \sigma_2 \leq 0.01 \text{ and } \sigma_3 \leq 0.01 \text{ and they match}
    \]
  - If the point is outside, the turtle stops to stay safe.
- **Example**:
  - Check if \( (5.5, 6.655) \) is inside:
    - Plug in coordinates to compute \( \sigma_1, \sigma_2, \sigma_3 \).
    - If all are ≤ 0.01 and consistent, the point is inside (it is, as it’s a weighted average of the vertices).
  - This ensures the turtle never leaves the triangle, like a rover staying in a designated exploration zone.

### 3. Finding the Nearest Corner (Closest Vertex)
After picking a target, the system figures out which triangle corner (igneous, metamorphic, sedimentary) is closest to it, like finding the nearest landmark. This helps us understand where the turtle is headed and logs it for clarity.

- **How It Works**:
  - Measure the straight-line distance from the target \( (x_t, y_t) \) to each corner using the distance formula:
    \[
    d_k = \sqrt{(x_t - x_k)^2 + (y_t - y_k)^2}, \quad \text{where } k \text{ is igneous, metamorphic, or sedimentary}
    \]
  - Pick the corner with the smallest distance:
    \[
    \text{Closest vertex} = \text{the } k \text{ with smallest } d_k
    \]
  - Log the result (e.g., “Leaning towards sedimentary vertex”).
- **Example**:
  - For target \( (5.5, 6.655) \):
    - Igneous: \( d_i = \sqrt{(5.5 - 1.5)^2 + (6.655 - 3.19)^2} \approx \sqrt{16 + 11.83} \approx 5.27 \)
    - Metamorphic: \( d_m = \sqrt{(5.5 - 9.5)^2 + (6.655 - 3.19)^2} \approx \sqrt{16 + 11.83} \approx 5.27 \)
    - Sedimentary: \( d_s = \sqrt{(5.5 - 5.5)^2 + (6.655 - 10.12)^2} \approx \sqrt{0 + 11.83} \approx 3.44 \)
  - Sedimentary is closest, so it logs: “Leaning towards sedimentary vertex.”

### 4. Moving the Turtle (State Machine)
The turtle moves like a car: first, it turns to face the target, then it drives forward. This is handled by a **state machine** with two modes: **Turning** and **Moving**. Think of it as a simple plan: point the right way, then go.

#### State Machine Flowchart
```
+-----------------+
|    Turning        |
| - Point to target |<----+
| - Check angle     |     |
| - Avoid stalling  |     |
+-----------------+       |
      |                   |
      v                   |
+-----------------+       |
|    Moving          |-----+
| - Drive forward    |
| - Stay in triangle |
+-----------------+
```

#### How It Works
- **Turning Mode**:
  - **Goal**: Point the turtle toward the target, like turning a steering wheel.
  - **Steps**:
    - Find the angle to the target:
      \[
      \text{Target angle} = \text{atan2}(y_t - y_c, x_t - x_c)
      \]
      This gives the direction from the turtle’s current spot \( (x_c, y_c) \) to the target \( (x_t, y_t) \).
    - Calculate how far off the turtle’s current angle \( \theta_c \) is:
      \[
      \text{Angle difference} = \text{Target angle} - \theta_c
      \]
      Adjust it to be between -180° and 180° for smooth turning.
    - Set the turning speed (how fast to rotate):
      \[
      \text{Turning speed} = 10 \cdot \text{Angle difference}
      \]
      - If the speed is too small (< 0.005), use ±0.005 to keep moving.
      - Send this speed to TurtleSim to rotate.
    - Log the process:
      ```
      [INFO] Turning towards target, desired angle: 1.57, current angle: 0.0, angle difference: 1.57, iterations: 1
      ```
  - **Switching to Moving**:
    - If the angle difference is tiny (< 0.0001 radians, like 0.006°), the turtle is pointing right, so switch to moving.
    - If turning takes too long (> 4 seconds or 20 steps), switch to moving to avoid getting stuck.
      ```
      [INFO] Angle within deadband, switching to moving state
      ```

- **Moving Mode**:
  - **Goal**: Drive the turtle forward to the target, like pressing the gas pedal.
  - **Steps**:
    - Measure the distance to the target:
      \[
      \text{Distance} = \sqrt{(x_t - x_c)^2 + (y_t - y_c)^2}
      \]
    - If close (< 0.05 units), stop:
      ```
      [INFO] Reached target: [5.5, 6.655]
      ```
    - Set forward speed to avoid overshooting:
      \[
      \text{Speed} = \text{min}(1.0, \frac{\text{Distance}}{1.6})
      \]
      The 1.6 factor slows the turtle near the target for precision.
    - Predict where the turtle will be in 0.05 seconds:
      \[
      \text{Next x} = x_c + \text{Speed} \cdot \cos(\theta_c) \cdot 0.05
      \]
      \[
      \text{Next y} = y_c + \text{Speed} \cdot \sin(\theta_c) \cdot 0.05
      \]
    - Check if the next spot is inside the triangle (using barycentric coordinates). If not, stop:
      ```
      [INFO] Attempting to move outside triangle, stopping
      ```
    - Send the speed to TurtleSim to move forward.
    - Log the movement:
      ```
      [INFO] Moving linearly
      [INFO] Moving to target: [5.5, 6.655], current: [5.5, 5.5], angle: 1.57
      ```
  - **Switching to Turning**:
    - If the turtle’s angle drifts too far (> 0.01 radians, about 0.57°), go back to turning to realign:
      \[
      \text{Angle difference} > 0.01 \implies \text{Switch to turning}
      \]

#### Why This Works
- The state machine splits the task into “point” and “go,” making navigation smooth and precise.
- Barycentric checks keep the turtle safe inside the triangle.
- Timeouts and small-angle checks prevent the turtle from getting stuck or wobbling.
- It’s like a driver who turns the wheel to face the destination, then drives carefully, stopping at boundaries.

### Science Behind It
- **Geological Exploration**: The triangle mimics a terrain with different rock types. The turtle’s target reflects where the most rocks are, like a rover seeking a mineral-rich area.
- **Control System**: The turning uses a “proportional controller,” like adjusting a steering wheel based on how far you need to turn. The state machine ensures the turtle focuses on one task at a time (turning or moving).
- **Geometry**: Barycentric coordinates are a smart way to check if a point is inside a shape, perfect for keeping the turtle in bounds.

## Robustness Features
1. **Docker Setup**:
   - Runs in a consistent ROS2 Humble environment.
   - Works the same on any computer.
2. **Error Handling**:
   - `setup.sh` checks for ROS2, Docker, Python, and X11.
   - `rock_navigator.py` retries connections if the data source fails and waits up to 30 seconds for TurtleSim controls.
   - `test_rock_server.py` ensures the IP is valid.
3. **WebSocket Flexibility**:
   - Handles mock data, local servers, or remote servers.
   - Reconnects if the server drops, keeping the turtle moving.
4. **Clean Operation**:
   - `setup.sh` stops all processes when you press Ctrl+C.
   - Frees up network ports if they’re busy.
5. **Helpful Logs**:
   - Shows rock data, turtle movements, and any issues for easy debugging.

## Troubleshooting
- **TurtleSim Won’t Open**:
  - Check display: `echo $DISPLAY` (should show `:0` or similar), `xhost +local:docker`.
  - Verify controls: `ros2 service list | grep -E "teleport_absolute|set_pen"`.
- **WebSocket Problems**:
  - Check IP: `ip addr` (e.g., ensure `192.168.1.163` is listed).
  - Free port: `fuser -k 8080/tcp`.
  - Test server: `wscat -c ws://localhost:8080`.
- **Docker Issues**:
  - Check `Dockerfile`: `cat Dockerfile`.
  - View logs: `docker logs <container_id>`.


