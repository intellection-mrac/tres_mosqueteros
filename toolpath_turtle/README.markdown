# ROS TurtleSim with CSV Path Generation

This repository provides a ROS Noetic setup for controlling TurtleSim using path data generated from a custom toolpath planner. The setup uses Docker for a consistent environment, ensuring all dependencies are included without modifying core scripts. The `csv_turtle` package includes nodes to generate paths dynamically and control TurtleSim, with options for dynamic or static CSV input.

## Introduction

This project demonstrates how to control ROS TurtleSim using path data generated from a custom toolpath planner. Key components include:
- **csv_turtle.py**: A ROS node that reads path data from a CSV file and moves the turtle accordingly.
- **turtle_plan_bridge.py**: A script that interfaces with `main_plan_ABC.py` to generate `points.csv` dynamically.
- **main_plan_ABC.py**: A toolpath generation script within the `Toolpath_clustering` module.
- **Docker Environment**: A containerized ROS Noetic setup for consistency and ease of use (Use 'docker' path if visible / '.docker' path if hidden in local build)

The project supports dynamic path generation (via `turtle_plan_bridge.py`) and static CSV mode, offering flexibility for testing and deployment.

## Prerequisites

- **Docker**: Install Docker on your host machine ([Docker Installation](https://docs.docker.com/get-docker/)).
- **Host Configuration**: For GUI support (TurtleSim), ensure X11 forwarding is enabled on your host. On Linux, this typically requires `xhost +local:` before running the container.
- **Basic Knowledge**: Familiarity with Docker and ROS is helpful but not required.

## Folder Structure

The repository is organized as follows:

| Directory/File | Description |
|----------------|-------------|
| `.docker/` | Contains scripts and configuration for Docker setup. |
| `.docker/build_image.sh` | Script to build the Docker image. |
| `.docker/run_user.sh` | Script to run the Docker container with GUI support. |
| `.docker/Dockerfile` | Defines the ROS Noetic image with dependencies. |
| `.docker/entrypoint.sh` | Entry point script for the container. |
| `.docker/setup.bash` | Sources ROS and workspace environments. |
| `csv_turtle/` | ROS package containing custom nodes and toolpath planner. |
| `csv_turtle/scripts/` | Scripts for path generation and TurtleSim control. |
| `csv_turtle/scripts/csv_turtle.py` | ROS node to control TurtleSim using CSV data. |
| `csv_turtle/scripts/turtle_plan_bridge.py` | Generates path data from `main_plan_ABC.py`. |
| `csv_turtle/Toolpath_clustering/` | Contains `main_plan_ABC.py` and dependencies. |
| `csv_turtle/CMakeLists.txt` | Configuration for building the ROS package. |
| `csv_turtle/package.xml` | Metadata for the ROS package. |
| `dev_ws/` | ROS workspace where packages are built and installed. |

## Building the Docker Image

The Docker image is built using the `Dockerfile`, which sets up ROS Noetic with dependencies like `numpy`, `Pillow`, and `scikit-learn`.

1. Navigate to the repository root:
   ```bash
   cd ~/Intellection/local/ros/toolpath_turtle
   ```

2. Build the Docker image:
   ```bash
   ./.docker/build_image.sh
   ```
   - This creates an image named `ros_introduction:latest`. Note: The script may reference `lastest` due to a typo, but it should produce `latest`.

3. Verify the image:
   ```bash
   docker images
   ```
   - Look for `ros_introduction:latest` in the output.

## Running the Docker Container

To run the Docker container with the ROS environment:

1. Enable X11 forwarding on the host (Linux):
   ```bash
   xhost +local:
   ```

2. Run the container:
   ```bash
   ./.docker/run_user.sh
   ```
   - This script starts the container with GUI support (X11 forwarding) and mounts necessary volumes.

3. Set permissions inside the container:
   ```bash
   sudo chown -R $USER:$USER /dev_ws
   ```
   - This ensures the current user has write access to the `/dev_ws` workspace. Replace `$USER` with your username (check with `whoami`) if needed.

## Building the ROS Workspace

The ROS workspace (`/dev_ws`) contains the `csv_turtle` package. Build it to ensure all nodes are available.

1. Build the workspace
   ```bash
   .docker/build_image.sh
   ```
   
2. Build the workspace:
   ```bash
   .docker/run_user.sh
   ```

3. Source the workspace and modify permission:
   ```bash
   source /dev_ws/devel/setup.bash && sudo chown -R [YOUR USER NAME] /dev_ws 
   ```
   - This makes the `csv_turtle` package available. The `.docker/setup.bash` script automates sourcing `/opt/ros/noetic/setup.bash` and `/dev_ws/devel/setup.bash`.

## Starting ROS Core

1. Start the ROS master in a terminal inside the container (Terminator):
   ```bash
   roscore
   ```
   - Keep this terminal open, as `roscore` is required for node communication.

## Running TurtleSim

1. Start TurtleSim in another terminal:
   ```bash
   rosrun turtlesim turtlesim_node
   ```
   - This opens the TurtleSim GUI, displaying the turtle’s movement.

## Running the Custom Nodes

The `csv_turtle` package includes two key scripts:
- **turtle_plan_bridge.py**: Generates path data (e.g., for a letter like ‘F’) using `main_plan_ABC.py` and saves it as `points.csv`.
- **csv_turtle.py**: Reads `points.csv` and controls TurtleSim to follow the path.

1. Generate the path:
   ```bash
   rosrun csv_turtle turtle_plan_bridge.py
   ```
   - **Prompts**:
     - **Letter**: Enter a letter (A–Z, e.g., `F`).
     - **Grid Size**: Enter a value (default 300; up to 1000 if modified).
     - **Cluster Size**: Enter a value (default 25).
   - **Output**: Creates `/dev_ws/src/csv_turtle/scripts/points.csv`.

2. Control TurtleSim:
   ```bash
   rosrun csv_turtle csv_turtle.py
   ```
   - Waits for `points.csv`, loads it, scales the path to fit TurtleSim (0.5–10.5), and moves the turtle.

## Using Static CSV Mode

For testing with a pre-existing `points.csv`:
```bash
rosrun csv_turtle csv_turtle.py _use_static_csv:=true
```
- This mode uses the existing `/dev_ws/src/csv_turtle/scripts/points.csv` without waiting for dynamic generation.

## Expected Output

- **TurtleSim**: The turtle traces the specified letter (e.g., ‘F’), scaled to the TurtleSim window.
- **Logs**:
  - `turtle_plan_bridge.py`: Outputs like `Generated toolpath for letter 'F' with total reward: X.XX` and `Saved path data to .../points.csv`.
  - `csv_turtle.py`: Outputs like `Waiting for CSV file...`, `Loaded and scaled X points`, `Moved to (x, y)`.
- **Files**:
  - `/dev_ws/src/csv_turtle/scripts/points.csv`: Generated path data.
  - `/dev_ws/src/csv_turtle/Toolpath_clustering/runs/...`: Visualizations (`heatmap_F.png`, `path_data_F.csv`).

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **Permission Errors** | Run `sudo chown -R $USER:$USER /dev_ws` and verify with `ls -l /dev_ws`. |
| **Docker Build Fails** | Check `.docker/build_image.sh` output. Ensure `pip3` dependencies (`numpy==1.23.5`, `Pillow==10.4.0`) are installed correctly. |
| **TurtleSim GUI Fails** | Verify X11 forwarding (`xhost +local:` on host) and check `run_user.sh` for `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`. |
| **Node Errors** | Ensure `roscore` and `turtlesim_node` are running. Source `/dev_ws/devel/setup.bash` before running nodes. |
| **Path Distortion** | Check `grid_size` (e.g., 300) and scaling logic in `csv_turtle.py`. |

## Additional Notes

- **Dynamic vs. Static Mode**: Dynamic mode generates `points.csv` via `turtle_plan_bridge.py`. Static mode is useful for testing with a known CSV file.
- **Custom Inputs**: Adjust grid size and cluster size in `turtle_plan_bridge.py` prompts. Defaults are optimized for typical use cases.
- **Dependencies**: The `Dockerfile` includes `python3-tk`, `numpy`, `Pillow`, `scikit-learn`, `scipy`, and `matplotlib` for `Toolpath_clustering`.

This guide ensures you can build and run the ROS setup to control TurtleSim with dynamically generated paths. For further assistance, check the troubleshooting section or consult the ROS documentation ([ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)).

## Key Citations
- [Docker Installation Guide](https://docs.docker.com/get-docker/): Instructions for installing Docker on various platforms.
- [ROS Noetic Tutorials](http://wiki.ros.org/ROS/Tutorials): Official tutorials for learning ROS Noetic.
