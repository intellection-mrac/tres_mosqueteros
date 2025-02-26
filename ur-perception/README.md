
# UR10e Industrial Reconstruction Controller

---

## Overview

---

The **UR10e Industrial Reconstruction Controller** is a robust ROS node that provides precise control over an industrial 3D scanning process using a UR10e robot. By monitoring IO feedback from the robot, this node automatically triggers the start and stop of a 3D reconstruction process. It is built on top of the MRAC UR Perception framework, which leverages a Dockerized ROS workflow to ensure reproducibility, ease of deployment, and GPU acceleration for processing-intensive tasks.

This documentation offers an exhaustive overview of the node’s design, hardware/software dependencies, Docker-based development setup, code execution flow, and troubleshooting recommendations. It features multiple ASCII diagrams that visually present the system architecture and execution logic.

---

## Table of Contents  

1. **[Overview](#overview)**  
2. **[System Overview](#system-overview)**  
3. **[Features](#features)**  
4. **[Installation & Docker Setup](#installation--docker-setup)**  
   - [Software Dependencies](#software-dependencies)  
   - [Repository Components](#repository-components)  
   - [Installation Instructions](#installation-instructions)  
   - [Key Components](#key-components)  
5. **[Code Execution Flow](#code-execution-flow)**  
6. **[Execution Details](#execution-details)**  
7. **[Usage Instructions](#usage-instructions)**  
   - [Running the Reconstruction Controller Node](#running-the-reconstruction-controller-node)  
   - [Launching Additional Perception Pipelines](#launching-additional-perception-pipelines)  
   - [3D Reconstruction Example](#3d-reconstruction-example)  
8. **[Technical Explanation](#technical-explanation)**  
   - [ROSPy Communication Diagram](#ropy-communication-diagram)  
   - [IO Processing and Service Calls](#io-processing-and-service-calls)  
   - [Service Request Generation](#service-request-generation)  
9. **[Hand-Eye Calibration](#hand-eye-calibration)**  
10. **[Advanced Configuration & Optimization](#advanced-configuration--optimization)**  
    - [Optimizing Camera Settings](#optimizing-camera-settings)  
    - [Optimizing Robot Movements](#optimizing-robot-movements)  
    - [Mesh Reconstruction Parameters](#mesh-reconstruction-parameters)  
    - [GPU Acceleration](#gpu-acceleration)  
    - [Blocking Mechanism](#blocking-mechanism)  
11. **[Testing and Debugging](#testing-and-debugging)**  
    - [ROS Communication Test](#ros-communication-test)  
    - [Camera and Robot Connectivity Test](#camera-and-robot-connectivity-test)  
    - [Mesh Reconstruction Validation](#mesh-reconstruction-validation)  
    - [Debugging Tips](#debugging-tips)  
12. **[Mesh Reconstruction with Grasshopper Python Script](#mesh-reconstruction-with-grasshopper-python-script)**  
    - [Alpha Shape Reconstruction](#alpha-shape-reconstruction)  
    - [Ball Pivoting Algorithm (BPA)](#ball-pivoting-algorithm-bpa)  
    - [Poisson Surface Reconstruction](#poisson-surface-reconstruction)  
    - [Usage Workflow](#usage-workflow)  
    - [Python Script for Mesh Reconstruction](#python-script-for-mesh-reconstruction)  
      - [Load the Point Cloud](#load-the-point-cloud)  
      - [Create a Mesh Using Poisson Surface Reconstruction](#create-a-mesh-using-poisson-surface-reconstruction)  
      - [Save the Mesh](#save-the-mesh)  
      - [Visualize the Mesh](#visualize-the-mesh)  
    - [Integrating with Grasshopper](#integrating-with-grasshopper)  
13. **[Troubleshooting](#troubleshooting)**  
14. **[Contributions](#contributions)**  
15. **[License](#license)**  
16. **[Contact](#contact)**  
17. **[Final Remarks](#final-remarks)**  

---

## System Overview

---

The **UR Perception** system connects depth sensing cameras (ZED Camera or Azure Kinect) with the **UR10e robot** to perform advanced **robotic perception tasks**. The system enables:

- **3D Environment Mapping** using depth perception data.
- **Robot Manipulation** with **MoveIt** integration for UR10e.
- **3D Mesh Reconstruction** from captured point clouds.
- **Seamless integration with Grasshopper** for optimization and visualization of 3D models.

The system workflow involves:

1. **Depth Data Acquisition** using either ZED Camera or Azure Kinect.
2. **Robot Manipulation** via MoveIt and ROSPy for UR10e.
3. **Point Cloud Processing** and **Mesh Reconstruction** using Python-based algorithms.
4. **3D Visualization and Optimization** through Grasshopper 3D.

```plaintext
+------------------+         +-----------------+        +-------------------+
|  ZED Camera     |         |  UR10e Robot    |        |  Grasshopper 3D    |
| (Depth Sensing) |----->---| (Robot Control) |----->--| (3D Reconstruction) |
+------------------+         +-----------------+        +-------------------+
```

---

## Features

---

- **IO Monitoring:**  
  Listens to the `/ur_hardware_interface/io_states` topic to detect digital output signals from the UR10e robot.

- **Reconstruction Process Control:**  
  Seamlessly initiates (`/start_reconstruction`) and terminates (`/stop_reconstruction`) the 3D reconstruction process using ROS service calls with dynamically generated parameters.

- **State Debouncing & Signal Transition Handling:**  
  Employs internal state management to ensure service calls occur solely on the transition of the “capture” signal, thereby avoiding superfluous triggers.

- **Timestamped Data Output:**  
  The reconstructed mesh is saved as a PLY file with an appended timestamp in a preconfigured directory (default: `/home/capture`), facilitating easy historical data management.

- **Synchronization and Process Blocking:**  
  Incorporates a blocking mechanism to prevent new triggers until the current reconstruction process is completely finished, ensuring data consistency.

- **Docker-Based Workflow:**  
  Part of the broader MRAC UR Perception ecosystem, this node benefits from a Dockerized ROS environment that ensures consistency, GPU acceleration, and simplified dependency management.

- **Extensive Perception Pipeline Support:**  
  Although this node is specific to reconstruction control, the repository also supports camera drivers (ZED and Azure Kinect), MoveIt configurations, and Commander nodes for comprehensive UR10e control.

---

## Installation & Docker Setup

---

Before proceeding, ensure the following dependencies and tools are installed and setup:

---

### Software Dependencies

**Docker**: For containerized development. Install Docker

**NVIDIA Container Toolkit**: For GPU acceleration. Install NVIDIA Container Toolkit

**Visual Studio Code (VSCode)**: For code editing and debugging. Download VSCode

**ROS (Robot Operating System)**: ROS is required for communication between components. Install ROS

**UR10e Robot**: Ensure your UR10e robot is properly connected. The IP address should be set to 192.168.56.1.

**Python**: Ensure Python 3.x and pip are installed.

---

### Repository Components

1. **Camera Drivers**:
ZED Camera Driver: For depth perception using the ZEDm camera.
Azure Kinect Driver: For depth perception using the Azure Kinect camera.

2. **MoveIt Configurations**:
Pre-configured MoveIt setup for UR10e robots to handle robotic motion planning.

3. **Industrial Reconstruction Package**:
Tools for 3D reconstruction from point clouds, used to build digital models of environments.

4. **Commander Node**:
A ROS node that interfaces with the UR10e robot for direct control and manipulation.

---

### Installation Instructions

The MRAC UR Perception repository utilizes Docker to facilitate consistent and reproducible ROS environments. Follow these steps to set up and run the system:

1. **Clone the Repository**

   Open a terminal and execute:

   ```
   git clone https://github.com/your-username/MRAC-UR-Perception.git
   cd MRAC-UR-Perception
   ```

2. **Build the Docker Image**

   Build the Docker image by running:

   ```
   .docker/build_image.sh
   ```

3. **Run the Docker Container**

   Launch the container using:

   ```
   .docker/run_user_nvidia.sh
   ```

   After starting the container, you may be prompted to change the ownership of the development workspace. Execute the following command (replace `[YOUR USER NAME]` accordingly):

   ```
   sudo chown -R [YOUR USER NAME] /dev_ws
   ```

4. **Open a Terminal Within Docker**

   Use your preferred terminal emulator (such as Terminator) inside the container to execute ROS commands.
   
---

### Key Components 

- **README.md:**  
  This file provides detailed usage instructions, technical explanations, and diagrams.

- **package.xml & CMakeLists.txt:**  
  Define the package dependencies and build configurations required by ROS.

- **commander/scripts/recon_io.py:**  
  Implements the primary ROS node responsible for IO monitoring and 3D reconstruction control.

- **.docker Directory:**  
  Contains scripts and configurations for Docker-based deployment.

---

## Code Execution Flow

---

Below is a ASCII diagram that outlines the node's execution flow. All lines have been aligned for clarity:

```
+--------------------------------------------------------------+
|                          main()                              |
+--------------------------------------------------------------+
             │
             ▼
+--------------------------------------------------------------+
|    Initialize ROS Node ("recon_io") (via rosrun in Docker)   |
+--------------------------------------------------------------+
             │
             ▼
+--------------------------------------------------------------+
|                Instantiate ReconIO Class                     |
|        - Subscribe to IO states                              |
|        - Wait for reconstruction service proxies             |
+--------------------------------------------------------------+
             │
             ▼
+--------------------------------------------------------------+
|                         run() Loop                           |
|  while not rospy.is_shutdown():                              |
|      ┌───────────────────────────────┐                       |
|      │  Check "capture" flag state   │                       |
|      └──────────────┬────────────────┘                       |
|                     │                                        |
|         ┌───────────┴────────────┐                           |
|         │    Edge Detection      │                           |
|         └───────┬────────┬───────┘                           |
|                 │        │                                   |
|       (Rising Edge)  (Falling Edge)                          |
|                 │        │                                   |
|                 ▼        ▼                                   |
|     +----------------+  +------------------------------+     |
|     | /start_recon   |  |    /stop_recon               |     |
|     | - Generate     |  | - Save timestamped file      |     |
|     |   parameters   |  | - Reset flag                 |     |
|     +----------------+  +------------------------------+     |
|                 │        │                                   |
|                 └────┬───┘                                   |
|                      ▼                                       |
|          +-----------------------------+                     |
|          | wait_for_recon_to_finish()  |  (blocking loop)    |
|          +-----------------------------+                     |
+--------------------------------------------------------------+
             │
             ▼
+--------------------------------------------------------------+
|                rospy.spin() / Shutdown                       |
+--------------------------------------------------------------+
```

---

## Execution Details

---

1. **Initialization:**  
   - The ROS node is initialized.
   - An instance of the ReconIO class is created that sets up IO subscriptions and waits for the required service proxies.
2. **IO Signal Processing:**  
   - The node continuously listens to the `/ur_hardware_interface/io_states` topic.
   - It updates its internal state based on the digital "capture" signal.
3. **Service Invocation:**  
   - On a rising edge (False to True), the node issues a `/start_reconstruction` request with dynamically generated parameters.
   - On a falling edge (True to False), the node invokes `/stop_reconstruction`, saves a timestamped file, and enters a blocking loop until the reconstruction process completes.
4. **Sequential Consistency:**  
   - The blocking mechanism ensures that no new commands are initiated until the current reconstruction process is entirely finished.

---

## Usage Instructions

---

### Running the Reconstruction Controller Node

Inside the Docker container, follow these steps:

1. **Source the Workspace**

   ```bash
   source /dev_ws/devel/setup.bash
   ```

2. **Run the Node with rosrun**

   ```bash
   rosrun commander recon_io.py
   ```

   The node subscribes to the `/ur_hardware_interface/io_states` topic and triggers the 3D reconstruction process based on the received digital IO signals.

---

### Launching Additional Perception Pipelines

You can run additional perception nodes within the container:

- **ZED Camera:**

  ```bash
  roslaunch zed_wrapper zedm.launch
  ```

- **Azure Kinect:**

  ```bash
  roslaunch azure_kinect_ros_driver driver.launch
  ```

- **Robot Driver:**
  - *Simulated Robot:*

    ```bash
    roslaunch ur10e_moveit_config demo.launch
    ```

  - *Real Robot with ZED Camera (mounted):*

    ```bash
    roslaunch commander ur10e_zed_commander.launch
    ```

  - *Real Robot with Azure Kinect (stationary):*

    ```bash
    roslaunch commander ur10e_ka_commander.launch

---```

---

### 3D Reconstruction Example

Launch the reconstruction package:

```bash
roslaunch commander reconstruction.launch
```

Then, use the Commander Node interface to manipulate the robot and capture data.

---

## Technical Explanation

---

### **ROSPy Communication Diagram**:

```plaintext
  +-----------+        +----------------+        +------------+
  |  Python   |        |  ROS           |        |  UR10e     |
  |  Node     |--------|  Master        |--------|  Robot     |
  |  (ROSPy)  |        |  Communication |        |  Control   |
  +-----------+        +----------------+        +------------+
```

---

### IO Processing and Service Calls

The node’s core logic converts IO signals into service calls. The following pseudo-code outlines the process:

```
+--------------------------------------------------+
|                   IO Callback                    |
+--------------------------------------------------+
            │
            ▼
+--------------------------------------------------+
|              Read IO States                      |
|              - io                                |
|              - capture                           |
+--------------------------------------------------+
            │
            ▼
+--------------------------------------------------+
|             Update internal flag                 |
|                  (last_state)                    |
+--------------------------------------------------+
            │
            ▼
+--------------------------------------------------+
|   if (capture == True) and (last_state is False):|
|         → Trigger /start_reconstruction          |
|           (Generate parameters, set flag)        |
+--------------------------------------------------+
            │
            ▼
+--------------------------------------------------+
|   if (capture == False) and (last_state is True):|
|         → Trigger /stop_reconstruction           |
|           (Save file, reset flag, block until    |
|            recon completes)                      |
+--------------------------------------------------+
```

---

### Service Request Generation

The function `gen_recon_msg()` generates two types of ROS service requests:

- **StartReconstructionRequest:**  
  Contains parameters for camera frames, TSDF values (voxel length, truncation distance), and RGBD settings.

- **StopReconstructionRequest:**  
  Automatically constructs a file path for the output mesh, including a timestamp for unique identification.

---

##  Hand-Eye Calibration

---

Hand-eye calibration is a critical process to synchronize the robot's movements with camera data. This ensures that the camera data is correctly mapped to the robot's workspace. Follow these steps for calibration:

1. Choose the camera image topics for your camera.
2. Select the appropriate frames for calibration.
3. Manually move the robot to different poses.
4. Capture data from 10-15 poses by clicking “Save Sample.”
5. Save the camera pose data to a new launch file and integrate it into your project's launch node.

For a detailed tutorial, follow the official **Hand-Eye Calibration** guide in the documentation.

------

## Advanced Configuration & Optimization

---

The following configurations allow you to fine-tune the system for improved performance and efficiency. These settings are particularly useful in environments with large datasets or intricate 3D objects.

---

### Optimizing Camera Settings:

- **ZED Camera**: Adjust the **resolution**, **frame rate**, and **depth range** based on your scanning needs. Higher resolution and frame rates improve detail but require more processing power.

Example:

  ```xml
<param name="camera_resolution" value="HD720" />
<param name="camera_fps" value="30" />
<param name="depth_stabilization" value="true" />
  ```

---

### Optimizing Robot Movements:

   Use MoveIt’s motion planning and trajectory optimization features to reduce execution time and increase path accuracy.

Example:
```yaml
move_group:
  max_velocity_scaling_factor: 0.8
  max_acceleration_scaling_factor: 0.5
```

---

### Mesh Reconstruction Parameters:

   Fine-tune mesh reconstruction parameters in the Industrial Reconstruction Package to balance between computation speed and mesh detail.

Example:

```python
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud, alpha=0.02)
```

---

### GPU Acceleration:

Ensure that the NVIDIA Container Toolkit is installed to take advantage of GPU acceleration for depth data processing. This significantly speeds up the computation of point cloud processing and mesh reconstruction tasks.

---

### Blocking Mechanism

After issuing the stop command, the function `wait_for_recon_to_finish()` blocks further execution until the reconstruction process signals completion. This ensures sequential integrity and prevents data corruption.

---

## Testing and Debugging

---

### ROS Communication Test:

Use rosnode list to check if all nodes are running and communicating correctly.

```bash
rosnode list
```

---

### Camera and Robot Connectivity Test:

Verify that both the camera and robot are connected to the ROS network.

```bash
rostopic list
```

---

### Mesh Reconstruction Validation:

After performing mesh reconstruction, validate the result visually and check the mesh for any anomalies such as holes or noise. Adjust reconstruction parameters if needed.

---

### Debugging Tips:

   **Camera Not Detected**: Check if the camera driver is launched and connected.
   **Robot Not Moving**: Verify that the MoveIt configuration is properly loaded.

---

## Mesh Reconstruction with Grasshopper Python Script

---

### Virtual Environment Setup in Grasshopper
To ensure modularity and reproducibility, this project uses a virtual environment directly in Grasshopper.

**Automatic Virtual Environment Creation**
Inside Grasshopper’s Python component, simply specify the virtual environment in the script:

```python
#! python 3
# venv : testOpen3D
# r: open3d
import open3d as o3d
```

This automatically creates and manages a virtual environment within Grasshopper, making the workflow faster and more efficient.

**Installing Dependencies in Grasshopper**
The necessary libraries are installed inside the Grasshopper Python component:

```python
#! python 3
# venv : testOpen3D
# r: open3d
# r: numpy
# r: scipy
# r: rhino3dm
import open3d as o3d
import numpy as np
import scipy
import Rhino.Geometry as rg
```

This ensures that all dependencies remain isolated, avoiding conflicts with the system-wide Python environment.

---

### **Mesh reconstruction techniques**

This section outlines three advanced mesh reconstruction techniques using Open3D, each tailored to specific types of point clouds.

---

### Alpha Shape Reconstruction

	Description:
	The Alpha Shape algorithm is ideal for reconstructing complex,organic shapes from point clouds. It works by triangulating the point cloud and adapting the shape based on an alpha parameter.

**Algorithm Flow**

Input
	Point cloud (.PLY file)
Pre-processing
	Downsample point cloud to reduce computation time.
	Estimate normals to improve mesh orientation.
Triangulation
	Construct an alpha shape mesh, which adapts to the point cloud's structure, maintaining sharp features.
Post-processing
	Crop the bounding box for refinement.

**Best Use Cases**

Organic structures. 
Closed surfaces with varying point density. 

**Example Usage**

```python
mesh_o3d = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud, alpha)
```

---

### Ball Pivoting Algorithm (BPA)

	Description:
	The Ball Pivoting Algorithm (BPA) is a geometry-driven method for reconstructing meshes by rolling a virtual ball over the point cloud and forming triangles where the ball intersects.

**Algorithm Flow**

Input
	Point cloud (.PLY file).
Pre-processing
	Downsample and estimate normals.
Pivoting
	The virtual ball pivots over points to form triangles.
Gap Filling
	Detect open edges and fill the gaps by adding more triangles.
Smoothing
	Smoothing the final mesh for better quality.

**Best Use Cases**

Structured models with edge details. 
Uniform point cloud density. 

**Example Usage**

```python
mesh_o3d = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    point_cloud, o3d.utility.DoubleVector([ball_radius, ball_radius * 1.5, ball_radius * 2])
)
```

---

### Poisson Surface Reconstruction

	Description:
	Poisson Surface Reconstruction generates a smooth and watertight surface by solving an implicit function that fits the point cloud.

**Algorithm Flow**

Input
	Point cloud with estimated normals.
Pre-processing
	Downsample and correct normals.
Reconstruction
	Use Poisson's equation to compute the smooth surface.
Post-processing
	Crop bounding box and refine the mesh.

**Best Use Cases**

Smooth and watertight meshes. 
High-detail reconstructions. 

**Example Usage**

```python
mesh_o3d, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud, depth=poisson_depth)
```

---

### Usage Workflow

After scanning and processing the point cloud, you can reconstruct the mesh in Rhino/Grasshopper. Here’s how the full workflow unfolds - 

**Scanning the Object**
	Move the UR10e robot to different positions while the ZEDm camera captures the object.
	The ROS node listens to the robot’s IO signals and triggers the scanning process.

**Point Cloud to Mesh Conversion**
    Run the Open3D script in Rhino/Grasshopper to process the point cloud and convert it into a mesh.
    Choose the appropriate reconstruction method (Alpha Shape, BPA, or Poisson).

**Mesh Manipulation in Rhino/Grasshopper**
    Utilize Rhino's modeling tools to manipulate, refine, and visualize the 3D mesh.
    Perform tasks like mesh smoothing, detail addition, or exporting to CAD formats.

---

### Python Script for Mesh Reconstruction:

Integrate the captured point cloud data into Grasshopper for mesh reconstruction. Here's a basic Python script for reconstructing the mesh from point cloud data:

```python
import open3d as o3d

# Load the point cloud
point_cloud = o3d.io.read_point_cloud("point_cloud.ply")

# Create a mesh from the point cloud using Poisson Surface Reconstruction
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud, depth=9)[0]

# Save the mesh
o3d.io.write_triangle_mesh("reconstructed_mesh.obj", mesh)

# Visualize the mesh
o3d.visualization.draw_geometries([mesh])
```

---

### Integrating with Grasshopper:

Use the Python script above inside Grasshopper script component to import a sample .ply file for processing inside rhino's script component.

```plaintext
+---------------------+          +-----------------+      +-------------------+
|  UR10e Robot        |   --->   |  ROSPy Node     | ---> |  Grasshopper      |
|  (Robot Movement)   |          |  (Data Capture) |      |  (Mesh Update)    |
+---------------------+          +-----------------+      +-------------------+
```

Grasshopper provides a visual scripting environment to manipulate meshes and interact with Rhino. In this workflow, the Open3D scripts can be used directly within the Grasshopper Python component to achieve seamless integration.

**Load the Point Cloud**
	Import the .PLY point cloud into Grasshopper using a file path.
**Set Reconstruction Parameters**
	Define parameters such as alpha, radius_factor, and poisson_depth based on the selected method.
**Generate the Mesh**
	Trigger the mesh reconstruction using the Python component in Grasshopper.
**Output to Rhino**
	The resulting mesh can be previewed and manipulated directly within Rhino3D.

The following flowchart illustrates the code execution in ROS to Open3d / Grasshopper pipeline

```plaintext
+-----------------------------------------------------------+
|                Start ROS Node (recon_io)                  |
|  (Subscribes to /ur_hardware_interface/io_states)         |
+-----------------------------------------------------------+
                  │
                  ▼
+-----------------------------------------------------------+
|   Wait for "capture" signal from robot's IO feedback      |
|   (Trigger start/stop reconstruction process)             |
+-----------------------------------------------------------+
                  │
                  ▼
+-----------------------------------------------------------+
|    Process Point Cloud in Open3D (via Rhino/Grasshopper)  |
|    - Alpha Shape, BPA, or Poisson                         |
+-----------------------------------------------------------+
                  │
                  ▼
+-----------------------------------------------------------+
|   Final Mesh Output and Manipulation in Rhino3D/Grasshopper|
+-----------------------------------------------------------+
```

---

## Troubleshooting 

---

- **Docker GPU Integration:**  
Ensure the NVIDIA Container Toolkit is correctly installed and that the container runs with GPU support.

- **Camera Connectivity:**  
Confirm physical connections, check the relevant ROS topics, and verify camera drivers (ZED or Azure Kinect) are running.

- **ROS Communication:**  
Verify the ROS master is active and that all nodes (IO, reconstruction, perception) are properly communicating.

- **Environment Variables:**  
Check that necessary environment variables (like IP addresses and ROS_MASTER_URI) are correctly set within the Docker container.

- **Robot not detecting IO signals**
Ensure the robot is connected and the IO states are correctly subscribed to in ROS. Verify the configuration of /ur_hardware_interface/io_states.

- **Camera not providing data**
Check the ZEDm camera connection and ROS node status using rosnode list to verify active nodes.

- **Mesh reconstruction is noisy or incomplete**
Adjust reconstruction parameters, such as alpha, radius_factor, or poisson_depth, based on the point cloud density and detail.

---

## Contributors

---

- **Neeeeeeeil**
- **Nacho**
- **Santosh**

Contributions to improve functionality, robustness, and academic depth are welcome. Please submit issues or pull requests via the GitHub repository.

---

## License

---

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---

## Contact

---

For further details, academic inquiries, or collaboration opportunities, please reach out to:

**Email:** santosh.shenbagamoorthy@students.iaac.net  
**Organization:** MRAC-IAAC

---

## Final Remarks

---

The **UR10e Industrial Reconstruction Controller** integrates  robotics, computer vision, and 3D scanning within an industrial context. With its Dockerized MRAC UR Perception framework, this node supports reproducible and scalable deployments, serving as a robust platform for academic research and industrial automation. 

---

*End of Documentation*
