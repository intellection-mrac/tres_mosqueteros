
# UR10e Industrial Reconstruction Controller

## Overview

The **UR10e Industrial Reconstruction Controller** is a robust ROS node that provides precise control over an industrial 3D scanning process using a UR10e robot. By monitoring IO feedback from the robot, this node automatically triggers the start and stop of a 3D reconstruction process. It is built on top of the MRAC UR Perception framework, which leverages a Dockerized ROS workflow to ensure reproducibility, ease of deployment, and GPU acceleration for processing-intensive tasks.

This documentation offers an exhaustive overview of the node’s design, hardware/software dependencies, Docker-based development setup, code execution flow, and troubleshooting recommendations. It features multiple ASCII diagrams that visually present the system architecture and execution logic.

## Table of Contents

1. [Features](#features)
2. [Installation & Docker Setup](#installation--docker-setup)
3. [Code Execution Flow](#code-execution-flow)
4. [Usage Instructions](#usage-instructions)
5. [Technical Explanation](#technical-explanation)
6. [Troubleshooting](#troubleshooting)
7. [Contributions](#contributions)
8. [License](#license)
10. [Contact](#contact)

---

## Features

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

### Prerequisites

Before setting up the environment, ensure that you have the following installed and configured on your host machine:

- **Docker:**  
  [Install Docker](https://docs.docker.com/get-docker/) for containerization.

- **NVIDIA Container Toolkit:**  
  [Install NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) for enabling GPU acceleration inside Docker containers.

- **Visual Studio Code (VSCode):** *(Optional but Recommended)*  
  [Download VSCode](https://code.visualstudio.com/) for a seamless development and debugging experience.

- **UR10e Robot:**  
  Verify that your UR10e robot is properly configured and connected. Ensure the machine's IP address is set to `192.168.56.1`.

### Docker Workflow for MRAC UR Perception

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

**Key Components:**

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

Below is a beautifully composed ASCII diagram that outlines the node's execution flow. All lines have been aligned for clarity:

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
|                Instantiate ReconIO Class                   |
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

**Execution Details:**

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
    ```

### 3D Reconstruction Example

Launch the reconstruction package:

```bash
roslaunch commander reconstruction.launch
```

Then, use the Commander Node interface to manipulate the robot and capture data.

---

## Technical Explanation

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

### Service Request Generation

The function `gen_recon_msg()` generates two types of ROS service requests:

- **StartReconstructionRequest:**  
  Contains parameters for camera frames, TSDF values (voxel length, truncation distance), and RGBD settings.

- **StopReconstructionRequest:**  
  Automatically constructs a file path for the output mesh, including a timestamp for unique identification.

### Blocking Mechanism

After issuing the stop command, the function `wait_for_recon_to_finish()` blocks further execution until the reconstruction process signals completion. This ensures sequential integrity and prevents data corruption.

---

## Troubleshooting

- **Docker GPU Integration:**  
  Ensure the NVIDIA Container Toolkit is correctly installed and that the container runs with GPU support.

- **Camera Connectivity:**  
  Confirm physical connections, check the relevant ROS topics, and verify camera drivers (ZED or Azure Kinect) are running.

- **ROS Communication:**  
  Verify the ROS master is active and that all nodes (IO, reconstruction, perception) are properly communicating.

- **Environment Variables:**  
  Check that necessary environment variables (like IP addresses and ROS_MASTER_URI) are correctly set within the Docker container.

---

## Contributions

- **Neeeeeeeil**
- **Nacho**
- **Santosh**

Contributions to improve functionality, robustness, and academic depth are welcome. Please submit issues or pull requests via the GitHub repository.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---

## Contact

For further details, academic inquiries, or collaboration opportunities, please reach out to:

**Email:** santosh.shenbagamoorthy@students.iaac.net  
**Organization:** MRAC-IAAC

---

## Final Remarks

The **UR10e Industrial Reconstruction Controller** integrates robotics, computer vision, and 3D scanning within an industrial context. With its Dockerized MRAC UR Perception framework, this node supports reproducible and scalable deployments, serving as a robust platform for academic research and industrial automation. We hope this documentation helps you to fully leverage your UR10e robot for advanced 3D reconstruction applications.

---

*End of Documentation*
