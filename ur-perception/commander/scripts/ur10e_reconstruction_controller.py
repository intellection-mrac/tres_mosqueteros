#!/usr/bin/env python3
"""
UR10e Reconstruction Controller Node

This ROS node listens for a UR10e signal (published on the "/ur_signal" topic as a Bool).
When the signal is high (True), the node will:
  - Set the robot's end effector to the camera tool (e.g. "rgb_camera_tcp").
  - Optionally perform any motion planning needed (this example can easily be extended).
  - Start an industrial reconstruction process using the 'StartReconstruction' service.

When the signal goes low (False), the node will:
  - Stop the reconstruction using the 'StopReconstruction' service.
  - Export the reconstructed mesh to a .ply file whose filename is based on the current date and time.

This script uses ROS service proxies to call services for setting the end effector, planning
trajectories, executing trajectories, and controlling the reconstruction.
"""

import rospy
from std_msgs.msg import Bool
from datetime import datetime
from math import pi
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

# Import reconstruction service definitions
from industrial_reconstruction_msgs.srv import (
    StartReconstruction, StartReconstructionRequest,
    StopReconstruction, StopReconstructionRequest
)

# Import additional services from our commander package for robot motions and control.
from commander.srv import SetEe, PlanGoal, ExecuteTrajectory
from commander.msg import Goal

# Global flag to track the reconstruction state.
is_reconstructing = False

# Option flag to capture reconstruction
CAPTURE = True

# Initialize global service proxies (they will be set up later in the main() function)
start_recon_srv = None
stop_recon_srv = None
set_ee_srv = None
plan_goal_srv = None
execute_trajectory_srv = None

# Directory where the exported .ply file will be saved.
SAVE_DIR = "/home/intellection/capture"

def gen_recon_msg(path: str) -> (StartReconstructionRequest, StopReconstructionRequest):
    """
    Generates a pair of service requests:
      - StartReconstructionRequest: Contains settings for the reconstruction process.
      - StopReconstructionRequest: Specifies the output file path for the reconstructed mesh.

    Parameters:
      path (str): Base directory and filename prefix for saving the .ply result.

    Returns:
      (start_srv_req, stop_srv_req) tuple
    """
    # Configure how the reconstruction process will work.
    start_srv_req = StartReconstructionRequest()
    start_srv_req.tracking_frame = "rgb_camera_tcp"   # Frame attached to your camera tool
    start_srv_req.relative_frame = "base_link"          # Reference frame (typically the robot base)
    start_srv_req.translation_distance = 0.0            # No minimum translation required to trigger update
    start_srv_req.rotational_distance = 0.0             # No minimum rotation required
    start_srv_req.live = True                           # Enable live reconstruction mode
    # TSDF parameters for signed distance function based reconstruction.
    start_srv_req.tsdf_params.voxel_length = 0.001      # Voxel size
    start_srv_req.tsdf_params.sdf_trunc = 0.002         # Truncation value for the TSDF
    start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
    start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
    # RGBD parameters for depth image handling.
    start_srv_req.rgbd_params.depth_scale = 1
    start_srv_req.rgbd_params.depth_trunc = 1.0
    start_srv_req.rgbd_params.convert_rgb_to_intensity = False

    # Create a stop service request which will export the mesh.
    stop_srv_req = StopReconstructionRequest()
    timestamp = datetime.now().strftime("%m_%d_%H_%M")
    stop_srv_req.mesh_filepath = path + timestamp + ".ply"

    return start_srv_req, stop_srv_req

def signal_callback(msg):
    """
    Callback function for the UR signal.
    
    This function is triggered every time a message is published on the '/ur_signal' topic.
    It checks whether the signal is high or low and takes action accordingly:
      - If the signal is high (True) and we're not already reconstructing, it starts reconstruction.
      - If the signal is low (False) while reconstructing, it stops reconstruction and exports the mesh.

    Parameters:
      msg (Bool): The incoming message containing the state of the UR signal.
    """
    global is_reconstructing

    # If signal is high and we haven't started reconstruction yet:
    if msg.data and not is_reconstructing:
        rospy.loginfo("Received HIGH signal: Starting reconstruction")
        
        # Set the end effector to the proper tool ("rgb_camera_tcp")
        ret = set_ee_srv("rgb_camera_tcp")
        if not ret:
            rospy.logwarn("Failed to set end effector")
            return

        # Any additional motion planning can be inserted here if required.
        # e.g., you can plan a motion to a specific pose before starting reconstruction.

        if CAPTURE:
            # Generate reconstruction request messages.
            start_req, _ = gen_recon_msg(SAVE_DIR)
            try:
                start_recon_srv(start_req)
                is_reconstructing = True  # Mark reconstruction as started.
                rospy.loginfo("Reconstruction started")
            except rospy.ServiceException as e:
                rospy.logerr("Failed to start reconstruction: %s", e)

    # If signal is low and reconstruction is active, stop and export the mesh.
    elif not msg.data and is_reconstructing:
        rospy.loginfo("Received LOW signal: Stopping reconstruction and exporting mesh")
        if CAPTURE:
            # Generate a new reconstruction stop request message.
            _, stop_req = gen_recon_msg(SAVE_DIR)
            try:
                stop_recon_srv(stop_req)
                is_reconstructing = False  # Mark reconstruction as stopped.
                rospy.loginfo("Reconstruction stopped, mesh saved at: %s", stop_req.mesh_filepath)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to stop reconstruction: %s", e)

def main():
    """
    Main function to initialize the ROS node and service proxies.
    
    This function:
      1. Initializes the node.
      2. Waits for and sets up necessary service proxies for setting the end effector,
         planning, executing trajectories, and reconstruction.
      3. Subscribes to the '/ur_signal' topic to monitor the UR signal state.
      4. Keeps the node running with rospy.spin().
    """
    global start_recon_srv, stop_recon_srv, set_ee_srv, plan_goal_srv, execute_trajectory_srv

    # Initialize the ROS node with a descriptive name.
    rospy.init_node("ur10e_reconstruction_controller")

    # Wait for the set end effector service to be available, then create its proxy.
    rospy.wait_for_service("/commander/set_ee", timeout=10)
    set_ee_srv = rospy.ServiceProxy("/commander/set_ee", SetEe)

    # Wait for the planning and trajectory execution services to become available.
    rospy.wait_for_service("/commander/plan_goal", timeout=10)
    plan_goal_srv = rospy.ServiceProxy("/commander/plan_goal", PlanGoal)

    rospy.wait_for_service("/commander/execute_trajectory", timeout=10)
    execute_trajectory_srv = rospy.ServiceProxy("/commander/execute_trajectory", ExecuteTrajectory)

    # If capturing reconstruction data, set up the required service proxies.
    if CAPTURE:
        rospy.wait_for_service("/start_reconstruction", timeout=10)
        start_recon_srv = rospy.ServiceProxy("/start_reconstruction", StartReconstruction)
        rospy.wait_for_service("/stop_reconstruction", timeout=10)
        stop_recon_srv = rospy.ServiceProxy("/stop_reconstruction", StopReconstruction)

    # Subscribe to the UR signal topic.
    # The node expects messages on "/ur_signal" as a Bool indicating the signal state.
    rospy.Subscriber("/ur_signal", Bool, signal_callback)

    rospy.loginfo("UR10e reconstruction controller node started. Listening for UR signal changes...")
    rospy.spin()  # Keep the node running until shutdown.

if __name__ == "__main__":
    main()

