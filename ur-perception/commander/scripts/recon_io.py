import rospy
from std_msgs.msg import Bool
from datetime import datetime
from geometry_msgs.msg import Vector3
from ur_msgs.msg import IOStates
from typing import Tuple

# Import reconstruction service definitions
from industrial_reconstruction_msgs.srv import (
    StartReconstruction, StartReconstructionRequest,
    StopReconstruction, StopReconstructionRequest
)

class ReconIO:
    
    def __init__(self):
        self.sub = rospy.Subscriber('/ur_hardware_interface/io_states', IOStates, self.io_cb)

        self.path = "/home/intellection/capture"
        self.io = None
        self.capture = None
        self.last_capture_state = False  # Track the previous state of capture

        rospy.wait_for_service("/start_reconstruction", timeout=10)
        self.start_recon_srv = rospy.ServiceProxy("/start_reconstruction", StartReconstruction)
        rospy.wait_for_service("/stop_reconstruction", timeout=10)
        self.stop_recon_srv = rospy.ServiceProxy("/stop_reconstruction", StopReconstruction)

    def run(self):
        while not rospy.is_shutdown():
            if not self.capture and not self.last_capture_state:
                rospy.loginfo("Waiting for capture signal...")
                
            # Trigger start reconstruction when capture is True
            if self.io and self.capture and not self.last_capture_state:
                rospy.loginfo("Triggering reconstruction start...")
                start_recon_req, _ = self.gen_recon_msg()
                self.start_recon_srv(start_recon_req)
                self.last_capture_state = True
                rospy.loginfo("Reconstruction started")
            
            # Trigger stop reconstruction only once when capture is False
            elif not self.capture and self.last_capture_state:
                rospy.loginfo("Triggering reconstruction stop...")
                _, stop_recon_req = self.gen_recon_msg()
                self.stop_recon_srv(stop_recon_req)
                self.last_capture_state = False
                rospy.loginfo("Reconstruction stopped")
                self.wait_for_recon_to_finish()
            
            # Add a small delay before the next check
            rospy.sleep(0.5)

    def wait_for_recon_to_finish(self):
        """ Block the process until the reconstruction completes. """
        rospy.loginfo("Waiting for reconstruction to complete...")
        # You can implement a more sophisticated check here, such as checking if capture is False.
        while self.capture:
            rospy.sleep(0.5)  # Check periodically if capture is False

    def gen_recon_msg(self) -> Tuple[StartReconstructionRequest, StopReconstructionRequest]:
        start_srv_req = StartReconstructionRequest()
        start_srv_req.tracking_frame = "rgb_camera_tcp"
        start_srv_req.relative_frame = "base_link"
        start_srv_req.translation_distance = 0.0
        start_srv_req.rotational_distance = 0.0
        start_srv_req.live = True
        start_srv_req.tsdf_params.voxel_length = 0.001
        start_srv_req.tsdf_params.sdf_trunc = 0.002
        start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
        start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
        start_srv_req.rgbd_params.depth_scale = 1
        start_srv_req.rgbd_params.depth_trunc = 1.0
        start_srv_req.rgbd_params.convert_rgb_to_intensity = False

        stop_srv_req = StopReconstructionRequest()
        path = self.path + datetime.now().strftime("%m_%d_%H_%M") + ".ply"
        stop_srv_req.mesh_filepath = path

        return start_srv_req, stop_srv_req

    def io_cb(self, data):
        if data is not None:
            self.io = data.digital_out_states[0].state
            self.capture = data.digital_out_states[1].state
        else:
            rospy.loginfo('No data received')


def main():
    rospy.init_node('recon_io', anonymous=True)
    recon_io = ReconIO()
    recon_io.run()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()