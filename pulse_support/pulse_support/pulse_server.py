#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from pulse_msgs.srv import Pulse as PulseMsg
from pulse_msgs.msg import Thickness
import pulse
import warp as wp

from pulse.lib.warp_kernels import Pose

import pulse.lib.plotting as plot

from pulse.lib.pulse import Pulse

import numpy as np

class PulseServer(Node):
    def __init__(self):
        super().__init__('pulse_server')
        self.srv = self.create_service(
            PulseMsg,
            'pulse',
            self.pulse_callback
        )
        self.get_logger().info('Pulse service ready on "pulse"')

    def pulse_callback(self, request: PulseMsg.Request, response: PulseMsg.Response) -> PulseMsg.Response:
        self.get_logger().info(f"Received pulse request")

        p = Pulse(sigma_u=request.pulse_params.sigma_u,
                        sigma_v=request.pulse_params.sigma_v,
                        a=request.pulse_params.a,
                        ref_dist=request.pulse_params.ref_distance,
                        samples_per_pulse=request.pulse_params.samples_per_pulse,
                        volumetric_flow_rate=request.pulse_params.volumetric_flow_rate)
        p.load_mesh(request.mesh_filepath)
        # positions = request.poses
        warp_path_segments = []

        viz_poses = []

        for seg in request.paths:
            warp_seg = []
            for ros_pose in seg.poses:
                pose = Pose()
                pose.position = wp.vec3(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z)
                pose.rotation = wp.quat(ros_pose.orientation.x,
                                       ros_pose.orientation.y,
                                   ros_pose.orientation.z,
                                   ros_pose.orientation.w)
                warp_seg.append(pose)
                viz_poses.append(pose)
            warp_path_segments.append(warp_seg)

        # Dwell times
        dwell_times = request.dwell_times

        self.get_logger().info(f"Evaluating {len(warp_path_segments)} path segments with a total of {len(viz_poses)} poses.")
        self.get_logger().info(f"Dwell times provided: {len(dwell_times)}")

        # get current time
        from datetime import datetime
        current_datetime = datetime.now()
        thicknesses_per_pulse = []
        idx = 0
        for seg in warp_path_segments:
            for pose_idx in range(0, len(seg) - 1):
                pose_pair = [seg[pose_idx], seg[pose_idx + 1]]
                res = p.evaluate_pulses_with_dwells([pose_pair], [dwell_times[idx]])
                thicknesses_per_pulse.append(p.get_scaled_thicknesses(res))
                idx += 1

        total_thicknesses = np.sum(np.array(thicknesses_per_pulse), axis=0)
        end_time = datetime.now()

        total_time = end_time - current_datetime
        self.get_logger().info(f"Pulse evaluation completed in {total_time.total_seconds():.2f} seconds.")
        if request.display_results:
            interp_poses, _, _ = p.interpolate_poses_by_dwell(warp_path_segments, dwell_times)
            plot.visualize_result(viz_poses, total_thicknesses, request.mesh_filepath)
            plot.visualize_result(interp_poses, total_thicknesses, request.mesh_filepath)
            plot.display_within_bounds(total_thicknesses, request.min_thickness, request.max_thickness, request.mesh_filepath)
        response.message = "Pulse operation completed successfully."

        response.success = True

        response.thicknesses.data = total_thicknesses.tolist()
        for tp in thicknesses_per_pulse:
            thickness_msg = Thickness()
            thickness_msg.data = tp.tolist()
            response.thicknesses_per_pulse.append(thickness_msg)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = PulseServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down pulse_server')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()