
import rclpy
from rclpy.node import Node
from pulse_msgs.srv import Pulse as PulseMsg
import pulse
import warp as wp

from pulse.lib.warp_kernels import Pose

import pulse.lib.plotting as plot

from pulse.lib.pulse import Pulse
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
                        samples_per_pulse=request.pulse_params.samples_per_pulse)
        p.load_mesh(request.mesh_filepath)
        # positions = request.poses
        warp_poses = []

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
            warp_poses.append(warp_seg)

        # Dwell times
        dwell_times = request.dwell_times

        # Pass to evaluate_pulses
        # res = p.evaluate_pulses(warp_poses)
        interp_poses, _, _ = p.interpolate_poses_by_dwell(warp_poses, dwell_times)
        res = p.evaluate_pulses_with_dwells(warp_poses, dwell_times)
        res_scaled = p.get_scaled_thicknesses(res)
        print(res_scaled.shape)
        # response.texture.height = res_scaled.shape[0]
        # response.texture.width = res_scaled.shape[1]
        response.texture.data = (res_scaled.flatten()).astype('uint8').tolist()

        # plot.visualize_result(warp_poses, res, request.mesh_filepath)
        plot.visualize_result(viz_poses, res_scaled, request.mesh_filepath)
        plot.visualize_result(interp_poses, res_scaled, request.mesh_filepath)
        response.message = "Pulse operation completed successfully."

        response.success = True
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