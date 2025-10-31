#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pulse_msgs.srv import Pulse
import pulse

class PulseServer(Node):
    def __init__(self):
        super().__init__('pulse_server')
        self.srv = self.create_service(
            Pulse,
            'pulse',
            self.pulse_callback
        )
        self.get_logger().info('Pulse service ready on "pulse"')

    def pulse_callback(self, request: Pulse.Request, response: Pulse.Response) -> Pulse.Response:
        self.get_logger().info(f"Received pulse request: {request}")

        params = pulse.PulseParams(
            sigma_u = request.pulse_params.sigma_u,
            sigma_v = request.pulse_params.sigma_v,
            a = request.pulse_params.a,
            ref_dist = request.pulse_params.ref_dist,
            samples_per_pulse = request.pulse_params.samples_per_pulse
        )

        p = pulse.Pulse(params)
        p.load_mesh(request.mesh_filepath)
        res = p.evaluate_poses(request.poses)
        res_scaled = p.get_scaled_thicknesses(res)
        
        response.texture.height = res_scaled.shape[0]
        response.texture.width = res_scaled.shape[1]
        response.texture.data = (res_scaled.flatten() * 255).astype('uint8').tolist()
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