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