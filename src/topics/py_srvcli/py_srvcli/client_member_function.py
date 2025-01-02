
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

import sys

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('serivce not avaliable, waiting')
        self.req = AddTwoInts.Request()
    
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    
def main():
    rclpy.init()
    
    minmal_cli = MinimalClientAsync()
    response = minmal_cli.send_request(10, 2)
    minmal_cli.get_logger().info(f'result of add_two_ints, 1 + 2 = {response.sum}')
    
    minmal_cli.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()