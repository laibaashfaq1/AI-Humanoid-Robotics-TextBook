import rclpy
from rclpy.node import Node
from my_service_package.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        node = rclpy.create_node('add_two_ints_client_error')
        node.get_logger().info('Usage: ros2 run my_service_package add_two_ints_client <int> <int>')
        rclpy.shutdown()
        sys.exit(1)

    client = AddTwoIntsClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    if response:
        client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    else:
        client.get_logger().error('Service call failed.')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
