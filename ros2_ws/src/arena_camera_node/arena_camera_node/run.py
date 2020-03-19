
import rclpy
from arena_api.system import system
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class ArenaCameraNode(Node):
    # publisher pattern
    def __init__(self):
        super().__init__('arena_camera_node')


        # get first device 
        self.device_ = system.create_device()
        
        while not len(system.device_infos):
            self.get_logger().info('No arena camera is connected')
            

        
        self.publisher_ = self.create_publisher(
            String, 'arena_camera_node', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: seeing # of cams {len(system.device_infos)}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def run(args=None):
    rclpy.init(args=args)

    arena_camera_node = ArenaCameraNode()
    rclpy.spin(arena_camera_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arena_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run()
