
'''

from rclpy.node import Node
from arena_camera_trigger import TriggerImage
import rclpy


class TriggerClientNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cli = self.create_client(TriggerImage, 'trigger_image_hi')

        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('service not available, waiting again...')
            self.req = TriggerImage.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)


def trigger_client_runner(args=None):

    rclpy.init(args=args)

    trigger_client = TriggerClientNode(node_name='trigger_image_node',
                                       namespace='arena_camera_node')
    trigger_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(trigger_client)
        if trigger_client.future.done():
            try:
                response = trigger_client.future.result()
                if not response.publish:
                    raise Exception(f'Image pulisher failed to publish Image '
                                    f'on trigger')
            except Exception as e:
                trigger_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                trigger_client.get_logger().info(f'Image has been published '
                                                 f'to {response.topic}')
            break

    trigger_client.destroy_node()
    print('node \"trigger_client\" destroyed')
    # rclpy.shutdown()


if __name__ == '__main__':
    trigger_client_runner()
'''
