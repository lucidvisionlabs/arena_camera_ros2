import rclpy
#from arena_camera_software_trigger.srv import TriggerImage
from rclpy.node import Node
from std_srvs.srv import Trigger


class TriggerClientNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        log_debug = self.get_logger().debug
        log_info = self.get_logger().info

        self.cli = self.create_client(Trigger, 'trigger_image')

        while not self.cli.wait_for_service(timeout_sec=0.5):
            log_info(f'service not available, waiting again... ->')
        log_debug('service is available now')
        self.req = Trigger.Request()
        log_debug('trigger request object has been created')

    def send_request(self):
        self.future = self.cli.call_async(self.req)


def main(args=None):

    log_debug = None
    log_info = None

    rclpy.init(args=args)

    trigger_client = TriggerClientNode(node_name='trigger_image_client_node')
    log_debug = trigger_client.get_logger().debug
    log_info = trigger_client.get_logger().info
    log_debug('trigger_image_client_node has been created')

    trigger_client.send_request()
    log_info('trigger_image_client_node sent a request')

    while rclpy.ok():
        log_debug('trigger_image_client is waiting for a response')

        rclpy.spin_once(trigger_client)
        log_debug('trigger_image_client spinned once')

        log_debug('checking if responce is received?')
        if trigger_client.future.done():
            log_debug('responce is received')
            try:
                response = trigger_client.future.result()
                log_debug('response is read')
                if not response.success:
                    log_info('response.success == False')
                    raise Exception(f'Failed to publish an image on trigger')
            except Exception as e:
                log_info('Service call failed %r' % (e,))
            else:
                log_info(response.message)
            log_debug('breaking the loop')
            break

    log_debug('trigger_image_client is destroyed')
    trigger_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
