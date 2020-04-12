
import rclpy
from arena_camera_software_trigger.srv import TriggerImage
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import HistoryPolicy
from arena_camera_node.image_publisher import ImagePublisherHelper


class TriggerServiceNode(Node):
    def __init__(self,
                 device,
                 is_trigger_mode_active,
                 *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._device = device
        self._is_trigger_mode_active = is_trigger_mode_active

        self._log_info = self.get_logger().info
        self._log_warn = self.get_logger().warn

        self.srv = self.create_service(TriggerImage,
                                       'trigger_image',
                                       self._publish_images_at_trigger)
        self.get_logger().info('TriggerServiceNode.__init__ --> done')

    def _x(self, request, response):
        response.published = True
        response.topic = 'x topic'
        self.get_logger().info('TriggerServiceNode._x --> done')
        return response

    def _publish_images_at_trigger(self, request, response):
        # when here the device is assumed to be already streaming
        if not self._is_trigger_mode_active:
            self._log_warn(f'{self._device} is not in trigger mode. '
                           f'Please run the node with '
                           f'\"--ros-args -p trigger_mode:=true\"')
            response.published = False
            response.topic = ''
            return response
        # later this is the frame rate
        topic = f"{self.get_name()}/{self.get_parameter('topic').value}"
        self._image_publisher = self.create_publisher(
            msg_type=Image,
            topic=topic,
            qos_profile=HistoryPolicy.SYSTEM_DEFAULT)

        is_armed = self._device.nodemap['TriggerArmed']
        while not is_armed.value:
            continue
        try:
            self._buffer = self._device.get_buffer()
            self._image_msg = ImagePublisherHelper.msg_from_buffer(
                self._buffer)
            self._image_publisher.publish(self._image_msg)
            self._log_info(
                f'image{self._buffer.frame_id} published to {topic}')
            self._device.requeue_buffer(self._buffer)
            response.published = True
            response.topic = topic

        except Exception as e:
            # clean up
            if self._buffer:
                self._device.requeue_buffer(self._buffer)
                self._buffer = None
            self._log_warn(f'Exception occurred when grabbing '
                           f'an image\n{e}')
            response.published = False
            response.topic = ''
        self.destroy_publisher(self._image_publisher)

        return response


def trigger_image_service_fn(args=None, device=None, is_trigger_mode_active=False):

    # rclpy.init(args=args)

    trigger_server = TriggerServiceNode(device=device,
                                        is_trigger_mode_active=is_trigger_mode_active,
                                        node_name='trigger_image_service_node',
                                        # namespace='arena_camera_node'
                                        )

    trigger_server.get_logger().info('trigger_image_service_fn --> started')

    rclpy.spin(trigger_server)

    print('node \"trigger_image_service_node\" destroyed')
    rclpy.shutdown()


if __name__ == '__main__':
    trigger_image_service_fn()
