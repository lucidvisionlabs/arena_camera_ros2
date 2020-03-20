
import rclpy
from arena_api.system import system
from rclpy.node import Node
from sensor_msgs.msg import Image
from arena_api.buffer import BufferFactory
from arena_api import enums


class ArenaCameraNode(Node):
    # publisher pattern
    def __init__(self):
        node_name = 'arena_camera_node'
        super().__init__(node_name)

        self._log = self.get_logger()
        self._log_info = self._log.info

        self._wait_for_devices_secs = system.DEVICE_INFOS_TIMEOUT_MILLISEC / 1000

        self._wait_until_a_device_is_discovered()

        # get first device
        devices = system.create_device()

        self._device = devices[0]
        self._log_info(
            f'arena camera node is running for device {self._device}')

        # debug settings
        self._device.nodemap['TimestampReset'].execute()
        #self._device.nodemap['Width'].value = self._device.nodemap['Width'].min
        #self._device.nodemap['Width'].value = self._device.nodemap['Width'].max
        self._device.nodemap['Width'].value = 1000
        #self._device.nodemap['Height'].value = self._device.nodemap['Height'].min
        #self._device.nodemap['Height'].value = self._device.nodemap['Height'].max
        self._device.nodemap['Height'].value = 1000
        self._device.nodemap['PixelFormat'].value = enums.PixelFormat.Mono8

        # publish Images
        # later this is the frame rate
        self._image_publisher = self.create_publisher(
            Image, f'{node_name}/images')
        # Image, 'arena_camera_node_images')
        self._publish_images()
        self._log_info('Done')

    def _publish_images(self):

        with self._device.start_stream():
            count = 30000
            while count:
                count -= 1
                try:
                    self._buffer = self._device.get_buffer()
                    self._buffer_copied = BufferFactory.copy(self._buffer)

                    # publishing
                    self._image_msg = Image()
                    self._image_msg.data = self._buffer_copied.data
                    self._image_msg.width = self._buffer_copied.width
                    self._image_msg.height = self._buffer_copied.height

                    # TODO create a transltor for pixelformats from pfnc names to ros2 std names
                    # https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/image_encodings.hpp

                    #self._image_msg.encoding = self._buffer_copied.pixel_format.name
                    self._image_msg.encoding = 'mono8'

                    # https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Time.msg
                    self._image_msg.header.stamp.nanosec = self._buffer_copied.timestamp_ns % 1000000000
                    self._image_msg.header.stamp.sec = int(
                        self._buffer_copied.timestamp_ns / 1000000000)

                    self._image_msg.header.frame_id = str(
                        self._buffer_copied.frame_id)
                    self._image_msg.is_bigendian = self._buffer_copied.pixel_endianness == enums.PixelEndianness.BIG

                    self._image_publisher.publish(self._image_msg)
                    # print(self._image_msg)

                    self._device.requeue_buffer(self._buffer)
                    BufferFactory.destroy(self._buffer_copied)

                    self._log_info(f'Buffer {count}received and requeued')

                except Exception as e:
                    print(f'EXCEPTION {count}\n {e}')
                    print(self._buffer_copied.timestamp_ns)
                    # clean up
                    if self._buffer:
                        self._device.requeue_buffer(self._buffer)
                        self._buffer = None

                    if self._buffer_copied:
                        BufferFactory.destroy(self._buffer_copied)
                        self._buffer_copied = None

        self._log_info('Done grabbing')

    def _wait_until_a_device_is_discovered(self):

        def check(self=self):

            # could have a bug as we are rereading the device_info_again
            device_infos = system.device_infos
            devices_discovered_count = len(device_infos)
            if not devices_discovered_count:
                self._log_info('No arena camera is connected')
            else:
                self.destroy_timer(self.timer)  # To stop the timer

        self.timer = self.create_timer(self._wait_for_devices_secs, check)


def run(args=None):
    rclpy.init(args=args)

    arena_camera_node = ArenaCameraNode()
    rclpy.spin_once(arena_camera_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arena_camera_node.destroy_node()
    print('node destroied')
    rclpy.shutdown()
    print('rclpy shutdoen')


if __name__ == '__main__':
    run()


'''
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_publisher')
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()
    i = 0

    def timer_callback():
        nonlocal i
        msg.data = 'Hello World: %d' % i
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)

    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

'''
