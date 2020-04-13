from sensor_msgs.msg import Image
from arena_camera_node._enum_translator import ROS2PixelFormat
from arena_api import enums


class ImagePublisherHelper:
    def __init__(self):
        pass

    @staticmethod
    def msg_from_buffer(buffer):
        '''
        construct Image ROS SENSOR MSG from arena_api buffer

        #todo fill more of the image info
        '''

        image_msg = Image()
        image_msg.data = buffer.data  # todo could be optimized
        image_msg.width = buffer.width
        image_msg.height = buffer.height

        # TODO create a transltor for pixelformats from pfnc names to ros2 std names
        # https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/include/sensor_msgs/image_encodings.hpp
        try:
            image_msg.encoding = ROS2PixelFormat[buffer.pixel_format.name]
        except KeyError:
            raise ValueError('PixelFormat value is not supported')

        # https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Time.msg
        image_msg.header.stamp.nanosec = buffer.timestamp_ns % 1000000000
        image_msg.header.stamp.sec = int(
            buffer.timestamp_ns / 1000000000)

        image_msg.header.frame_id = str(buffer.frame_id)
        image_msg.is_bigendian = buffer.pixel_endianness == enums.PixelEndianness.BIG

        return image_msg
