
import rclpy
from arena_api import enums
from arena_api.system import system
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from arena_camera_node._enum_translator import ROS2PixelFormat
from arena_camera_node.device_manager import DeviceCreationManager
from arena_camera_node.image_publisher import ImagePublisherHelper


class ArenaCameraNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # logs ---------------------------------------------
        self._log = self.get_logger()
        self._log_debug = self._log.debug
        self._log_error = self._log.error
        self._log_fatal = self._log.fatal
        self._log_info = self._log.info
        self._log_warn = self._log.warn

        # params  ------------------------------------------
        # device creation
        self.declare_parameter(name='serial', value=None)  # string or number

        # to be supported by
        # self.declare_parameter(name='ip', value=None)
        # self.declare_parameter(name='mac', value=Node)
        # self.declare_parameter(name='id', value=Node)
        self.declare_parameter(
            name='topic', value=f'/{self.get_name()}/images')

        # Nodes
        self.declare_parameter(name='gain', value=None)
        self.declare_parameter(name='width', value=None)
        self.declare_parameter(name='height', value=None)
        self.declare_parameter(name='pixelformat', value=None)
        self.declare_parameter(name='exposure_auto', value='true')
        self.declare_parameter(name='exposure_time', value=-1)
        self.declare_parameter(name='trigger_mode', value='false')

    def run(self):

        # device -------------------------------------------
        self._wait_until_a_device_is_discovered()
        self._create_device()

        # nodes --------------------------------------------
        self._set_nodes()

        # services -----------------------------------------

        self._srv = self.create_service(Trigger, 'trigger_image',
                                        self._publish_images_at_trigger)
        # streaming ----------------------------------------
        self._device.start_stream()

        # publish images -------------------------------
        if not self.trigger_mode_active:
            self._publish_images()

    def _wait_until_a_device_is_discovered(self):
        self._wait_for_devices_secs = system.DEVICE_INFOS_TIMEOUT_MILLISEC / 1000
        def check(self=self):

            # could have a bug as we are rereading the device_info_again
            device_infos = system.device_infos
            devices_discovered_count = len(device_infos)
            if not devices_discovered_count:
                self._log_info('No arena camera is connected')
            else:
                self.destroy_timer(self.timer)  # To stop the timer

        self.timer = self.create_timer(self._wait_for_devices_secs, check)

    def _create_device(self):
        # no device guard
        device_infos = system.device_infos

        # check param for device to create
        # by serial
        serial = self.get_parameters(['serial'])[0]  # just searial for now
        if serial.value:
            self._device = DeviceCreationManager.create_device_with_serial(
                str(serial.value), device_infos)
            # if not correct serial make the system waits for it
            # DeviceListener.wait_for_device(serial=serial)

        # first discovered
        else:
            # pass in only one device info so not all devices are created
            one_device_in_list = system.create_device(
                device_infos=device_infos[0])
            self._device = one_device_in_list[0]

        self._log_info(f'arena camera node is running for '
                       f'device {self._device}')

    def _set_nodes(self):

        # device run on default profile all the time if no args are passed
        # otherwise, overwise only these params
        self._device.nodemap['UserSetSelector'].value = 'Default'
        self._device.nodemap['UserSetLoad'].execute()

        gain = self.get_parameter('gain')
        if gain.value:
            self._device.nodemap['Gain'].value = gain.value

        width = self.get_parameter('width')
        if width.value:
            # TODO > max flat to max
            self._device.nodemap['Width'].value = width.value

        height = self.get_parameter('height')
        if height.value:
            self._device.nodemap['Height'].value = height.value

        pixelformat = self.get_parameter('pixelformat')
        if pixelformat.value:
            arena_api_pixelformat_enum_key = None
            for arena_api_pixelformat_name, ros_pixelformat_name in ROS2PixelFormat.items():
                if ros_pixelformat_name == pixelformat.value:
                    arena_api_pixelformat_enum_key = arena_api_pixelformat_name
                    break
            else:
                raise ValueError(f'PixelFormat {pixelformat.value} is not '
                                 f'supported')
            self._device.nodemap['PixelFormat'].value = enums.PixelFormat[arena_api_pixelformat_enum_key]

        exposure_auto = self.get_parameter('exposure_auto')
        exposure_time = self.get_parameter('exposure_time')
        if str(exposure_auto.value).lower() == 'true':
            self._device.nodemap['ExposureAuto'].value = 'Continuous'
            if exposure_time.value != -1:
                self._log_warn('exposure_time is ignored for exposure_auto')
        elif str(exposure_auto.value).lower() == 'false':
            self._device.nodemap['ExposureAuto'].value = 'Off'
            if exposure_time.value == -1:
                raise ValueError(
                    f'\"exposure_time\" is required when \"exposure_auto\"=\"false\"')

            else:
                exposure_time_float = float(exposure_time.value)
                self._device.nodemap['ExposureTime'].value = exposure_time_float
        else:
            raise ValueError(
                f'\"exposure_auto\" must be one of [true , false]')

        trigger_mode = self.get_parameter('trigger_mode')
        if str(trigger_mode.value).lower() == 'true':
            # Enable trigger mode before setting the source and selector
            # and before starting the stream. Trigger mode cannot be turned
            # on and off while the device is streaming.

            # Make sure Trigger Mode set to 'Off' after finishing this example
            self._device.nodemap['TriggerMode'].value = 'On'

            # Set the trigger source to software in order to trigger buffers
            # without the use of any additional hardware.
            # Lines of the GPIO can also be used to trigger.
            self._device.nodemap['TriggerSource'].value = 'Software'
            self._device.nodemap['TriggerSelector'].value = 'FrameStart'
            self._device.tl_stream_nodemap['StreamBufferHandlingMode'].value = 'OldestFirst'

            self.trigger_mode_active = True
            self._log_info(f'trigger_mode is activated. No images will be '
                           f'published until images are requested by '
                           f'trigger_image service')
        else:
            # device is not in trigger mode because the user default profile is
            # set
            self.trigger_mode_active = False
            pass

    def _publish_images(self):
        # when here the device is assumed to be already streaming

        # later this is the frame rate
        topic = self.get_parameter('topic').value
        self._image_publisher = self.create_publisher(
            msg_type=Image,
            topic=topic,
            qos_profile=HistoryPolicy.SYSTEM_DEFAULT)

        while rclpy.ok():

            if self.trigger_mode_active:
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

            except Exception as e:
                # clean up
                if self._buffer:
                    self._device.requeue_buffer(self._buffer)
                    self._buffer = None
                self._log_warn(f'Exception occurred when grabbing '
                               f'an image\n{e}')

    def _publish_images_at_trigger(self, request, response):
        # when here the device is assumed to be already streaming

        if not self.trigger_mode_active:
            self._log_warn(f'{self._device} is not in trigger mode. please run'
                           f' {self.get_name()} with trigger_mode:=true first')
            return response

        # later this is the frame rate
        topic = self.get_parameter('topic').value
        self._image_publisher = self.create_publisher(
            msg_type=Image,
            topic=topic,
            qos_profile=HistoryPolicy.SYSTEM_DEFAULT)

        while not self._device.nodemap['TriggerArmed'].value:
            continue

        self._device.nodemap['TriggerSoftware'].execute()

        try:
            self._buffer = self._device.get_buffer()
            self._image_msg = ImagePublisherHelper.msg_from_buffer(
                self._buffer)
            self._image_publisher.publish(self._image_msg)
            self._log_info(
                f'image{self._buffer.frame_id} published to {topic}')
            self._device.requeue_buffer(self._buffer)
            response.success = True
            response.message = f'image was published to {topic}'

        except Exception as e:
            # clean up
            if self._buffer:
                self._device.requeue_buffer(self._buffer)
                self._buffer = None
            self._log_warn(f'Exception occurred when grabbing '
                           f'an image\n{e}')

        self.destroy_publisher(self._image_publisher)

        return response


def main(args=None):

    rclpy.init(args=args)

    arena_camera_node = ArenaCameraNode(node_name='arena_camera_node')
    try:
        arena_camera_node.run()
        rclpy.spin(arena_camera_node)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        arena_camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
