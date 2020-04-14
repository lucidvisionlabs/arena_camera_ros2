class NodeManager:
    def __init__(device, params):
        self._device = device
        self._pararms = params

    def set_nodes_from_params(self):
        self._set_device_to_default_user_profile()
        self._set_gain_from_param()
        self._set_width_from_param()
        self._set_height_from_param()

        self._set_pixelformat_from_param()
        self._set_exposure_auto_and_exposure_time_from_param()
        self._set_trigger_mode_from_param()

    def _set_device_to_default_user_profile(self):

        # device run on default profile all the time if no args are passed
        # otherwise, overwise only these params
        self._device.nodemap['UserSetSelector'].value = 'Default'
        self._device.nodemap['UserSetLoad'].execute()

    def _set_gain_from_param(self):
        #gain = self.get_parameter('gain')
        # if gain.value:
        #    self._device.nodemap['Gain'].value = gain.value
        pass

    def _set_width_from_param(self):
        # width = self.get_parameter('width')
        # if width.value:
        #    # TODO > max flat to max
        #    self._device.nodemap['Width'].value = width.value
        pass

    def _set_height_from_param(self):
        #height = self.get_parameter('height')
        # if height.value:
        #    self._device.nodemap['Height'].value = height.value
        pass

    def _set_pixelformat_from_param(self):

        # pixelformat = self.get_parameter('pixelformat')
        # if pixelformat.value:
        #    arena_api_pixelformat_enum_key = None
        #    for arena_api_pixelformat_name, ros_pixelformat_name in ROS2PixelFormat.items():
        #        if ros_pixelformat_name == pixelformat.value:
        #           arena_api_pixelformat_enum_key = arena_api_pixelformat_name
        #            break
        #    else:
        #        raise ValueError(f'PixelFormat {pixelformat.value} is not '
        #                        f'supported')
        #   self._device.nodemap['PixelFormat'].value = enums.PixelFormat[arena_api_pixelformat_enum_key]
        pass

    def _set_exposure_auto_and_exposure_time_from_param(self):
        #exposure_auto = self.get_parameter('exposure_auto')
        #exposure_time = self.get_parameter('exposure_time')
        # if str(exposure_auto.value).lower() == 'true':
        #    self._device.nodemap['ExposureAuto'].value = 'Continuous'
        #    if exposure_time.value != -1:
        #        self._log_warn('exposure_time is ignored for exposure_auto')
        # elif str(exposure_auto.value).lower() == 'false':
        #    self._device.nodemap['ExposureAuto'].value = 'Off'
        #    if exposure_time.value == -1:
        #        raise ValueError(
        #            f'\"exposure_time\" is required when \"exposure_auto\"=\"false\"')
        #
        #    else:
        #        exposure_time_float = float(exposure_time.value)
        #        self._device.nodemap['ExposureTime'].value = exposure_time_float
        # else:
        #    raise ValueError(
        #        f'\"exposure_auto\" must be one of [true , false]')
        pass

    def _set_trigger_mode_from_param(self):
        #trigger_mode = self.get_parameter('trigger_mode')
        # if str(trigger_mode.value).lower() == 'true':
        #    # Enable trigger mode before setting the source and selector
        #    # and before starting the stream. Trigger mode cannot be turned
        #    # on and off while the device is streaming.

        #    # Make sure Trigger Mode set to 'Off' after finishing this example
        #    self._device.nodemap['TriggerMode'].value = 'On'

        #    # Set the trigger source to software in order to trigger buffers
        #    # without the use of any additional hardware.
        #    # Lines of the GPIO can also be used to trigger.
        #    self._device.nodemap['TriggerSource'].value = 'Software'
        #    self._device.nodemap['TriggerSelector'].value = 'FrameStart'
        #    self._device.tl_stream_nodemap['StreamBufferHandlingMode'].value = 'OldestFirst'

        #    self.trigger_mode_active = True
        #    self._log_info(f'trigger_mode is activated. No images will be '
        #                   f'published until images are requested by '
        #                   f'trigger_image service')
        # else:
        #    # device is not in trigger mode because the user default profile is
        #    # set
        #    self.trigger_mode_active = False
        #    pass
        pass
