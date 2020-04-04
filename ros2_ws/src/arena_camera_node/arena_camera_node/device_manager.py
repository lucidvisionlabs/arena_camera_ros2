from arena_api.system import system


class DeviceCreationManager:
    def __init__(self):
        pass

    @staticmethod
    def create_device_with_serial(serial, device_infos):
        for device_info in device_infos:
            if device_info['serial'] == serial:
                return system.create_device(device_info)[0]
        else:
            # to do should wait for it
            raise ValueError(
                f'device with serial "{serial}" is not connected')


'''
class DeviceListener:
    def __init__(self):
        pass

    @staticmethod
    def wait_for_device_with_serial(serial, device_infos):

        wait_for_devices_secs = system.DEVICE_INFOS_TIMEOUT_MILLISEC / 1000
        timer = None
        def check(timer):

            # could have a bug as we are rereading the device_info_again
            device_infos = system.device_infos
            devices_discovered_count = len(device_infos)
            if not devices_discovered_count:
                self._log_info('No arena camera is connected')
            else:
                self.destroy_timer(timer)  # To stop the timer

        timer = self.create_timer(wait_for_devices_secs, check, timer)
'''
