
import time
from arena_api import sys
# /home/abdul/ software/arena_api
# /home/abdul/ arena_camera_ros2/ros2_ws/build/arena_camera_node


def run():
    help("modules")
    count = 0
    while(True):
        count += 1
        print(f'arena_node {count} | seeing {len(system.device_info)} devices')

        time.sleep(1.5)


if __name__ == '__main__':
    run()
