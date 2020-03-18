
import time
from arena_api import system


def run():
    count = 0
    while(not count or count < 10):
        count += 1
        print(f'arena_node {count} | seeing '
        '{len(system.device_info)} devices')

        time.sleep(1.5)


if __name__ == '__main__':
    run()
