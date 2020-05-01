# -----------------------------------------------------------------------------
# Copyright (c) 2020, Lucid Vision Labs, Inc.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------

from pprint import pprint

from arena_api.system import system
import time



def example_entry_point():

    # Discover devices --------------------------------------------------------

    print('Discover devices on network')
    device_infos = system.device_infos
    print(f'{len(device_infos)} devices found')

    total_connect = len(device_infos)

    if not device_infos:
        raise BaseException('No device is found!')
    devices = system.create_device()
    for device in devices:
        device.nodemap['DeviceReset'].execute()

    while len(system.device_infos) < total_connect:
        time.sleep(1)
        print("waiting for devices to be up")
    print("devices have restarted successfully")


if __name__ == '__main__':

    print('WARNING:\nTHIS EXAMPLE MIGHT CHANGE THE DEVICE(S) SETTINGS!')
    print('Example started')
    example_entry_point()
    print('Example finished successfully')
