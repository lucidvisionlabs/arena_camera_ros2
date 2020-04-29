#include "LightArena.h"
#include <algorithm>
#include <stdexcept>

#include <iostream>

size_t IDeviceHelpers::get_index_of_serial(
    std::vector<Arena::DeviceInfo> device_infos, std::string serial)
{
  auto itr = std::find_if(
      device_infos.begin(),
      device_infos.end(), [=](Arena::DeviceInfo & itrr) -> auto {
        auto curr_serial = std::string(itrr.SerialNumber().c_str());
        std::cout << curr_serial << '\n';
        return serial == curr_serial;
      });

  if (itr == device_infos.end()) {
    throw std::invalid_argument(std::string("No device with serial number ") +
                                serial);
  }
  auto index = std::distance(device_infos.begin(), itr);
  return index;
}