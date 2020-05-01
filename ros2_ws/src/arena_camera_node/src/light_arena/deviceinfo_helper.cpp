#include "deviceinfo_helper.h"
#include <algorithm>
#include <stdexcept>

#include <iostream>

size_t DeviceInfoHelper::get_index_of_serial(
    std::vector<Arena::DeviceInfo> device_infos, std::string serial)
{
  auto itr = std::find_if(
      device_infos.begin(),
      device_infos.end(), [=](Arena::DeviceInfo & itr_) -> auto {
        auto curr_serial = std::string(itr_.SerialNumber().c_str());
        // std::cout << curr_serial << '\n';
        return serial == curr_serial;
      });

  if (itr == device_infos.end()) {
    throw std::invalid_argument(std::string("No device with serial number ") +
                                serial);
  }
  auto index = std::distance(device_infos.begin(), itr);
  return index;
}
std::string DeviceInfoHelper::info(Arena::DeviceInfo device_info)
{
  auto msg = std::string(device_info.MacAddressStr().c_str()) + " | " +
             device_info.SerialNumber().c_str() + " | " +
             device_info.ModelName().c_str() + " | " +
             device_info.IpAddressStr().c_str() + " | " +
             device_info.UserDefinedName().c_str() + " |";
  return msg;
}