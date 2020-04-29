#include <memory>
#include <vector>
#include "ArenaApi.h"

class IDeviceHelpers
{
 public:
  static size_t get_index_of_serial(std::vector<Arena::DeviceInfo> device_infos,
                                    std::string serial);
};

class PixelFormatTranslator
{
 public:
  std::string to_ros(std::string pixelformat);
  // std::string to_arena(std::string pixelformat);
}