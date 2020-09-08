#pragma once

// STD
#include <map>
#include <string>

// ROS2
#include <sensor_msgs/image_encodings.hpp>

// ArenaSDK
#include "ArenaApi.h"  // GetPixelFormatName() , PfncFormat::

static std::map<std::string, std::string> K_ROS2_PIXELFORMAT_TO_PFNC(
    {{sensor_msgs::image_encodings::RGB8, GetPixelFormatName(PfncFormat::RGB8)},
     {sensor_msgs::image_encodings::RGBA8,
      GetPixelFormatName(PfncFormat::RGBa8)},
     {sensor_msgs::image_encodings::RGB16,
      GetPixelFormatName(PfncFormat::RGB16)},
     {sensor_msgs::image_encodings::RGBA16,
      GetPixelFormatName(PfncFormat::RGBa16)},
     {sensor_msgs::image_encodings::BGR8, GetPixelFormatName(PfncFormat::BGR8)},
     {sensor_msgs::image_encodings::BGRA8,
      GetPixelFormatName(PfncFormat::BGRa8)},
     {sensor_msgs::image_encodings::BGR16,
      GetPixelFormatName(PfncFormat::BGR16)},
     {sensor_msgs::image_encodings::BGRA16,
      GetPixelFormatName(PfncFormat::BGRa16)},
     {sensor_msgs::image_encodings::MONO8,
      GetPixelFormatName(PfncFormat::Mono8)},
     {sensor_msgs::image_encodings::MONO16,
      GetPixelFormatName(PfncFormat::Mono16)},
     {sensor_msgs::image_encodings::BAYER_RGGB8,
      GetPixelFormatName(PfncFormat::BayerRG8)},
     {sensor_msgs::image_encodings::BAYER_BGGR8,
      GetPixelFormatName(PfncFormat::BayerBG8)},
     {sensor_msgs::image_encodings::BAYER_GBRG8,
      GetPixelFormatName(PfncFormat::BayerGB8)},
     {sensor_msgs::image_encodings::BAYER_GRBG8,
      GetPixelFormatName(PfncFormat::BayerGR8)},
     {sensor_msgs::image_encodings::BAYER_RGGB16,
      GetPixelFormatName(PfncFormat::BayerRG16)},
     {sensor_msgs::image_encodings::BAYER_BGGR16,
      GetPixelFormatName(PfncFormat::BayerBG16)},
     {sensor_msgs::image_encodings::BAYER_GBRG16,
      GetPixelFormatName(PfncFormat::BayerGB16)},
     {sensor_msgs::image_encodings::BAYER_GRBG16,
      GetPixelFormatName(PfncFormat::BayerGR16)},
     {sensor_msgs::image_encodings::YUV422,
      GetPixelFormatName(PfncFormat::YUV422_8)}});

static std::map<std::string, std::string> K_PFNC_TO_ROS2_PIXELFORMAT(
    {{GetPixelFormatName(PfncFormat::RGB8), sensor_msgs::image_encodings::RGB8},
     {GetPixelFormatName(PfncFormat::RGBa8),
      sensor_msgs::image_encodings::RGBA8},
     {GetPixelFormatName(PfncFormat::RGB16),
      sensor_msgs::image_encodings::RGB16},
     {GetPixelFormatName(PfncFormat::RGBa16),
      sensor_msgs::image_encodings::RGBA16},
     {GetPixelFormatName(PfncFormat::BGR8), sensor_msgs::image_encodings::BGR8},
     {GetPixelFormatName(PfncFormat::BGRa8),
      sensor_msgs::image_encodings::BGRA8},
     {GetPixelFormatName(PfncFormat::BGR16),
      sensor_msgs::image_encodings::BGR16},
     {GetPixelFormatName(PfncFormat::BGRa16),
      sensor_msgs::image_encodings::BGRA16},
     {GetPixelFormatName(PfncFormat::Mono8),
      sensor_msgs::image_encodings::MONO8},
     {GetPixelFormatName(PfncFormat::Mono16),
      sensor_msgs::image_encodings::MONO16},
     {GetPixelFormatName(PfncFormat::BayerRG8),
      sensor_msgs::image_encodings::BAYER_RGGB8},
     {GetPixelFormatName(PfncFormat::BayerBG8),
      sensor_msgs::image_encodings::BAYER_BGGR8},
     {GetPixelFormatName(PfncFormat::BayerGB8),
      sensor_msgs::image_encodings::BAYER_GBRG8},
     {GetPixelFormatName(PfncFormat::BayerGR8),
      sensor_msgs::image_encodings::BAYER_GRBG8},
     {GetPixelFormatName(PfncFormat::BayerRG16),
      sensor_msgs::image_encodings::BAYER_RGGB16},
     {GetPixelFormatName(PfncFormat::BayerBG16),
      sensor_msgs::image_encodings::BAYER_BGGR16},
     {GetPixelFormatName(PfncFormat::BayerGB16),
      sensor_msgs::image_encodings::BAYER_GBRG16},
     {GetPixelFormatName(PfncFormat::BayerGR16),
      sensor_msgs::image_encodings::BAYER_GRBG16},
     {GetPixelFormatName(PfncFormat::YUV422_8),
      sensor_msgs::image_encodings::YUV422}});
