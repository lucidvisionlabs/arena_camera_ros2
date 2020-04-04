from arena_api.enums import PixelFormat

'''
src:
    https://github.com/ros2/common_interfaces/blob \
    master/sensor_msgs/include/sensor_msgs/image_encodings.hpp

    namespace image_encodings block
'''
ROS2PixelFormat = {
    PixelFormat.RGB8.name: "rgb8",
    PixelFormat.RGBa8.name: "rgba8",
    PixelFormat.RGB16.name: "rgb16",
    PixelFormat.RGBa16.name: "rgba16",
    PixelFormat.BGR8.name: "bgr8",
    PixelFormat.BGRa8.name: "bgra8",
    PixelFormat.BGR16.name: "bgr16",
    PixelFormat.BGR16.name: "bgra16",
    PixelFormat.Mono8.name: "mono8",
    PixelFormat.Mono16.name: "mono16",

    # OpenCV CvMat types
    #

    #
    # Bayer encodings
    #
    # PixelFormat.BayerRGB8: "bayer_rggb8",      # ------------------>
    # PixelFormat.BayerBGr8: "bayer_bggr8",      # ------------------>
    # PixelFormat.BayerGBR8: "bayer_gbrg8",      # ------------------>
    # PixelFormat.BayerGRB8: "bayer_grbg8",      # ------------------>
    # PixelFormat.BayerRGB16: "bayer_rggb16",    # ------------------>
    # PixelFormat.BayerBGR16: "bayer_bggr16",    # ------------------>
    # PixelFormat.BayerGBR16: "bayer_gbrg16",    # ------------------>
    # PixelFormat.BayerGRBG16: "bayer_grbg16",   # ------------------>

    # Miscellaneous
    PixelFormat.YUV422_8.name: "yuv422"
    # PixelFormat.YUV422_8_UYVY: "yuv422_yuy2",  # ------------------>
}

