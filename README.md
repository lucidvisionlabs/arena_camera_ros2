# arena_camera_ros2
Arena Camera deriver for ROS2

# Note
- This ROS2 package is still in beta. Please provide your feedback is welcomed at support@thinklucid.com or the repo issue page
  
# Requirements
- 1 
  - OS       : Linux (x64/amd64) (>=18.04) 
  - ROS2     : Eloquent distro (installation steps in ros2_arena_setup.sh)
  - ArenaSDK : Contact support@thinklucid.com
  - arena_api : Contact support@thinklucid.com
- or 2 Docker
  - ArenaSDK : Contact support@thinklucid.com
  - arena_api : Contact support@thinklucid.com
  - build the docker image provided
  
# Getting Started
- clone repo
    
    `git clone https://github.com/lucidvisionlabs/arena_camera_ros2.git`

- install ROS2 and setup the environment 
    
    `cat arena_camera_ros2\ros2_arena_setup.sh` to view the script

    `cd arena_camera_ros2 ; sudo sh ros2_arena_setup.sh` installs ROS2 Eloquent distro

- install ArenaSDK and arena_api
  - contact support@thinklucid.com

- build workspace and its dependencies

    `cd arena_camera_ros2\ros2_ws`

    `rosdep install --from-paths src --ignore-src -r -y`

    `colcon build --symlink-install # build workspace for dev`

# Explore
- explore nodes
    - arena_camera_node
      - This is the main node for each device created. It represent a Lucid Camera.
      - it has two executable `start` and `trigger_image`
      - ros arguments
        - serial 
          - a string representing the serial of the device to create.
          - if not provided the node, it will represent the first dicovered camera.
        - topic
          - the topic the camera publish images on.
          - default value is /arena_camera_node/images.
          - if passed as a ros argument, it should be preceded with "/"
        - width
          - the width of desired image
          - default value is the one in `default` user profile.
        - height
          - the height of desired image
          - default value is the one in `default` user profile.
        - pixelformat
          - the pixel format of the deisred image
          - supported pixelformats are "rgb8", "rgba8", "rgb16", "rgba16", "bgr8", "bgra8", "bgr16", "bgra16",
                                       "mono8", "mono16", "bayer_rggb8", "bayer_bggr8", "bayer_gbrg8",
                                       "bayer_grbg8", "bayer_rggb16", "bayer_bggr16", "bayer_gbrg16", "bayer_grbg16", 
                                       "yuv422"
        - gain
          - a double value represents the gain of the image.

        - exposure_time
          - the time elapsed before the camera sensor creates the image.
          - units is micro seconds.
          - big values might makes the image take too long before it is view/published.
          - if trigger_mode is passed to node them it is recommended to set exposure_time as well so the
            triggered images do not take longer than expected.

        - trigger_mode
          - puts the device in ready state where it will wait for a `trigger_image` client to request an image.
          - default value is false. It means the device will be publishing images to the
            default topic `/arena_camera_node/images`.
          - values are true and false.
          - when false, images can be viewed 
  
            `ros2 run arena_camera_node start`
            `ros2 run image_tools showimage -t /arena_camera_node/images`
          
          - when true, image would not be published unless requested/triggered
  
            `ros2 run arena_camera_node start --ros-args -p exposure_time:=<proper value> -p trigger_mode:=true`
            `ros2 run image_tools showimage -t /arena_camera_node/images # no image will be displayed yet`
            `ros2 run arena_camera_node trigger_image`
       - examples for using all arguments
            
            `ros2 run arena_camera_node start --ros-args -p serial:="904240001" -p topic:=/special_images -p width:=100 -p height:=200 -p pixelformat:=rgb8 -p gain:=10 -p exposure_time:=150 -p trigger_mode:=true` 

    explore excutables

        ros2 pkg executables | grep arena
    
    all excutables can be run by using 
        
        `ros2 run <pakg name> <executable name>`

    explore actions
    - None

    explore services 
    - trigger_image 
      - trigger image form a device that is running in trigger_mode.
  
        - To run a device in trigger mode
    
            `ros2 run arena_camera_node start--ros-args -p exposure_time:=<proper value> -p trigger_mode=true`.
        
        - To trigger an image run 
            
            `ros2 run arena_camera_node trigger_image`

# Road map
- launch file
- send raw images
- camera_info
- access to nodemaps
- settings dump/read to/from file
- support two devices
- support windows
- support ARM64 and ARMhf
- launch file