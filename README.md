# arena_camera_ros2
Arena Camera deriver for ROS2

# Note
- Please provide your feedback is welcomed at support@thinklucid.com or the repo issue page
  
# Requirements
- 1 
  - OS       : Linux (x64/amd64/arm64) (==18.04) 
  - ROS2     : Eloquent distro (installation steps in ros2_arena_setup.sh)
  - ArenaSDK and arena_api : https://thinklucid.com/downloads-hub/
- or 2 Docker
  - ArenaSDK and arena_api : https://thinklucid.com/downloads-hub/
  - build the docker image provided
  
# Getting Started
- clone repo or download release
    
    `git clone https://github.com/lucidvisionlabs/arena_camera_ros2.git`

- install ArenaSDK and arena_api
  - https://thinklucid.com/downloads-hub/

- install ROS2 and setup the environment 
    
    `cat arena_camera_ros2/ros2_arena_setup.sh` to view the script

    `cd arena_camera_ros2 ; sudo sh ros2_arena_setup.sh` installs ROS2 Eloquent distro

- build workspace and its dependencies

    `source ~/.bashrc`if using a regular terminal

    `cd arena_camera_ros2/ros2_ws`

    `rosdep install --from-paths src --ignore-src -r -y`

    `colcon build --symlink-install # build workspace for dev`

- install the build

    `. install/setup.bash`

# Explore
- explore nodes
    - arena_camera_node
      - this is the main node. It represent one LUCID Camera.
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
          - when `false`, images can be viewed

            `ros2 run arena_camera_node start --ros-args -p qos_reliability:=reliable -p topic:=image`

            `ros2 run image_tools showimage`

          - when `true`, image would not be published unless requested/triggered

            `ros2 run arena_camera_node start --ros-args -p qos_reliability:=reliable -p topic:=image -p exposure_time:=<proper value> -p trigger_mode:=true`

            `ros2 run image_tools showimage # no image will be displayed yet`

            `ros2 run arena_camera_node trigger_image`
       
      - QoS related parameters
        - if using these images with some subscriber make sure: 
          - both `arena_camera_node` and the subscriber on the same topic.
          - both have the same `QoS` settings else the images will be published but the subscriber would not see them because the image mags have a different `QoS` than the subscriber.
          - `QoS` parameter
          - qos_history
            - represents the history value of `QoS` for the image publisher.
            - default value is `keep_last`. 
            - supported values are "system_default","keep_last", "keep_all", "unknown".
            - more about `QoS`: https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/
          
          - qos_history_depth
            - represents the depth value of `QoS` for the image publisher.
            - default value is `5`.
            - more about `QoS`: https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/
          
          - qos_reliability
            - represents the reliability value of `QoS` for the image publisher.
            - default value is `best_effort`
            - supported values are "system_default", "reliable", "best_effort", "unknown".
            - more about `QoS`: https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/

        # simple example for using arguments together

          `ros2 run arena_camera_node start --ros-args -p serial:="904240001" -p topic:=/special_images -p width:=100 -p height:=200 -p pixelformat:=rgb8 -p gain:=10 -p exposure_time:=150 -p trigger_mode:=true` 

- explore excutables

  - `ros2 pkg executables | grep arena`
    
  - all excutables can be run by using 

    `ros2 run <pakg name> <executable name>`

- explore actions
  
  - None

- explore services 
  - trigger_image 
    - trigger image form a device that is running in trigger_mode.
    - To run a device in trigger mode
      `ros2 run arena_camera_node start --ros-args -p exposure_time:=<proper value> -p trigger_mode=true`
    - To trigger an image run 
      `ros2 run arena_camera_node trigger_image`

# Road map
- support windows
- add -h flag to nodes
- showimage node to view 2D and 3D images
- launch file
- camera_info
- access to nodemaps
- settings dump/read to/from file
- support two devices
