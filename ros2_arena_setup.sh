
# UNINSTALL ROS2 
UNINSTALL_ROS2=0
INSTALL_ROS2=1




set -x #echo on

# only run with sudo
if [ `id -u` -ne 0 ]; then 
    echo "Please run as root"
    exit
fi

# -----------------------------------------------------------------------------
#                                   R O S 2
# -----------------------------------------------------------------------------

if [ $UNINSTALL_ROS2 -eq 1 ]; then
    sudo apt remove ros-eloquent-* && sudo apt autoremove
    exit $?
fi

# ROS2::Eloquent full instalation guide
# https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

if [ $INSTALL_ROS2 -eq 1 ]; then

    # setup locale
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # setup Sources
    sudo apt update && sudo apt install curl gnupg2 lsb-release --assume-yes
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

    # install ROS 2 packages -------------------------------
    sudo apt update
    sudo apt install ros-eloquent-ros-base --assume-yes
    #   -   Desktop Install (Recommended): ROS, RViz, demos, tutorials.
    sudo apt install ros-eloquent-desktop --assume-yes
    #sudo apt-get install 'ros-eloquent-launch*' --assume-yes

    # env setup --------------------------------------------
    #   - echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
    #   - echo "source /opt/ros/eloquent/setup.zsh>" >> ~/.zshrc

    # install argcomplete (optional)
    # - sudo apt install python3-argcomplete --assume-yes

    # workspace --------------------------------------------

    # package dep
    #   - sudo apt-get update
    #   - sudo apt-get install python-rosdep
    #   - sudo rosdep init
    
    # for each package to install package deps run
    #   - sudo rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    #   - arena_camera uses image msg which require python3-numpy
    #     if was not installed by the rosdep for some reason, install using:
    #       - sudo apt-get install python3-numpy

    # for building workspaces with colcon
    #   - sudo apt install -y python3-colcon-common-extensions
    #   - cd to <X> package root 
    #   - colcon build
    #   - echo "source <X>/setup.bash" >> ~/.bashrc
    #   - echo "source <X>/setup.zsh>" >> ~/.zshrc
    #   - to run 

    # virtual_env
    #   - pip install lark-parser # for building services 
    
fi

# -----------------------------------------------------------------------------
#                                   ARENASDK
# -----------------------------------------------------------------------------