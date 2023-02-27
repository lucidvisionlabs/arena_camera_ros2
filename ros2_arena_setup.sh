
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
    sudo apt remove ros-humble-* && sudo apt autoremove
    exit $?
fi

# ROS2::Humble Hawksbill full instalation guide
# https://docs.ros.org/en/humble/Installation.html

if [ $INSTALL_ROS2 -eq 1 ]; then

    # setup locale
    sudo apt update && sudo apt install locales --assume-yes
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # setup Sources
    sudo apt install software-properties-common --assume-yes
    sudo add-apt-repository universe
    
    # Now add the ROS 2 GPG key with apt.
    sudo apt update && sudo apt install curl --assume-yes
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # install ROS 2 packages -------------------------------
    sudo apt update  --assume-yes
    sudo apt upgrade  --assume-yes
    sudo apt install ros-humble-ros-base --assume-yes
    #   -   Desktop Install (Recommended): ROS, RViz, demos, tutorials.
    sudo apt install ros-humble-desktop --assume-yes
    sudo apt-get install 'ros-humble-launch*' --assume-yes

    # env setup --------------------------------------------
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    # Uncomment if use zshell
    #   - echo "source /opt/ros/humble/setup.zsh>" >> ~/.zshrc

    # install argcomplete (optional)
    # - sudo apt install python3-argcomplete --assume-yes

    # workspace --------------------------------------------

    # workspace package dep
    sudo apt-get update --assume-yes
    sudo apt install python3-rosdep2 --assume-yes
    sudo rosdep init
    rosdep update
    
    # for each package to install package deps run
    #   - sudo rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    #   - arena_camera uses image msg which require python3-numpy
    #     if was not installed by the rosdep for some reason install using:
    #       - sudo apt-get install python3-numpy

    # workspace building tool
    sudo apt install -y python3-colcon-common-extensions --assume-yes
    # for building workspaces with colcon
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
