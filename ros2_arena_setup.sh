
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
    sudo apt remove ros-dashing-* && sudo apt autoremove
    exit $?
fi

# ROS 2 full instalation guid
# https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/

if [ $INSTALL_ROS2 -eq 1 ]; then

    # setup locale
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # setup Sources
    sudo apt update && sudo apt install curl gnupg2 lsb-release --assume-yes
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'


    # install ROS 2 packages
    sudo apt update
    sudo apt install ros-dashing-ros-base --assume-yes
    #   -   Desktop Install (Recommended): ROS, RViz, demos, tutorials.
    sudo apt install ros-dashing-desktop --assume-yes

    # env setup
    echo "source /opt/ros/dashing/setup.bash">> ~/.bashrc
    echo "source /opt/ros/dashing/setup.zsh>">> ~/.zshrc

    # install argcomplete (optional)
    sudo apt install python3-argcomplete --assume-yes

fi

# -----------------------------------------------------------------------------
#                                   ARENASDK
# -----------------------------------------------------------------------------
