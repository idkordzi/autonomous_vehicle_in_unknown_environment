sudo apt update
sudo apt upgrade -y
sudo apt install -y python3-pip ros-dev-tools stm32flash ros-${ROS_DISTRO}-teleop-twist-keyboard

export HUSARION_ROS_BUILD_TYPE=simulation
export PIP_BREAK_SYSTEM_PACKAGES=1

cd simulation
vcs import modules < modules/rosbot_ros/rosbot/rosbot_${HUSARION_ROS_BUILD_TYPE}.repos
rm -rf modules/rosbot_ros/rosbot_bringup

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths modules -y -i
