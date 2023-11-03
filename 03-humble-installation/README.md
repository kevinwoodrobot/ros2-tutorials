# ROS2 Humble Installation 
Installation:
https://docs.ros.org/en/humble/Installation.html

1. **Locale Configuration**
```bash
locale  # Check current locale settings
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # Confirm updated locale settings
```

2. **Software Properties and Universe Repository**
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

3. **Curl Installation and ROS Key**
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

4. **ROS2 Sources Configuration**
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

5. **Update and Upgrade**
```bash
sudo apt update
sudo apt upgrade
```

6. **ROS Desktop Installation**
```bash
sudo apt install ros-humble-desktop
```

7. **ROS Environment Setup**
```bash
source /opt/ros/humble/setup.bash
```

8. **ROS2 Command Check**
```bash
ros2 
```

# Next Video:<br>Command Not Found