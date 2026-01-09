# Kurulum Kilavuzu

## Sistem Gereksinimleri

### Donanim
- CPU: Intel Core i5 veya ustu
- RAM: 8 GB minimum, 16 GB onerilen
- GPU: NVIDIA GPU (CUDA destekli, egitim icin onerilen)
- Depolama: 10 GB bos alan

### Yazilim
- Ubuntu 22.04 LTS (Humble) veya Ubuntu 20.04 LTS (Foxy)
- ROS2 Humble veya Foxy
- Gazebo 11
- Python 3.8+

## ROS2 Kurulumu

### Ubuntu 22.04 icin ROS2 Humble

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

## TurtleBot3 Kurulumu

```bash
sudo apt install ros-${ROS_DISTRO}-turtlebot3-gazebo
sudo apt install ros-${ROS_DISTRO}-turtlebot3-teleop
export TURTLEBOT3_MODEL=burger
```

## Python Bagimliliklari

```bash
pip3 install torch numpy tensorboard PyYAML matplotlib scipy transforms3d
```

## Proje Kurulumu

### Otomatik Kurulum

```bash
cd d3qn-sosyal-navigasyon
chmod +x scripts/kurulum.sh
./scripts/kurulum.sh
```

### Manuel Kurulum

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash

mkdir -p ~/d3qn_ws/src
cd ~/d3qn_ws/src
ln -s /path/to/d3qn-sosyal-navigasyon d3qn_sosyal_navigasyon

cd ~/d3qn_ws
colcon build --packages-select d3qn_sosyal_navigasyon --symlink-install

echo "source ~/d3qn_ws/install/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## Dogrulama

```bash
ros2 pkg list | grep d3qn
ros2 launch d3qn_sosyal_navigasyon hastane_egitim.launch.py gazebo_gui:=true
```
