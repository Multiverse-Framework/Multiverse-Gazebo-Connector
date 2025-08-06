# Multiverse-Gazebo-Connector

```bash
sudo apt install python3-pip python3-venv lsb-release gnupg curl git
pip3 install vcstool colcon-common-extensions
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
mkdir -p ./gazebo_ws/src
cd ./gazebo_ws/src
curl -O https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-ionic.yaml
vcs import < collection-ionic.yaml
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
cd ..
colcon build --merge-install
cd ..
```

```bash
cd ./plugin/multiverse_connector
mkdir build
cd build
cmake ..
make
cd ../..
```

```bash
source ./gazebo_ws/install/setup.bash
export GZ_CONFIG_PATH="$PWD"/gazebo_ws/install/share/gz
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD"/plugin/multiverse_connector/build
```

```bash
gz sim shapes.sdf
```
