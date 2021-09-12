# purt_catkin_ws
PURT catkin workspace

## Setup

### Before you build
1. Make sure you have ROS noetic installed
2. Make sure your .bashrc only contains the following line for sourcing ros:

```bash
. /opt/ros/noetic/setup.bash

```

### Building

```bash
mkdir -p ~/git
cd ~/git
git clone https://github.com/jgoppert/purt_catkin_ws.git
cd purt_catkin_ws
git submodule update --init --recursive
./src/PX4-Autopilot/Tools/setup/ubuntu.sh 
```

Reboot computer

```bash
sudo apt install python3-catkin-tools ros-noetic-geographic-msgs ros-noetic-mavlink geographiclib-tools libgeographic-dev libignition-common3-graphics-dev xterm ros-noetic-web-video-server
cd ~/git/purt_catkin_ws
catkin build
. ./devel/setup.bash
```

Make with PX4 to create temporary directories

```bash
cd ~/git/purt_catkin_ws/src/PX4-Autopilot
make
```

Install geographiclib datasets

```bash
cd ~/git/purt_catkin_ws/src/mavros/mavros/scripts
./install_geographiclib_datasets.sh
```

### Troubleshooting

1. If PX4 is stuck building cmake, make sure you have run:
```bash
git submodule update --init --recursive
```
and that it completes successfully.

## Run

After the system is built, run:

```bash
cd ~/git/purt_catkin_ws
. ./devel/setup.bash
roslaunch qualisys abu_dhabi.launch
```

### Web Video Server (for VR Goggles)

Navigate to browser on desktop running simulation: 
http://0.0.0.0:8080/stream?topic=/vr_image

To navigate to stream on phone, need ip address of simulation computer, phone has to be on 
the same wifi network:
http://192.168.123.125:8080/stream?topic=/vr_image

On android set your screen timeout to 30 minutes. Settings -> Display -> Screen Timeout

On the phone use application: https://play.google.com/store/apps/details?id=tk.klurige.fullscreenbrowser&hl=en_US&gl=US, this is a full screen browser app.
