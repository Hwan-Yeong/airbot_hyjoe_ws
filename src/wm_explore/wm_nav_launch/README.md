# TotalLauncher
airbot 오토맵핑용 navigation 설정 파일

## 개발 환경
* ROS2 Foxy
* Ubuntu 20.04
* gcc 9.4.0
* visual code

## Tutorials


### Default Environment
```
start or source ~/.bashrc_setup
```

### Build
```
cd ~/ros2_ws/src/launcher/total_launcher
colcon build --symlink-install
cd install
source setup.bash
```

### Edit
```
cd ~/ros2_ws/src/launcher/total_launcher/param
```

### Start
```
ros2 launch total_launcher total_launch.py
```
