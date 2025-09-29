# A1_maneuver

- A1_perception과 연동해서 로봇의 운동을 제어하는 패키지

## Subscribe topics

1. Navigation status[From EveryBot]
- topic: /navi_datas
- type : robot_custom_msgs::msg::NaviState

2. Motor status[From EveryBot]
- topic: /motor_status
- type : robot_custom_msgs::msg::MotorStatus

3. Lidar scan(back)[From EveryBot]
- topic: /scan_back
- type : sensor_msgs::msg::LaserScan

4. Lidar scan(front)[From EveryBot]
- topic: /scan_front
- type : sensor_msgs::msg::LaserScan

5. Stop command[From Clabil]
- topic: /perception/action/stop
- type : std_msgs::msg::Int32

6. Cmd vel [From Nav2]
- topic: /cmd_vel_smoothed
- type : geometry_msgs::msg::Twist


## Publish topics
1. Cmd vel [to EveryBot]
- topic: /cmd_vel
- type : geometry_msgs::msg::Twist
