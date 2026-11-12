# kebi_follow_me

카메라(YOLO, RK3588 NPU) + 2D 라이다로 사람 다리를 보고 따라가는 follow-me.

| 패키지 | 내용 |
|---|---|
| kebi_follow_msgs | 내부 메시지 |
| kebi_follow_detector | `cam_leg_detector` 카메라 다리 검출 |
| kebi_follow_tracker | `leg_tracker` 라이다 다리 추적, 타깃 관리 |
| kebi_follow_controller | `follow_controller` 상태기계, 추종 제어, 안전 게이트 |
| kebi_follow_me | launch, 파라미터, 문서 |

```bash
colcon build --packages-up-to kebi_follow_me
ros2 launch kebi_follow_me follow_me.launch.py            # shadow:=true 면 /cmd_vel 대신 그림자 토픽
ros2 topic pub --once /follow/ui/client follow_msgs/msg/UiClient "{cmd: 1}"   # START (3 = STOP)
```

사양: [docs/SPEC.md](docs/SPEC.md)
