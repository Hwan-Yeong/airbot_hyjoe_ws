# kebi follow-me 사양 v0

2026-11-06 작성. 시험하면서 값을 조정한다. 바꾸면 이 문서와 해당 config 를 같이 고친다.

## 1. 범위

- v0 목표: 앞에 선 사람을 획득하고, 다리를 보고 따라가며, 위험하면 멈춘다.
- 범위 밖: 장애물 회피 경로 생성, 후진, 지도·금지구역, 여러 사람 중 선택 UI.
- 실행 보드는 AMR(RK3588)이다. 카메라 영상은 Jetson 에서 DDS 로 받는다.
- 기존 바이너리 follow-me(`namuhx_follow_me`)를 대체한다. 그 바이너리는 인터페이스(토픽 이름)만 참고했다.

## 2. 구성

| 패키지 | 노드 | 내용 | 담당 |
|---|---|---|---|
| `kebi_follow_msgs` | - | 내부 메시지 | 리드 |
| `kebi_follow_detector` | `cam_leg_detector` | YOLO(NPU) + depth → 다리 위치 | W1 |
| `kebi_follow_tracker` | `leg_tracker` | 라이다 다리 추적, 타깃 획득·유지 | W2 |
| `kebi_follow_controller` | `follow_controller` | 상태기계, 추종 제어, 안전 게이트 | W3 |
| `kebi_follow_me` | 컨테이너 | launch, 덮어쓰기 파라미터, 문서 | 리드 |

- 세 노드는 `component_container_mt` 프로세스 하나로 띄운다(`follow_me.launch.py`).
  - 이유: AMR 의 nav2 컨테이너가 새 DDS 참여자가 생길 때마다 discovery 패킷 폭주를 겪는다. 참여자를 하나로 줄인다.
- 기본 파라미터는 각 모듈 패키지 `config/<노드>.yaml` 에 둔다. `kebi_follow_me/config/follow_me.yaml` 은 시험 단계별 덮어쓰기만 담는다.

## 3. 좌표계·센서·시간

### 좌표계

- `base_link`: x 전방, y 왼쪽. 로봇 반경 0.22 m.
- `odom`: 추적용 세계 좌표. `odom→base_link` 가 100 Hz 로 들어온다.
- `map`: 주행 중에만 있다. **사용하지 않는다.**

### 센서

| 센서 | 토픽 | 프레임 | 사양 |
|---|---|---|---|
| 라이다 | `/scan` | `base_scan` | base_link +0.155 m, 360°, 0.9° 간격, 401점, 10 Hz, 0.03~8 m. 앞뒤 라이다 2개를 병합한 것. 개별 토픽은 `/scan_front`, `/scan_back`(각 220°) |
| RGB | `/camera/rgb/image_raw` | `inusensor_rgb` | 640×480 bgr8, fx=fy=369.8, cx 320.3, cy 267.9, 약 8.5~11 Hz |
| depth | `/camera/depth/image_raw` | `inusensor_depth` | 848×480 16UC1 [mm], fx 424, fy 340, 약 8.5~11 Hz |
| 오도메트리 | `/odom` | | 100 Hz |

- 카메라 TF(`base_link→카메라`)는 Jetson URDF 값을 쓴다: z 0.26 m, pitch 0°.
- 실제 숙임은 약 9~10.5°(아래)다. 도킹 위치 bag 에서 바닥 추정치는 10.5 ± 0.45° 였고, 세션마다 달라진다. 높이를 쓰는 계산은 바닥 평면 추정이나 `camera_pitch_correction_deg` 파라미터로 보정한다.
- RGB 아래쪽 약 30%(행 330 이후)에는 로봇 자기 몸체가 찍힌다.
- 계산상 1 m 앞 사람은 허벅지 아래(약 0.7 m)까지만 보인다. 1.5 m 이면 엉덩이, 2 m 이면 허리까지 보인다.

### 시간 — 주의

- 카메라 stamp 는 Jetson 시계(정상 epoch)를 쓴다.
- AMR 시계는 설계상 epoch 가 +9 h(약 32400 s)다. `/scan`, `/odom`, TF stamp 는 AMR 시계를 쓴다.
- `cam_leg_detector` 는 출력 stamp 를 AMR 시간축으로 바꿔서 낸다.
  - offset 추정: 최근 수 초 동안 "영상 수신 순간의 최신 `/odom` stamp − 영상 stamp" 를 모아 최솟값을 쓴다.
  - 변환: `stamp_out = stamp_cam + offset − camera_min_latency_s`. 기본 `camera_min_latency_s` 는 0.10 s.
  - 실측 (2026-11-06, 첫 데이터 bag): depth 내용은 라이다보다 약 0.1 s 늦다. 걷는 구간 상호상관 +0.04~0.10 s, 방향 전환 시점 차 +0.02~0.26 s.
  - 같은 bag 에서 시계 차(AMR − Jetson)는 32396.5 s 였다. 9 h 와 다르고 부팅마다 달라질 수 있으니 고정값을 쓰지 않는다.
- 이 방식은 실시간과 bag 재생(`--clock`) 양쪽에서 같게 동작한다.

## 4. 토픽

내부 토픽은 모두 `/kebi_follow/` 아래에 있다.

| 토픽 | 타입 | 발행 → 구독 | QoS | 주기 |
|---|---|---|---|---|
| `/follow/ui/client` | follow_msgs/UiClient | state_manager → controller | reliable, depth 10 | 이벤트 |
| `/follow/detectuser` | follow_msgs/Detect | controller → state_manager | reliable, depth 1 | 변화 시 + 1 Hz |
| `/kebi_follow/mode` | FollowMode | controller → detector, tracker | reliable + transient_local, depth 1 | 변화 시 + 1 Hz |
| `/kebi_follow/cam_legs` | CamLegArray | detector → tracker | reliable, depth 5 | 영상마다 |
| `/kebi_follow/target` | FollowTarget | tracker → controller | reliable, depth 5 | scan 마다 |
| `/kebi_follow/tracks` | LegTrackArray | tracker → (디버그) | reliable, depth 5 | scan 마다 |
| `/kebi_follow/state` | FollowState | controller → (모니터) | reliable, depth 5 | 10 Hz |
| `/cmd_vel` | geometry_msgs/Twist | controller → 베이스 | reliable, depth 1 | 활성일 때 20 Hz |
| `/kebi_follow/cmd_vel_shadow` | geometry_msgs/Twist | controller(그림자 모드) | reliable, depth 1 | 20 Hz |
| `/kebi_follow/<노드>/markers` | visualization_msgs/MarkerArray | 각 노드 → RViz | best effort | 선택 |
| `/kebi_follow/debug_image` | sensor_msgs/CompressedImage | detector → RViz | best effort | `publish_debug_image` 일 때만 |

- 센서 입력 구독은 SensorDataQoS(best effort, depth 5)로 한다.
- 영상 구독은 mode 가 OFF 가 아닐 때만 만든다. 추종하지 않을 때 대역폭과 CPU 를 쓰지 않기 위해서다.

## 5. 상태기계 (`follow_controller` 소유)

| 상태 | mode | 진입 조건 | 동작 | 나가는 조건 |
|---|---|---|---|---|
| IDLE | OFF | 시작, STOP | `/cmd_vel` 미발행(진입 시 0 을 3회) | START → ACQUIRING |
| ACQUIRING | ACQUIRE | START | 0 속도 발행 | target LOCKED → FOLLOWING / `acquire_timeout_s`(30 s) → ENDED |
| FOLLOWING | TRACK | 락 | 추종 제어 + 안전 게이트 | 7절 정지 조건 → WAITING 또는 HALTED |
| WAITING | TRACK | 타깃 상실·모호·센서 끊김 | 0 속도 | 조건 해소가 `resume_confirm_s`(0.5 s) 유지 → FOLLOWING / `lost_end_timeout_s`(30 s) → ENDED |
| HALTED | TRACK | 낙하·충돌·로봇 이상 | 0 속도, 래치 | START → ACQUIRING (재획득부터) |
| ENDED | OFF | 시간 초과 | 진입 시 0 을 3회, 이후 미발행 | START → ACQUIRING |

- 어느 상태에서든 STOP 을 받으면 IDLE 로 간다.
- **IDLE·ENDED 에서는 `/cmd_vel` 을 발행하지 않는다.** 노드는 항상 떠 있으므로, 다른 주행 기능과 명령이 겹치면 안 된다.
  - 0 명령 3회는 활성 상태(ACQUIRING·FOLLOWING·WAITING·HALTED)에서 나갈 때만 낸다. 노드 시작과 ENDED→IDLE 때는 내지 않는다.
- HALTED 해제: 마지막 HALT 사유 뒤 1 s 안에 온 START 는 무시한다. state_manager 가 START 를 반복해 보내도 래치가 바로 풀리지 않게 하기 위해서다.
- WAITING 이 사유와 상관없이 30 s 이어지면 ENDED 다.

### state_manager 연동 계약 (popo 브랜치)

- `/follow/ui/client.cmd`: 1 이면 START, **그 밖의 값은 모두 STOP** 이다(원래 값을 로그로 남긴다).
  - popo 브랜치 state_manager 는 START=1, STOP=3 을 쓴다.
  - 신형 state_manager(PAUSE=2, RESUME=3, STOP_SOC=4, STOP_AMR=5)가 와도 정지 쪽으로 동작한다. 신형의 RESUME=3 도 STOP 이 된다.
- START 이전 stamp 의 타깃은 버린다. HALTED 뒤 START 가 지난 세션의 LOCKED 로 바로 추종하지 않게 하기 위해서다.
- `/emergency_stop`, `/error/s_code/cliff_detected` 의 마지막 값이 true 인 동안은 START 를 거부한다.
- `/follow/detectuser.detect_user` 값:

  | 값 | 의미 | 해당 상태 |
  |---|---|---|
  | 0 | 종료 | ENDED, HALTED |
  | 1 | 인식, 정지 중 | 락이 있고 이동하지 않음 (ACQUIRING 후보, FOLLOWING 근접 정지, WAITING 중 장애물) |
  | 2 | 영역 안에 사람 없음 | 후보 없음, 타깃 상실 |
  | 3 | 이동 중 | FOLLOWING 에서 \|v\|>0.02 또는 \|w\|>0.05 |

- `detect_status` 에는 FSM 값을 넣는다(참고용).
- state_manager 동작:
  - 0 을 받으면, 한 번이라도 3(이동)을 받은 뒤라면 COMPLETE 로 Following 을 끝낸다. 끝낼 때 STOP 을 보낸다.
  - ONSTATION 이면 state_manager 가 끝낸다.
- 개발 중(P3 까지)에는 state_manager 없이 시험한다. 언도킹 후 IDLE 상태에서 컨테이너를 띄우고 `/follow/ui/client` 로 START/STOP 을 직접 보낸다.

## 6. 모듈 사양

### 6.1 cam_leg_detector (W1)

- 모델: COCO 사전학습 YOLO, `person` 클래스. RK3588 NPU(RKNN)에서 실행한다.
  - 런타임 `librknnrt.so` 는 패키지에 넣어 배포한다(sudo 불필요).
  - 다리 전용 모델은 나중에 필요할 때 만든다.
- 처리: 가장 최근 프레임만 처리한다. 바쁘면 이전 프레임을 버린다.
- 위치 계산: bbox 아래쪽(다리 부분)의 depth 점을 RGB 좌표로 정합한 뒤 가까운 쪽 군집의 중앙값을 쓴다. 결과는 base_link 지면 투영 (x, y) 이다.
- 출력: `CamLegArray`. stamp 는 3절 방식으로 AMR 시간축으로 바꾼다. 검출이 없어도 프레임마다 발행한다.
- 예산: 프레임당 60 ms 이하, 평균 CPU 한 코어의 30% 이하.

### 6.2 leg_tracker (W2)

- 다리 후보 검출:
  1. `/scan` 을 거리 점프 기준으로 분할한다.
  2. 형상 특징으로 다리를 판정한다: 폭 0.05~0.25 m, 점 수, 원호 적합.
  3. 다리 두 개(간격 0.1~0.6 m)나 붙은 덩어리를 사람 후보로 만든다.
- 추적: odom 좌표에서 등속 KF 를 쓴다. 연관은 Mahalanobis 게이트 + 전역 최적 할당(GNN)으로 한다.
- 카메라 융합: `cam_legs` 를 stamp 시각의 odom 으로 변환해 0.4 m 게이트 안 트랙과 연관한다. 연관되면 `camera_confirmed` 와 `person_score` 를 올린다.
- 획득 (mode ACQUIRE): 아래를 모두 만족하면 LOCKED 다.
  - 범위 0.5~1.5 m, \|bearing\| ≤ 20°
  - 사람 형상
  - `camera_confirmed` (파라미터 `acquire.require_camera`, 기본 true)
  - `acquire.confirm_s`(1.0 s) 동안 유지
  - 영역 안 후보가 1명뿐
- 유지 (mode TRACK):
  - 측정이 없으면 OCCLUDED 로 두고 예측을 낸다.
  - `lost_s`(2.0 s)가 지나거나 불확실성이 0.5 m 를 넘으면 LOST 다.
  - 재획득: LOST 이후 마지막 예측 위치 1.0 m 안에서 같은 조건(사람 형상, 카메라 확인, 후보 1명)이 0.5 s 유지되면 다시 LOCKED 다.
- 모호: 타깃 0.6 m 안에 다른 사람 트랙이 있거나 연관 비용이 비슷하면 `ambiguous=true` 다.
- 출력: scan 마다 `FollowTarget`, `LegTrackArray`, markers.
  - `FollowTarget.position` 은 그 scan 시각의 base_link 기준이다.
- 구현하며 더한 규칙 (W2, 모두 파라미터로 조절 가능):
  - 정지 물체 트랙: 3 s·15회 이상 갱신되며 움직인 적 없는 트랙은 정지 물체 모델(게이트 약 0.12 m)을 쓴다. 카메라 확인이 없으면 경쟁 후보·모호 판정에서 사람으로 세지 않는다.
  - 라이다만으로 락하려면 그 트랙이 0.3 m 이상 걸어온 이력이 있어야 한다(`track.static_shape_factor`). 카메라 확인이 있으면 이 조건이 필요 없다. 기둥+의자 다리 쌍 거짓 락을 막는다.
  - 재획득 영역: 마지막 관측 위치 ~ 마지막 예측 위치 선분 주변 1.0 m (`target.reacquire_from_last_seen`).
  - 카메라 미확인 LOST: 카메라 프레임이 오고 있고 타깃이 카메라 확인 영역(±35°, 0.6~3.5 m) 안인데 3 s 넘게 확인되지 않으면 LOST (`target.camera_lost_s`). 사람이 떠난 뒤 사람 모양 가구에 붙는 것을 막는다.
    - 최소 거리는 통합 재생 결과로 1.0 → 0.6 m 로 낮췄다. 실 bag 0.5~0.8 m 카메라 검출률이 0.99 다.
- 알려진 한계: 서 있는 타깃 0.4 m 앞을 빠르게 지나는 교차에서 드물게(합성 1/20) 타깃이 넘어간다. 그동안 `ambiguous` 가 켜져 있어 S8 로 정지한다.

### 6.3 follow_controller (W3)

- 20 Hz 타이머로 돈다. 타깃은 stamp 이후 경과 시간만큼 `velocity` 로 외삽한다.
- 추종 법칙:
  - 거리 오차 `e = range − follow_distance`(1.0 m). `v = k_v·e + v_ff`
    - `v_ff`: 타깃 속도의 로봇 전방 성분. 0 이상만 쓴다.
  - `w = k_w·bearing`. 0.05 rad 불감대가 있다.
  - \|bearing\| 이 15° 를 넘으면 v 를 줄이기 시작해 45° 에서 0 이 된다. 45° 를 넘으면 제자리 회전한다.
- 사람 앞 제동 상한: `v ≤ 정지가능속도(range − 0.8 − 0.1 m, 감속 0.8 m/s², 지연 0.2 s)`. 사람이 갑자기 서도 0.9 m 앞에 선다.
  - 그래서 추종 거리가 속도에 따라 늘어난다: 0.3 m/s 이하 약 1.0 m, 0.5 m/s 약 1.16 m.
- 가감속: 선가속 0.4, 선감속 0.8 m/s², 각가속 1.5 rad/s². jerk 2.0 m/s³ 는 가속 쪽에만 건다.
  - 즉시 0 (감속 제한 없음): HALT, S3 장애물 정지, S2 근접, 센서 끊김, 회전 금지·회전 상한.
  - 감속 제한 정지 (0.8 m/s²): 타깃 상실, 모호.
- 시간: 주기 dt·신선도·FSM 타이머는 steady clock(재생 시 ROS 시간)을 쓴다. stamp 와 비교하는 외삽·odom 이력만 ROS 시간을 쓴다.
- 장애물(S3): `/scan` 과 depth 로 진행 경로 안 최근접 장애물 거리 d 를 구한다.
  - 타깃 주변 `target_exclusion_radius`(0.4 m) 안의 점은 제외한다.
  - `v ≤ sqrt(2·decel·max(0, d − margin))`. 1.0 m 부터 감속한다.
  - 진행 경로는 직진과 현재 곡률 원호의 합집합이고, 거리는 로봇 둥근 앞면 기준이다.
  - depth 는 프레임마다 바닥 평면(RANSAC)을 빼고, 바닥 위 3.5 cm(0.05 − 허용 0.015)~1.2 m 점만 장애물로 본다.
  - depth 점은 (수신 − `camera_latency_s` 0.1 s) 시점 odom 으로 이동을 보정한다. 타깃 제외는 그동안 타깃이 지나온 경로 전체로 한다.
  - depth 장애물의 정지 여유는 0.5 m 다. 카메라 근거리 사각(앞면 약 0.4 m 안)에 들어가기 전에 서기 위해서다.
  - 바닥 평면 추정 (독립 리뷰 M2 반영): RANSAC 가설은 로봇 앞 0.6~1.0 m 띠에서만 뽑는다.
    - 시작할 때 서로 맞는 추정 3 프레임(기준 숙임 ±3° 안)이 이어져야 기준(anchor)으로 받는다.
    - 그 뒤에는 기준에서 1°·1.5 cm 안의 추정만 받고, 출력 변화는 프레임당 0.5°·0.5 cm 로 제한한다.
    - 기준 평면으로 조용히 대신하는 경로는 없다. 먼 바닥 depth 가 없을 때 낮은 장애물을 바닥으로 흡수하는 것을 막는다.
  - depth 정상 판정 (M1 반영): 받은 바닥 평면이 2 s 안에 있고, 로봇 앞 0.6~1.2 m 영역의 유효 depth 비율이 0.15 이상이어야 한다.
    - 아니면 카메라 끊김과 같이 선속도 0.3 m/s 상한을 걸고 `depth_ok=false` 로 알린다.
  - 숙임이 두 번 들어가면(TF 숙임 > 2° 이면서 보정값 ≠ 0) ERROR 를 내고 depth 를 비정상으로 본다.
  - 실측 (도킹 bag, 0.6~1.0 m 띠): 숙임 9.85 ± 0.24°. 바닥은 0.7 m 와 2 m 사이에서 약 3 cm 휘어 있다.
- 입력 검사: NaN·Inf 이거나 8 m 를 넘는 타깃은 받지 않은 것으로 본다. 타깃 나이는 "수신 후 경과"와 "stamp 후 경과" 중 큰 값이다.
- 사건형 안전 토픽은 reliable 로 구독한다(발행자가 reliable).
- 입력: `/kebi_follow/target`, `/scan`, `/odom`, `/camera/depth/image_raw`, camera_info, TF, 7절 안전 토픽, `/follow/ui/client`.
- 출력: `/cmd_vel`(그림자 모드면 shadow 토픽), `/follow/detectuser`, `/kebi_follow/mode`, `/kebi_follow/state`.

## 7. 안전 사양 v0 (임시)

| # | 항목 | 값 · 동작 | 결과 상태 |
|---|---|---|---|
| S1 | 속도 | 선속도 상한 0.5 m/s(T3 까지 0.2), 각속도 0.8 rad/s, 가속 0.4, 감속 0.8 m/s², 후진 없음 | - |
| S2 | 거리 | 목표 1.0 m, 0.8 m 이내 정지(FOLLOWING 과 WAITING 모두, 모호·가림 중에도). 사람이 다가와도 물러서지 않음 | FOLLOWING(정지) |
| S3 | 전방 장애물 | 진행 경로 반폭 0.32 m(반경 0.22 + 0.10). 1.0 m 부터 감속, 제동거리 + 0.25 m 이내 정지. 라이다 + depth | FOLLOWING(정지) |
| S4 | 회전 | 중심에서 0.30 m 안에 물체가 있으면 제자리 회전 금지. 제자리 회전 연속 3 s 상한 후 정지, 2 s 뒤 또는 타깃이 45° 안이면 다시 허용 | WAITING |
| S5 | 낙하·충돌 | `/bottom_ir_data` 플래그 하나라도, `/collision_detected`(event_trigger≠0), `/sensor_to_pointcloud/collision`(점 있음), `/perception/action/stop`(≠0)이 오면 즉시 0 | HALTED |
| S6 | 센서 끊김 | `/scan` 0.3 s, `/odom` 0.2 s, `/bottom_ir_data` 0.2 s, `lifted` 1 s, `fall_down` 3 s, TF 실패 시 정지. 카메라만 끊기거나 depth 가 비정상이면(1 s) 선속도 0.3 m/s 로 제한 | WAITING |
| S7 | 타깃 상실 | 측정 없는 시간 0.7 s 까지는 속도 절반, 넘으면 정지. 30 s 지나면 종료 | WAITING → ENDED |
| S8 | 모호 | `ambiguous` 이면 정지 | WAITING |
| S9 | 로봇 상태 | `/error/s_code/lifted`, `fall_down`, `cliff_detected` 가 true, `/emergency_stop` true | HALTED |
| S10 | 워치독 | 20 Hz 발행. 입력이 끊기면 0. 종료 시 0 발행 | - |

- **S10 보충:** 베이스(uart_communication)는 cmd_vel 이 1 s 넘게 안 오면 0 으로 바꾼다. 하지만 선속도가 0.001 이하이면 이 보호가 걸리지 않는다. 그래서 제자리 회전 명령은 프로세스가 죽으면 계속될 수 있다.
  - 대책: 제자리 회전 중 선속도에 0.002 m/s(모터 불감대 0.01 미만)를 넣어 베이스 보호가 걸리게 한다.
  - 실차 T2 에서 프로세스를 죽여 검증한다.
  - 근본 대책(베이스 코드 수정)은 회사 저장소 변경이라 따로 제안한다.
- 안전 토픽 실측 (도킹 상태): `/bottom_ir_data` 100 Hz, `/error/s_code/lifted` 83 Hz, `fall_down` 1 Hz 는 항상 발행된다.
  - 이 셋은 레벨 신호로 다루고 1.5 s 유지한다. `/bottom_ir_data` 가 0.2 s 끊기면 센서 끊김(S6)으로 본다.
  - 충돌, `.../collision/front|rear`, `/perception/action/stop`, `cliff_detected`, `/emergency_stop` 은 사건형이다.
  - `/sensor_to_pointcloud/collision` 은 발행 노드가 없다.

### 남은 위험 (v0)

- 라이다(높이 0.155 m)보다 낮은 장애물은 depth 만 본다. 근거리 사각(앞면 약 0.4 m 안)에 처음 들어온 물체는 못 본다. 예: 제자리 회전 직후. 장애물 기억은 넣지 않았다(유령 정지 위험).
- 사람이 넘어간 낮은 물체는 사람이 0.4 m 지나간 뒤에야 보인다(타깃 제외 반경).
- 카메라가 끊기면 0.3 m/s 로 계속 가는 동안 낮은 장애물을 못 본다.
- 즉시 0 이후 베이스의 실제 제동 거리는 아직 재지 않았다. S3 상한은 0.8 m/s² 를 가정한다.
- 숙임 보정(`camera_pitch_correction_deg`)은 TF 위에 더한다. Jetson URDF 에 실제 숙임이 들어가면 0 으로 바꿔야 한다.

## 8. 시험 단계

| 단계 | 내용 | 조건 |
|---|---|---|
| T1 그림자 | 로봇 정지, `shadow:=true`. 사람이 앞에서 움직이고 명령 값만 기록 | 획득·유지·정지 판단이 맞음 |
| T2 회전 | 선속도 0, 회전만 (상한 0.3 rad/s) | 사람 쪽으로 돌고 S4·S10 확인 |
| T3 저속 | 0.2 m/s 직선 추종, 정지 시험(멈춤, 장애물, 시야 이탈, 교차) | S1~S10 확인 |
| T4 정상 | 0.4~0.5 m/s, 코너 | 사용자 동행 |

- 비상 정지: PC 에서 STOP + 0 속도를 보내는 스크립트(로봇 쪽 도구)와 로봇 전원.

## 9. 개발 규칙

- 빌드는 워커마다 따로 한다: `colcon build --build-base build/<w> --install-base install/<w> --packages-up-to <pkg>`.
- 로컬 ROS 시험은 격리한다: `export ROS_DOMAIN_ID=<71|72|73|77> ROS_LOCALHOST_ONLY=1; unset FASTRTPS_DEFAULT_PROFILES_FILE`. 로봇 도메인(30)에 아무것도 내보내지 않는다.
- 로봇 조작(노드 실행, 발행, 이동)은 리드만 한다.
- 이 레포는 public 이다.
  - 내부 IP, 접속 정보, 사람이 찍힌 이미지·bag 은 넣지 않는다.
  - 로봇용 스크립트와 데이터는 레포 밖 비공개 작업 폴더에 둔다.
- 메시지 변경: 필드 추가만 하고(이름 변경·삭제 금지) 리드와 합의한다. `FollowTarget` 은 W2·W3 둘 다 쓴다.
