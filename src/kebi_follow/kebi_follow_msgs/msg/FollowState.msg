# follow_controller 상태 (10 Hz, 모니터링·로그용)
uint8 FSM_IDLE=0
uint8 FSM_ACQUIRING=1
uint8 FSM_FOLLOWING=2
uint8 FSM_WAITING=3     # 타깃 상실·모호·센서 문제로 정지. 조건이 풀리면 자동 재개
uint8 FSM_HALTED=4      # 낙하·충돌 등으로 래치 정지. START 를 다시 받아야 해제
uint8 FSM_ENDED=5

uint32 STOP_NO_TARGET=1
uint32 STOP_TOO_CLOSE=2
uint32 STOP_OBSTACLE=4
uint32 STOP_ROTATION_BLOCKED=8
uint32 STOP_CLIFF=16
uint32 STOP_COLLISION=32
uint32 STOP_SENSOR_TIMEOUT=64
uint32 STOP_AMBIGUOUS=128
uint32 STOP_ROBOT_STATE=256
uint32 STOP_ROTATION_LIMIT=512

std_msgs/Header header
uint8 fsm
uint32 stop_reasons             # STOP_* 비트 합
bool shadow                     # true 면 /cmd_vel 대신 그림자 토픽으로만 발행 중
float32 cmd_v                   # 최종 명령 [m/s]
float32 cmd_w                   # [rad/s]
float32 v_limit                 # 이번 주기 선속도 상한 [m/s]
float32 obstacle_distance       # 진행 경로 최근접 장애물 [m], 없으면 큰 값
float32 target_range            # [m], 타깃 없으면 -1
float32 target_bearing          # [rad]

# --- 진단용 추가 필드 (W3, append-only) ---
float32 scan_obstacle_distance  # 라이다 기준 진행 경로 장애물까지 로봇 앞면 거리 [m], 없으면 큰 값
float32 depth_obstacle_distance # depth 기준 [m], 없으면 큰 값
bool depth_ok                   # depth 가 신선하고 healthy(바닥 평면 확보, 가까운 영역 유효 비율 충분). false 면 선속도 상한
float32 floor_pitch_deg         # depth 바닥 평면으로 추정한 카메라 숙임 [deg], 추정 실패 시 NaN
float32 depth_proc_ms           # 최근 depth 프레임 처리 시간 [ms]
float32 cycle_ms                # 최근 제어 주기 계산 시간 [ms]
uint8 floor_source              # depth 바닥: 0 없음(degraded), 1 이번 추정, 2 직전 추정(hold 안), 3 오래된 추정(degraded)
float32 depth_valid_ratio       # 로봇 앞 가까운 영역(0.6~1.2 m) 표본 중 유효 depth 비율
# 조향 목표 (2026-11-13 갇힘 개선): 0 사람에게 곧장, 1 직선이 막혀 사람 발자취 위 앞쪽 점,
# 2 막혀서 국소 회피, 3 벽에 붙어 지나온 길로 짧게 후진
uint8 STEER_DIRECT=0
uint8 STEER_TRAIL=1
uint8 STEER_AVOID=2
uint8 STEER_REVERSE=3
uint8 steer_mode
