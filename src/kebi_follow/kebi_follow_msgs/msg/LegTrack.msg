# leg_tracker 의 사람 트랙 1개 (디버그·시각화용)
uint8 KIND_PAIR=0      # 다리 두 개
uint8 KIND_SINGLE=1    # 다리 하나만 보임
uint8 KIND_MERGED=2    # 두 다리가 붙어 한 덩어리

int32 id
uint8 kind
geometry_msgs/Point position     # 상위 header.frame_id(odom) 기준 [m]
geometry_msgs/Vector3 velocity   # odom 기준 [m/s]
float32 position_std             # [m]
float32 person_score             # 0..1 사람일 가능성 (라이다 형상 + 카메라 확인)
bool camera_confirmed
uint32 age                       # 측정 갱신 횟수
float32 time_since_update        # [s]
