# 추종 타깃 상태. leg_tracker 가 scan 마다(10 Hz) 발행하고 follow_controller 가 쓴다.
uint8 STATUS_NONE=0        # 락 없음 (OFF 이거나 후보 없음)
uint8 STATUS_CANDIDATE=1   # ACQUIRE 중 후보는 있으나 확인 시간 미충족 (주행 금지)
uint8 STATUS_LOCKED=2      # 락 + 이번 주기에 측정 갱신
uint8 STATUS_OCCLUDED=3    # 락 유지, 측정 없음 (예측값). occluded_time 참고
uint8 STATUS_LOST=4        # 예측 한계 초과. 재획득 대기

std_msgs/Header header           # frame_id = base_link, stamp = 위치가 유효한 시각
uint8 status
int32 track_id
geometry_msgs/Point position     # base_link 기준 [m]
geometry_msgs/Vector3 velocity   # 타깃의 지면(odom) 속도를 base_link 축으로 표현 [m/s]
float32 range                    # [m]
float32 bearing                  # [rad] 왼쪽(+), 정면 0
float32 position_std             # [m]
float32 occluded_time            # 마지막 측정 이후 시간 [s]
bool camera_confirmed            # 최근 camera_confirm_window 안에 카메라 검출과 일치
bool ambiguous                   # 게이트 안에 경쟁 후보가 있음 (정지 사유 S8)
