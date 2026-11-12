# 추종 모드. follow_controller 가 발행하고 detector·tracker 가 따른다.
# QoS: reliable + transient_local, depth 1. 바뀔 때와 1 Hz 로 발행.
uint8 OFF=0       # 추종 안 함. detector 는 영상 구독 해제, tracker 는 락 해제
uint8 ACQUIRE=1   # 타깃 획득 중. tracker 는 획득 영역에서 사람을 찾아 락
uint8 TRACK=2     # 락 유지·추적 (정지 대기 중에도 TRACK)

std_msgs/Header header
uint8 mode
