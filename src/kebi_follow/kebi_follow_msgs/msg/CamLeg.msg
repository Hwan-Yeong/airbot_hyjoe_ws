# 카메라 검출 1개: RGB bbox + depth 로 구한 위치
uint8 CLASS_PERSON=0   # COCO person (가까우면 다리만 보이는 사람)
uint8 CLASS_LEG=1      # 다리 전용 모델을 쓸 때

uint8 class_id
float32 score                   # 0..1

# RGB 영상 좌표 bbox [px], camera_info 해상도 기준
float32 u_min
float32 v_min
float32 u_max
float32 v_max

bool has_position               # depth 로 위치를 구했으면 true
geometry_msgs/Point position    # 상위 header.frame_id(base_link) 기준 [m]. x,y = 다리 중심의 지면 투영, z = bbox 안 점 높이 중앙값
float32 position_std            # 위치 1-sigma [m]
float32 depth_valid_ratio       # bbox 안 유효 depth 비율 0..1
float32[] appearance            # 외형 특징(정규화 색 히스토그램 등). 비어 있을 수 있음
