# cam_leg_detector 출력. 처리한 RGB 프레임마다 1개 (검출이 없어도 발행)
std_msgs/Header header          # stamp = RGB 영상 stamp, frame_id = base_link
uint32 image_width
uint32 image_height
kebi_follow_msgs/CamLeg[] detections
float32 inference_ms            # NPU 추론 시간
float32 total_ms                # 수신~발행 전체 처리 시간
