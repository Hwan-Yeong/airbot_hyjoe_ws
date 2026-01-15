uint8 id        # The class ID
# [26.01.07] MR3 차 이후부터 confidencd score -> follow me 기능을 위한 tracked id 로 변경, score 파라미터 사용하지 말것!
uint8 score     # The confidence score (0~100)

float64 x
float64 y
float64 theta
float64 width
float64 height
float64 distance