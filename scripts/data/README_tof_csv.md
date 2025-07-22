## tof_csv 사용법

- 형식:

    python3 ~/airbot_hyjoe_ws/scripts/tof_csv.py <파일명> --duration <시간sec> --left_row <추출행> --right_row <추출행>


- 예시:

    python3 ~/airbot_hyjoe_ws/scripts/tof_csv.py test_1 --duration 5 --left_row 1,3,4 --right_row 6,7,8


## Tip

 - 내 컴퓨터 alias 설정

    alias tof_csv='function _tof_csv(){ python3 ~/airbot_hyjoe_ws/scripts/tof_csv.py "$1" --duration "$2 --left_row "$3" --right_row "$3"; }; _tof_csv'

 - 사용법:

    tof_csv <파일명> <추출시간[sec]> <추출행(8x8기준)>

 - 예시:

    tof_csv test_1 5 6,7,8