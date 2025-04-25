## tof_csv 사용법

- 형식:

    python3 ~/dev_ws/scripts/tof_csv.py <파일명> --left_row <추출행> --right_row <추출행>


- 예시:

    python3 ~/dev_ws/scripts/tof_csv.py test_1 --left_row 3,4 --right_row 3,4


## Tip

 - 내 컴퓨터 alias 설정

    alias tof_csv='function _tof_csv(){ python3 ~/dev_ws/scripts/tof_csv.py "$1" --left_row "$2" --right_row "$2"; }; _tof_csv'

 - 사용법:

    tof_csv <파일명> <추출행>

 - 예시:

    tof_csv test_1 3,4