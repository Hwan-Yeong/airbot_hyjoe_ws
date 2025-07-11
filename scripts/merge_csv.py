import os
import re
import pandas as pd
from glob import glob

# ---------------------- 설정 (Config) ------------------------
CSV_FOLDER_PATH = "/home/hyjoe/airbot_hyjoe_ws/csv/tof/0711/tof"  # CSV 파일들이 있는 폴더 경로
OUTPUT_CSV_PATH = "/home/hyjoe/airbot_hyjoe_ws/csv/tof/0711/merged_result.csv"  # 저장할 결과 CSV 파일 경로
# -------------------------------------------------------------

def extract_sensor_number(filename):
    match = re.search(r'L(\d+)_R(\d+)', filename)
    if match:
        return int(match.group(1))  # L101_R101 → 101
    return None

def process_csv(file_path):
    filename = os.path.basename(file_path)
    sensor_num = extract_sensor_number(filename)
    if sensor_num is None:
        print(f"파일명에서 센서 번호를 추출할 수 없음: {filename}")
        return None

    try:
        df = pd.read_csv(file_path, sep='\t', nrows=4)
    except Exception as e:
        print(f"CSV 읽기 실패: {file_path}, 오류: {e}")
        return None

    # 예상된 컬럼이 없을 경우 스킵
    if 'timestamp' not in df.columns:
        print(f"[스킵] 'timestamp' 컬럼 없음: {filename}")
        return None

    # 왼쪽과 오른쪽 열 나누기
    left_columns = ['timestamp'] + [col for col in df.columns if col.startswith('Left')]
    right_columns = ['timestamp'] + [col for col in df.columns if col.startswith('Right')]

    # 필수 컬럼 확인
    if len(left_columns) <= 1 or len(right_columns) <= 1:
        print(f"[스킵] Left/Right 컬럼 부족: {filename}")
        return None

    left_df = df[left_columns].copy()
    right_df = df[right_columns].copy()

    # 각 데이터프레임에 번호와 라벨 컬럼 추가
    labels = ['MIN', 'MAX', 'AVG', 'STD', ''][:len(df)]  # 행 수가 4일 수도 있으니 슬라이스

    left_df.insert(0, "번호", sensor_num)
    left_df.insert(1, "측정값", labels)

    right_df.insert(0, "번호", sensor_num)
    right_df.insert(1, "측정값", labels)

    # 컬럼명 충돌 방지를 위해 컬럼명 변경
    # right_df.columns = ["R_" + col if col not in ["번호", "측정값"] else col for col in right_df.columns]
    right_df.columns = [col if col not in ["번호", "측정값"] else col for col in right_df.columns]

    # 좌우 합치기
    merged = pd.concat([left_df, right_df], axis=1)
    return merged

def merge_all_csvs():
    all_files = sorted(glob(os.path.join(CSV_FOLDER_PATH, "L*_R*.csv")))
    print(f"{len(all_files)}개의 파일을 처리합니다...")

    all_data = []

    for file in all_files:
        merged_df = process_csv(file)
        if merged_df is not None:
            all_data.append(merged_df)

    if all_data:
        result_df = pd.concat(all_data, ignore_index=True)
        result_df.to_csv(OUTPUT_CSV_PATH, index=False)
        print(f"통합 CSV 저장 완료: {OUTPUT_CSV_PATH}")
    else:
        print("병합할 데이터가 없습니다.")

if __name__ == "__main__":
    merge_all_csvs()
