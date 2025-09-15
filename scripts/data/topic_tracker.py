#!/usr/bin/env python3
import subprocess
import csv
import json
import os

# ================  저장 경로 설정 부분 ================
SAVE_DIR = "/home/hyjoe/airbot_hyjoe_ws/csv/A1_SW_topic_info"
CSV_FILE = os.path.join(SAVE_DIR, "ros2_topics.csv")
JSON_FILE = os.path.join(SAVE_DIR, "ros2_topics.json")
# ==================================================

def run_cmd(cmd):
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return result.stdout.strip()

def get_topic_info(topic):
    info_raw = run_cmd(f"ros2 topic info {topic} -v")
    lines = info_raw.splitlines()

    data = {
        "topic": topic,
        "type": None,
        "publishers": [],
        "subscribers": []
    }

    section = None
    for line in lines:
        line = line.strip()
        if line.startswith("Type:"):
            data["type"] = line.split("Type:")[1].strip()
        elif line.startswith("Publisher count:"):
            section = "publishers"
        elif line.startswith("Subscriber count:"):
            section = "subscribers"
        elif line.startswith("*"):
            node = line.split("*")[1].strip()
            data[section].append(node)
    return data

def main():
    # 저장 경로 없으면 생성
    os.makedirs(SAVE_DIR, exist_ok=True)

    # 1. 토픽 리스트 가져오기
    topics = run_cmd("ros2 topic list").splitlines()

    all_data = []
    for topic in topics:
        info = get_topic_info(topic)
        all_data.append(info)

    # 2. CSV 저장
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Topic", "Type", "Publishers", "Subscribers"])
        for d in all_data:
            writer.writerow([
                d["topic"], 
                d["type"], 
                "; ".join(d["publishers"]),  # 여러 개일 경우 ; 로 구분
                "; ".join(d["subscribers"])
            ])

    # 3. JSON 저장
    with open(JSON_FILE, "w") as f:
        json.dump(all_data, f, indent=2, ensure_ascii=False)

    print(f"✅ Saved to:\n- {CSV_FILE}\n- {JSON_FILE}")

if __name__ == "__main__":
    main()
