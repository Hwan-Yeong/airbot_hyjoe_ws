# skix_a1_log_analysis.py 개발 완료 안내

요구하신 기능들을 바탕으로 PyQt5 기반의 로그 및 맵 분석 툴 개발을 완료하였습니다. 파일을 여러 모듈 형식으로 나누어 확장성과 유지보수성을 극대화하였습니다.

## 📂 파일 구조 (Architecture)

프로젝트는 `scripts/tools/` 디렉터리에 메인 실행 스크립트와 `log_analysis_core` 패키지로 구성되었습니다.
- `skix_a1_log_analysis.py`: 메인 엔트리 스크립트로 앱을 실행하는 역할
- `log_analysis_core/__init__.py`: 패키지 이니셜라이저
- `log_analysis_core/main_window.py`: 전체 GUI 레이아웃, 버튼 이벤트, 맵과 로그 데이터 간의 연동 및 렌더링(Export 기능 포함) 제어
- `log_analysis_core/map_manager.py`: 선택된 폴더에서 PGM 이미지, YAML(해상도 정보), 3종의 JSON 파일(금지 구역 Area, Wall, 충전기 위치)을 읽어와 시각화 객체(QGraphicsItem)로 변환
- `log_analysis_core/log_manager.py`: (최대 10개의) 로그 파일들을 한 번에 로드하여 병합/정렬한 후, 내장 타이머(QTimer)를 이용한 Play/Pause 등 재생 컨트롤을 담당
- `log_analysis_core/log_parser.py`: 제공해주신 로그 샘플과 일치하는 정규식(Regular Expression)들을 사용하여 로봇 포즈, Drop-off, 1D ToF 등을 판별하고 데이터 딕셔너리로 추출

---

## 🛠️ 주요 기능 소개 (Features)

### 1. Map Load 및 렌더링
`Map Load` 버튼을 눌러 폴더를 지정하면:
- **배경**: `airbot_map_00.pgm` 및 `.yaml` 정보를 토대로 Scale과 Origin을 맞춥니다.
- **오버레이**: `area.json` (빨간 반투명 다각형), `wall.json` (주황색 선), `station_pose.json` (초록색 원형 아이콘)을 오버레이합니다.
- 오른쪽 `Map Display Options`의 **체크박스**를 통해 각 항목별 즉각적인 표시/숨김 속성이 연동됩니다. (마우스 휠 스크롤 줌 기능 지원)

### 2. Log Data 파싱 (Multi-file Support & Regex Extraction)
`Log Load` 버튼을 눌러 `.txt` 파일들을 선택 (최대 10개)하면 다음 키워드 정규식에 의해 추출됩니다.
- `[2026-04-03 21:29:38.607] ... Current RobotPose [1](0.072, 0.092, 1.7(deg)) ...` -> Robot Pose (SLAM 기준)
- `... Detect drop off. robot_xy(-2.212, 0.791), drop(x:-2.745, y:0.949 ...` -> Drop-Off (x: -2.745, y: 0.949)
- `... 1D ToF detected. robot_xy(-2.228, 0.854), 1D(x:-2.729, y:0.997 ...` -> 1D ToF (x: -2.729, y: 0.997)

> [!NOTE]
> 추후 새로운 타입의 로그 정보 추가가 필요할 경우, 모듈 내 `log_parser.py`에 정규식 패턴을 하나 넣고 반환 값만 매핑하면 손쉽게 확장할 수 있게 코딩해두었습니다.

### 3. 체크박스 필터 및 재생 컨트롤 (Play/Pause)
- **로봇 및 장애물 마커 렌더링**: 로봇의 현재 위치(파란 원), Drop-off 센서 정보(자홍색 사각형), 1D ToF 센서 정보(노란 다이아몬드) 등을 독립 레이어에 표시합니다.
- `Log Display Options` **체크박스**를 통해 불필요한 데이터를 가릴 수 있습니다.
- 하단의 **툴바 컨트롤 (Play 버튼, 타임 슬라이더, Timestamp 표시)**이 QTimer와 동기화되어 시간축을 따라 재생 혹은 뒤로감기 갱신이 지원됩니다.

### 4. 뷰 캡처 (Export Image)
- **Export Frame** 버튼을 누르면 `QPixmap`을 통해 현재 캔버스에 표시된 맵, 장애물, 로봇, 금지구역 등의 혼합된 레이어를 하나로 래스터화하여 PNG 혹은 JPG 이미지 파일로 내보낼 수 있습니다. (보고서 작성 및 별도 분석용 이미지로 최적)

---

## 🚀 실행 안내 (How to run)

```bash
cd ~/namuhx_a1/airbot_hyjoe_ws/scripts/tools
./skix_a1_log_analysis.py
# (또는 python3 skix_a1_log_analysis.py)
```

> [!TIP]
> 동작을 위해 `PyQt5`와 `PyYAML`이 설치되어 있어야 합니다. (만약 설치되어 있지 않을 경우 `pip install PyQt5 pyyaml` 명령어를 사용해 주세요.)
> 추가적으로 로봇 방향(Yaw) 시각화 등은 구조가 잡혀있어 상황에 맞추어 `main_window.py`를 조금 튜닝하면 이쁘게 나옵니다!
