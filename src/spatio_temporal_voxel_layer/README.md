# Spatio-Temporal Voxel Layer (STVL)

이 패키지는 OpenVDB를 기반으로 하는 시공간 복셀 그리드 레이어입니다.

## 사전 준비 (External Binaries)

이 패키지는 빌드 속도를 높이고 이식성을 확보하기 위해 미리 빌드된 `openvdb_vendor` 바이너리를 사용합니다. 
Git 저장소를 처음 클론한 후, 빌드 전에 반드시 다음 스크립트를 실행하여 압축을 풀어주세요.

```bash
# 1. 패키지 디렉토리로 이동
cd src/spatio_temporal_voxel_layer/spatio_temporal_voxel_layer

# 2. 외부 라이브러리 압축 해제 스크립트 실행
chmod +x setup_external.sh
./setup_external.sh
```

## 빌드 및 설치

외부 라이브러리 준비가 완료되었다면, 일반적인 ROS 2 빌드 방식을 따릅니다.

```bash
cd ~/airbot_ws
colcon build --packages-select spatio_temporal_voxel_layer
source install/setup.bash
```

## 주요 특징
- **Portable**: `openvdb_vendor`를 매번 빌드할 필요 없이 미리 빌드된 바이너리를 사용합니다.
- **Auto Environment**: 빌드 후 `setup.bash`를 소싱하면 자동으로 라이브러리 경로가 설정됩니다.
