# spatio_temporal_voxel_layer 내부에 포함된 openvdb 라이브러리 경로를 LD_LIBRARY_PATH에 추가

# 이 스크립트가 위치한 패키지 설치 경로를 기준으로 상대 경로 설정
# install/spatio_temporal_voxel_layer/share/spatio_temporal_voxel_layer/hook 에 위치하게 됨
# 쉘 스크립트에서 $AMENT_CURRENT_PREFIX 는 install/spatio_temporal_voxel_layer 를 가리킴

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$AMENT_CURRENT_PREFIX/opt/openvdb_vendor/lib"
