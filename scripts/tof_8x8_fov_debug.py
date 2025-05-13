import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Literal


class TPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"TPoint(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"


# ========== [1] 사용할 센서 인덱스 입력 ==========
left_idx = [
    1, 3, 5, 7,
    17, 19, 21, 23,
    34, 37, 42, 45,
    50, 53, 58, 61
]

right_idx = [
    0, 2, 4, 6,
    16, 18, 20, 22,
    34, 37, 42, 45,
    50, 53, 58, 61
]

# left_idx = [
#     1, 3, 5, 7,
#     17, 19, 21, 23,
#     33, 35, 37, 39,
#     49, 51, 53, 55
# ]

# right_idx = [
#     1, 3, 5, 7,
#     17, 19, 21, 23,
#     33, 35, 37, 39,
#     49, 51, 53, 55
# ]

tof_fov_angle_deg = 45  # FOV 각도
distances = [1.0] * 16  # 거리 고정 1m
robot_radius = 0.19  # 로봇 반지름

yaw = 15.0
pitch = -5.0
translation = TPoint(x=0.145, y=0.076, z=0.03)

# ========== [2] tan 값 계산 ==========
def update_sub_cell_index_array(sub_cell_index_array, tof_bot_fov_ang_deg):
    masked_mat = [[False for _ in range(8)] for _ in range(8)]

    for idx in sub_cell_index_array:
        if 0 <= idx < 64:
            row = idx // 8
            col = idx % 8
            masked_mat[row][col] = True

    true_indices = []
    for i in range(8):
        for j in range(8):
            if masked_mat[i][j]:
                true_indices.append((i, j))

    y_tan_out = []
    z_tan_out = []
    for i, j in true_indices:
        y = np.tan(tof_bot_fov_ang_deg * ((7 - 2 * j) / 16.0) * np.pi / 180)
        z = np.tan(tof_bot_fov_ang_deg * ((7 - 2 * i) / 16.0) * np.pi / 180)
        y_tan_out.append(y)
        z_tan_out.append(z)

    return y_tan_out, z_tan_out


# ========== [3] 센서 포인트 생성 ==========
def transform_tof_msg_to_points_on_sensor_frame(
    input_tof_dist: List[float],
    y_tan: List[float],
    z_tan: List[float],
    is_both_side: bool,
    side: Literal['LEFT', 'RIGHT']
) -> List[TPoint]:
    assert len(input_tof_dist) == 16
    assert len(y_tan) == 16
    assert len(z_tan) == 16

    zero_dist_index = [False] * 32
    points = []

    for index in range(16):
        dist = input_tof_dist[index]
        if not is_both_side:
            zero_dist_index[index] = dist <= 0.001
        else:
            zero_dist_index[index + 16] = dist <= 0.001

        if dist > 0.001:
            x = dist
            y = x * y_tan[index]
            z = x * z_tan[index]
            points.append(TPoint(x, y, z))

    return points


# ========== [4] 센서 좌표 -> 로봇 좌표계 변환 ==========
def transform_tof_sensor_to_robot_frame(
    input_points: List[TPoint],
    is_left: bool,
    rotation_yaw_deg: float,
    rotation_pitch_deg: float,
    translation: TPoint
) -> List[TPoint]:
    """
    센서 좌표계의 포인트들을 로봇 좌표계로 변환함.
    - input_points: 센서 좌표계 기준 TPoint 리스트
    - is_left: 좌측 센서인지 여부
    - rotation_yaw_deg: 센서의 Yaw 회전 (도)
    - rotation_pitch_deg: 센서의 Pitch 회전 (도)
    - translation: 센서 위치의 로봇 기준 이동 벡터
    """
    output_points = []

    yaw_rad = np.deg2rad(rotation_yaw_deg)
    pitch_rad = np.deg2rad(rotation_pitch_deg)

    cos_yaw = np.cos(yaw_rad)
    sin_yaw = np.sin(yaw_rad)
    cos_pitch = np.cos(pitch_rad)
    sin_pitch = np.sin(pitch_rad)

    for pt in input_points:
        # Yaw 회전 (z 축 회전)
        x_yaw = pt.x * cos_yaw - pt.y * sin_yaw
        y_yaw = pt.x * sin_yaw + pt.y * cos_yaw
        z_yaw = pt.z

        # Pitch 회전 (y 축 회전) + 위치 이동
        if is_left:
            x_robot = x_yaw * cos_pitch + z_yaw * sin_pitch + translation.x
            y_robot = y_yaw + translation.y
            z_robot = -x_yaw * sin_pitch + z_yaw * cos_pitch + translation.z
        else:
            x_robot = x_yaw * cos_pitch + z_yaw * sin_pitch + translation.x
            y_robot = y_yaw - translation.y
            z_robot = -x_yaw * sin_pitch + z_yaw * cos_pitch + translation.z

        output_points.append(TPoint(x_robot, y_robot, z_robot))

    return output_points


# ========== [5] +90도 회전 ==========
def rotate_points_90_deg(points: List[TPoint]):
    return [[-p.y, p.x] for p in points]

def rotate_origin_90_deg(p: TPoint):
    return [-p.y, p.x]

# ====== 박스 장애물 그리기 ======
def draw_box_in_robot_frame(
    distance_from_origin: float,
    angle_deg_robot_frame: float,
    width: float,
    height: float,
    alpha=0.5,
    label='Obstacle',
    color='gray'
):
    """
    박스 아랫변 중앙이 로봇 중심 기준 (distance, angle)에 있고,
    그 아랫변에 수직으로 박스가 위로 세워진 형태로 회전시킴.
    """
    theta = np.deg2rad(angle_deg_robot_frame)

    # (1) 아랫변 중앙 좌표 (로봇 좌표계 기준)
    base_cx = (distance_from_origin + height) * np.cos(theta)
    base_cy = (distance_from_origin + height) * np.sin(theta)

    # (2) 박스 좌하단 꼭짓점: 아랫변 중심에서
    # 왼쪽으로 width/2, 위쪽으로 height
    # local 기준에서 (-w/2, 0) → 좌하단, 방향은 박스 기준 수직 방향
    angle_box = theta + np.pi / 2  # 박스가 세워져 있으므로 +90도

    dx_local = -width / 2
    dy_local = 0

    # (3) local 좌표 회전 (박스 수직 방향 기준)
    dx_rot = dx_local * np.cos(angle_box) - dy_local * np.sin(angle_box)
    dy_rot = dx_local * np.sin(angle_box) + dy_local * np.cos(angle_box)

    # (4) 로봇 좌표계 기준 좌하단 꼭짓점
    corner_rx = base_cx + dx_rot
    corner_ry = base_cy + dy_rot

    # (5) +90도 시각화 좌표계 변환
    def rotate_90(x, y):
        return -y, x

    corner_x_plot, corner_y_plot = rotate_90(corner_rx, corner_ry)
    base_cx_plot, base_cy_plot = rotate_90(base_cx, base_cy)
    angle_plot = np.rad2deg(angle_box) + 90  # 시각화 기준 각도

    # (6) Rectangle 그리기
    rect = patches.Rectangle(
        (corner_x_plot, corner_y_plot),
        width,
        height,
        angle=angle_plot,
        linewidth=1.5,
        edgecolor='black',
        facecolor=color,
        alpha=alpha,
        label=label
    )
    plt.gca().add_patch(rect)



# ========== [6] 시각화 ==========
def visualize_fov(left_points, right_points, origin_left: TPoint, origin_right: TPoint, viz_left, viz_right):
    # 센서 기준 위치도 회전
    origin_left_xy = rotate_origin_90_deg(origin_left)
    origin_right_xy = rotate_origin_90_deg(origin_right)

    # 포인트들도 회전
    left_points_xy = rotate_points_90_deg(left_points)
    right_points_xy = rotate_points_90_deg(right_points)

    plt.figure(figsize=(8, 8))
    plt.axis('equal')
    plt.grid(True)
    plt.title("Sensor FOV Visualization (+90° Rotated)")

    # 로봇 중심점
    plt.plot(0, 0, 'ko', markersize=8, label="Robot")

    # 로봇 바디
    circle = plt.Circle((0, 0), robot_radius, color='gray', fill=False, linestyle='-', linewidth=1.5, label="R = 0.19m")
    plt.gca().add_patch(circle)

    # 장애물
    draw_box_in_robot_frame(
        distance_from_origin = 0.4,
        angle_deg_robot_frame = -15,
        width=0.6,
        height=0.2,
        label="Obstacle",
        color='gray'
    )
    draw_box_in_robot_frame(
        distance_from_origin = 0.4,
        angle_deg_robot_frame = 15,
        width=0.6,
        height=0.2,
        label="Obstacle",
        color='blue'
    )

    if viz_left :
        # 왼쪽 센서 FOV
        for p in left_points_xy:
            plt.plot([origin_left_xy[0], p[0]], [origin_left_xy[1], p[1]], 'b--', alpha=0.6)
            plt.plot(p[0], p[1], 'bo')

    if viz_right :
        # 오른쪽 센서 FOV
        for p in right_points_xy:
            plt.plot([origin_right_xy[0], p[0]], [origin_right_xy[1], p[1]], 'r--', alpha=0.6)
            plt.plot(p[0], p[1], 'ro')

    closest_point = find_closest_intersection(
        TPoint(0, 0),
        TPoint(origin_left_xy[0], origin_left_xy[1]),
        [TPoint(p[0], p[1]) for p in left_points_xy],
        TPoint(origin_right_xy[0], origin_right_xy[1]),
        [TPoint(p[0], p[1]) for p in right_points_xy],
    )

    if closest_point:
        dist = np.sqrt(closest_point.x**2 + closest_point.y**2)
        plt.plot(closest_point.x, closest_point.y, 'ro', markersize=10, color='green')
        plt.text(
            0.95, 0.05,
            f"Closest Intersection\nDist = {dist:.3f} m",
            transform=plt.gca().transAxes,
            fontsize=15,
            fontweight='bold',
            color='green',
            ha='right',
            va='bottom',
            bbox=dict(facecolor='white', edgecolor='red', boxstyle='round,pad=0.3')
        )

    plt.legend()
    plt.xlabel("X (Up)")
    plt.ylabel("Y (Left)")
    # plt.xlim(-2, 2)
    # plt.ylim(-2, 2)
    plt.show()


# ========== 교점찾기 ===========
def compute_intersection(p1, p2, p3, p4):
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    x3, y3 = p3.x, p3.y
    x4, y4 = p4.x, p4.y

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-6:
        return None  # Parallel or too close

    px = ((x1 * y2 - y1 * x2) * (x3 - x4) -
          (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) -
          (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
    return TPoint(px, py)

def find_closest_intersection(origin: TPoint, left_origin: TPoint, left_points: List[TPoint],
                              right_origin: TPoint, right_points: List[TPoint]) -> TPoint:
    intersections = []
    for lp in left_points:
        for rp in right_points:
            p = compute_intersection(left_origin, lp, right_origin, rp)
            if p:
                dist = np.sqrt(p.x**2 + p.y**2)
                if dist > robot_radius and p.y > 0.0:
                    intersections.append((dist, p))
    if not intersections:
        return None
    intersections.sort(key=lambda x: x[0])
    return intersections[0][1]  # closest TPoint

# ========== 실행 ==========

# LEFT
left_y_tan, left_z_tan = update_sub_cell_index_array(left_idx, tof_fov_angle_deg)
left_sensor_points = transform_tof_msg_to_points_on_sensor_frame(
    distances, left_y_tan, left_z_tan, is_both_side=False, side='LEFT'
)
left_robot_points = transform_tof_sensor_to_robot_frame(
    left_sensor_points,
    True,
    yaw,
    pitch,
    translation
)

# RIGHT
right_y_tan, right_z_tan = update_sub_cell_index_array(right_idx, tof_fov_angle_deg)
right_sensor_points = transform_tof_msg_to_points_on_sensor_frame(
    distances, right_y_tan, right_z_tan, is_both_side=False, side='RIGHT'
)
right_robot_points = transform_tof_sensor_to_robot_frame(
    right_sensor_points,
    False,
    -yaw,
    pitch,
    translation
)

# 시각화
# visualize_fov(left_sensor_points, right_sensor_points,
#               TPoint(x=0.0, y=0.0), TPoint(x=0.0, y=-0.0),
#               False, True
# )
visualize_fov(left_robot_points, right_robot_points,
              TPoint(x=0.145, y=0.076), TPoint(x=0.145, y=-0.076),
              True, True
)
