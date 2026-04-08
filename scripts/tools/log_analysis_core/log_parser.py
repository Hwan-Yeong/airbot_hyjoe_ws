import re
from datetime import datetime

class LogParser:
    def __init__(self):
        # 정규식 패턴 사전 정리
        
        # [2026-04-03 21:29:38.607] ... Current RobotPose [1](0.072, 0.092, 1.7(deg))
        self.re_robot_pose = re.compile(
            r'\[(?P<time>[\d\-]+\s[\d:\.]+)\]\s+\[.*?\]\s+\[.*?\]\s+Current RobotPose.*?\]\((?P<x>[-\d\.]+),\s*(?P<y>[-\d\.]+),\s*(?P<yaw>[-\d\.]+)\(deg\)\)'
        )
        
        # [2026-02-04 10:04:56.215] ... Detect drop off. robot_xy(-2.212, 0.791), drop(x:-2.745, y:0.949, dist: 0.556, diff:0.083)
        self.re_drop_off = re.compile(
            r'\[(?P<time>[\d\-]+\s[\d:\.]+)\]\s+\[.*?\]\s+\[.*?\]\s+Detect drop off.*drop\(x:(?P<x>[-\d\.]+),\s*y:(?P<y>[-\d\.]+)'
        )
        
        # [2026-04-03 22:00:53.472] ... 1D ToF detected. robot_xy(-2.228, 0.854), 1D(x:-2.729, y:0.997, Dist:0.521)
        self.re_1d_tof = re.compile(
            r'\[(?P<time>[\d\-]+\s[\d:\.]+)\]\s+\[.*?\]\s+\[.*?\]\s+1D ToF detected.*1D\(x:(?P<x>[-\d\.]+),\s*y:(?P<y>[-\d\.]+)'
        )
        
        self.time_format = "%Y-%m-%d %H:%M:%S.%f"
        
    def _parse_time(self, time_str):
        try:
            # datetime 객체로 변환하거나, 간단히 timestamp를 사용할 수 있습니다.
            # 하지만 여러 파일에서 순서를 매기려면 datetime 변환이 가장 좋습니다.
            dt = datetime.strptime(time_str, self.time_format)
            return dt.timestamp()
        except Exception:
            return None

    def parse_line(self, line):
        """
        한 줄의 로그를 파싱하여 알맞은 형태의 사전 데이터로 리턴합니다.
        매칭되는 것이 없으면 None을 리턴합니다.
        """
        # 1. 로봇 포즈 매칭 (제일 빈번하게 발생할 수 있음)
        match = self.re_robot_pose.search(line)
        if match:
            ts = self._parse_time(match.group('time'))
            if ts is not None:
                return {
                    'timestamp': ts,
                    'type': 'robot_pose',
                    'x': float(match.group('x')),
                    'y': float(match.group('y')),
                    'yaw': float(match.group('yaw'))
                }
                
        # 2. Drop off 매칭
        match = self.re_drop_off.search(line)
        if match:
            ts = self._parse_time(match.group('time'))
            if ts is not None:
                return {
                    'timestamp': ts,
                    'type': 'drop_off',
                    'x': float(match.group('x')),
                    'y': float(match.group('y'))
                }
                
        # 3. 1D ToF 매칭
        match = self.re_1d_tof.search(line)
        if match:
            ts = self._parse_time(match.group('time'))
            if ts is not None:
                return {
                    'timestamp': ts,
                    'type': '1d_tof',
                    'x': float(match.group('x')),
                    'y': float(match.group('y'))
                }
                
        return None
