import re
import os
import yaml
from datetime import datetime

class LogParser:
    def __init__(self, config_path=None):
        """
        YAML 설정 파일에서 레이어 정의를 로드하여 정규식을 동적으로 구성합니다.
        config_path가 None이면 기본 경로(config/log_layers.yaml)를 사용합니다.
        """
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), 'config', 'log_layers.yaml')
        
        self.layer_configs = []  # YAML에서 읽어온 레이어 설정 리스트
        self._compiled_layers = []  # (compiled_regex, layer_config) 튜플 리스트
        
        self._load_config(config_path)
        
        # ReturnToCharger는 UI 표시용 레이어가 아닌 시스템 이벤트이므로 코드에 직접 유지
        self.re_return = re.compile(
            r'\[(?P<time>[\d\-]+\s[\d:\.]+)\]\s+\[.*?\]\s+\[.*?\]\s+.*soc-cmd received : ReturnToCharger'
        )
        
        self.time_format = "%Y-%m-%d %H:%M:%S.%f"
        
    def _load_config(self, config_path):
        """YAML 설정 파일을 읽어 레이어 정규식을 컴파일합니다."""
        if not os.path.isfile(config_path):
            print(f"[LogParser] Warning: config file not found at {config_path}")
            return
            
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            
        if not data or 'layers' not in data:
            print("[LogParser] Warning: no 'layers' key found in config")
            return
            
        for layer_def in data['layers']:
            try:
                compiled = re.compile(layer_def['regex'])
                self.layer_configs.append(layer_def)
                self._compiled_layers.append((compiled, layer_def))
            except re.error as e:
                print(f"[LogParser] Warning: regex error in layer '{layer_def.get('id', '?')}': {e}")
    
    def _parse_time(self, time_str):
        try:
            dt = datetime.strptime(time_str, self.time_format)
            return dt.timestamp()
        except Exception:
            return None

    def parse_line(self, line):
        """
        한 줄의 로그를 파싱하여 알맞은 형태의 사전 데이터로 리턴합니다.
        YAML에 정의된 레이어 정규식을 순회하며 매칭을 시도합니다.
        매칭되는 것이 없으면 시스템 이벤트(ReturnToCharger)를 확인한 뒤 None을 리턴합니다.
        """
        # 1. YAML 정의 레이어 순회
        for compiled_re, layer_def in self._compiled_layers:
            match = compiled_re.search(line)
            if match:
                ts = self._parse_time(match.group('time'))
                if ts is not None:
                    result = {
                        'timestamp': ts,
                        'type': layer_def['id'],
                    }
                    # x, y 좌표가 있으면 추출
                    try:
                        result['x'] = float(match.group('x'))
                        result['y'] = float(match.group('y'))
                    except (IndexError, AttributeError):
                        pass
                    # 정규식에 정의된 추가 named group 자동 추출 (yaw, class_id 등)
                    for gname in compiled_re.groupindex:
                        if gname not in ('time', 'x', 'y') and gname not in result:
                            try:
                                result[gname] = match.group(gname)
                            except (IndexError, AttributeError):
                                pass
                    return result
        
        # 2. 시스템 이벤트: ReturnToCharger (UI 레이어 아니므로 코드에 직접 유지)
        match = self.re_return.search(line)
        if match:
            ts = self._parse_time(match.group('time'))
            if ts is not None:
                return {
                    'timestamp': ts,
                    'type': 'return_to_charger'
                }
                
        return None
