from PyQt5.QtCore import QObject, pyqtSignal, QTimer
import os
from .log_parser import LogParser

class LogManager(QObject):
    # UI에 이벤트를 전달하는 시그널들 (virtual_time, real_time)
    time_updated = pyqtSignal(float, float)
    playback_status_changed = pyqtSignal(bool) # True면 playing
    
    # 시간 변경에 따라 새로운 이벤트들이 발생했음을 알림
    events_triggered = pyqtSignal(list) 
    
    # 추가된 기능용 시그널
    raw_log_updated = pyqtSignal(str)
    current_file_updated = pyqtSignal(str)
    
    # 마커(재부팅, 날짜변경) 기능용 시그널 (전체 라인 수 리스펜스용으로 두번째 인자 넘김)
    marks_updated = pyqtSignal(list, float)

    def __init__(self):
        super().__init__()
        self.parser = LogParser()
        self.events = [] # 모든 파싱된 로그 이벤트(시간 순 정렬)
        self.current_time = 0.0
        
        self.start_time = 0.0
        self.end_time = 0.0
        
        self.timer = QTimer()
        self.timer.timeout.connect(self._on_timer)
        self.is_playing = False
        
        self.playback_speed = 1.0
        self.play_direction = 1 # 1: forward, -1: backward
        self.timer_interval = 50 # ms, 20 hz 업데이트
        
        # 원본 데이터 저장소
        self.file_paths = []
        self.file_names = []
        self.raw_files = [] # 각 파일별 원본 라인 리스트의 리스트
        
        # 최적화를 위해 마지막으로 재생한 인덱스를 추적
        self.last_event_idx = -1 
        
    def load_files(self, file_paths):
        """
        여러 파일 경로를 받아 한 번에 파싱을 진행하고 하나의 events 리스트로 병합합니다.
        """
        self.file_paths = file_paths
        self.file_names = [os.path.basename(p) for p in file_paths]
        self.raw_files = []
        new_events = []
        
        self.events = []
        self.pause()
        
        total_lines = 0
        for i, filepath in enumerate(file_paths):
            with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
                lines = f.readlines()
                for j, line in enumerate(lines):
                    event = self.parser.parse_line(line)
                    if event:
                        event['file_idx'] = i
                        event['line_idx'] = j
                        event['global_line_idx'] = total_lines + j
                        new_events.append(event)
                self.raw_files.append(lines)
                total_lines += len(lines)
                        
        # global_line_idx 기준으로 쓰여진 순서대로 재생
        new_events.sort(key=lambda x: x['global_line_idx'])
        
        import bisect
        import datetime
        
        marks = []
        if new_events:
            self.events = new_events
            self.glines = [e['global_line_idx'] for e in self.events]
            self.start_time = 0.0 # start_time 변수명 유지 (실제로는 line index)
            self.end_time = float(total_lines) # end_time 도 line index
            self.last_event_idx = 0
            
            # 마커 감지(재부팅 및 날짜 변경)
            first_ts = self.events[0]['timestamp']
            max_ts_seen = first_ts
            current_day = datetime.datetime.fromtimestamp(first_ts).date()
            
            for ev in self.events:
                ts = ev['timestamp']
                dt = datetime.datetime.fromtimestamp(ts)
                day = dt.date()
                
                # 5초 이상 과거로 돌아가면 재부팅으로 간주
                if ts < max_ts_seen - 5.0:
                    marks.append({
                        'line_idx': ev['global_line_idx'],
                        'text': "reboot",
                        'color': 'red'
                    })
                    max_ts_seen = ts
                    current_day = day # 새롭게 시작된 타임라인의 날짜로 갱신
                else:
                    if ts > max_ts_seen:
                        max_ts_seen = ts
                    
                    if day > current_day:
                        marks.append({
                            'line_idx': ev['global_line_idx'],
                            'text': dt.strftime("%m/%d"),
                            'color': 'blue'
                        })
                        current_day = day
                        
            self.marks_updated.emit(marks, self.end_time)
            self.set_current_time(self.start_time, is_jump=True)
            return True
            
        self.marks_updated.emit([], 1.0)
        return False
        
    def set_current_time(self, time_val, is_jump=False):
        # time_val은 실제로는 global_line_idx (float 형식)
        self.current_time = max(self.start_time, min(time_val, self.end_time))
        
        if not self.events:
            return
            
        import bisect
        idx = bisect.bisect_right(self.glines, self.current_time)
        old_idx = getattr(self, 'last_event_idx', 0)
        
        if is_jump or idx < old_idx:
            triggered = self.events[:idx]
        else:
            triggered = self.events[old_idx:idx]
            
        self.last_event_idx = idx
        
        latest_event = triggered[-1] if triggered else (self.events[idx-1] if idx > 0 else self.events[0])
        
        real_time = latest_event['timestamp']
        self.time_updated.emit(self.current_time, real_time)
        self.events_triggered.emit(triggered)
        
        self._update_raw_view(latest_event)
        
    def get_time_string_at_line(self, line_idx):
        if not hasattr(self, 'glines') or not self.events:
            return ""
        import bisect
        import datetime
        idx = bisect.bisect_right(self.glines, line_idx)
        if idx == 0:
            ts = self.events[0]['timestamp']
        else:
            ts = self.events[idx-1]['timestamp']
        return datetime.datetime.fromtimestamp(ts).strftime("%m/%d %H:%M:%S")
        
        if self.current_time >= self.end_time and self.is_playing and self.play_direction == 1:
            self.pause()
            
    def _update_raw_view(self, event):
        file_idx = event.get('file_idx')
        line_idx = event.get('line_idx')
        
        if file_idx is not None and line_idx is not None and file_idx < len(self.raw_files):
            file_name = self.file_names[file_idx]
            self.current_file_updated.emit(file_name)
            
            lines = self.raw_files[file_idx]
            start_idx = max(0, line_idx - 10)
            end_idx = min(len(lines), line_idx + 11)
            
            raw_text_parts = []
            for i in range(start_idx, end_idx):
                prefix = ">> " if i == line_idx else "   "
                raw_text_parts.append(f"{prefix}{lines[i].rstrip()}")
                
            self.raw_log_updated.emit('\n'.join(raw_text_parts))

    def play_forward(self):
        if not self.events:
            return
        if self.current_time >= self.end_time:
            self.set_current_time(self.start_time)
        self.play_direction = 1
        self.is_playing = True
        self.playback_status_changed.emit(True)
        self.timer.start(self.timer_interval)

    def play_backward(self):
        if not self.events:
            return
        if self.current_time <= self.start_time:
            self.set_current_time(self.end_time)
        self.play_direction = -1
        self.is_playing = True
        self.playback_status_changed.emit(True)
        self.timer.start(self.timer_interval)
        
    def pause(self):
        self.is_playing = False
        self.playback_status_changed.emit(False)
        self.timer.stop()
        
    def toggle_play(self):
        if self.is_playing and self.play_direction == 1:
            self.pause()
        else:
            self.play_forward()
            
    def set_playback_speed(self, speed):
        self.playback_speed = speed
            
    def _on_timer(self):
        if not self.is_playing:
            return
            
        # 1초에 약 100줄의 텍스트가 쓰여진다고 가정(20Hz 포즈 등 포함)
        # 배속에 따라 이동할 줄(line) 개수 결정
        base_lines_per_sec = 200.0  
        dt_lines = (self.timer_interval / 1000.0) * self.playback_speed * base_lines_per_sec * self.play_direction
        
        new_time = self.current_time + dt_lines
        
        if new_time >= self.end_time:
            new_time = self.end_time
            self.set_current_time(new_time)
            self.pause()
        elif new_time <= 0:
            new_time = 0
            self.set_current_time(new_time)
            self.pause()
        else:
            self.set_current_time(new_time)
