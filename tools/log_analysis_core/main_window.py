import os
import datetime
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QSlider, QFileDialog, QGraphicsView, QGraphicsScene,
                             QCheckBox, QGroupBox, QFormLayout, QLabel, QListWidget,
                             QAction, QToolBar, QSpinBox, QComboBox, QTextEdit, QAbstractItemView,
                             QSplitter, QDateTimeEdit, QToolTip, QDoubleSpinBox)
from PyQt5.QtCore import Qt, QDateTime
from PyQt5.QtGui import QPen, QBrush, QColor, QTransform, QPolygonF, QPainter, QPainterPath, QPixmap, QIcon
from PyQt5.QtCore import QPointF

from .map_manager import MapManager
from .log_manager import LogManager

# ============================================================
# Qt 색상 이름 → QColor 변환 매핑 테이블
# YAML 설정의 color 필드에 쓸 수 있는 색상 이름들입니다.
# 새 색상을 추가하려면 여기에 항목을 추가하세요.
# ============================================================
COLOR_MAP = {
    "blue": Qt.blue,
    "red": Qt.red,
    "green": Qt.green,
    "yellow": Qt.yellow,
    "magenta": Qt.magenta,
    "cyan": Qt.cyan,
    "white": Qt.white,
    "black": Qt.black,
    "darkblue": Qt.darkBlue,
    "darkred": Qt.darkRed,
    "darkgreen": Qt.darkGreen,
    "darkyellow": Qt.darkYellow,
    "darkmagenta": Qt.darkMagenta,
    "darkcyan": Qt.darkCyan,
    "purple": QColor(128, 0, 128),
    "indigo": QColor(75, 0, 130),
    "tomato": QColor(255, 99, 71),
    "yellowgreen": QColor(154, 205, 50),
    "deepskyblue": QColor(0, 191, 255),
    "orange": QColor(255, 165, 0),
    "pink": QColor(255, 105, 180),
    "purple": QColor(128, 0, 128),
    "lime": QColor(0, 255, 0),
    "brown": QColor(139, 69, 19),
}

class MarkedSlider(QSlider):
    def __init__(self, orientation, parent=None):
        super().__init__(orientation, parent)
        self.marks = []
        self.total_lines = 1.0
        # 글자가 표시될 수 있도록 기본 슬라이더 위젯의 높이를 약간 확보
        self.setMinimumHeight(40)
        self.setMouseTracking(True)
        self.tooltip_callback = None

    def set_tooltip_callback(self, cb):
        self.tooltip_callback = cb

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        if self.tooltip_callback and self.total_lines > 0:
            padding = 10
            usable_width = self.width() - padding * 2
            if usable_width > 0:
                x = event.x() - padding
                ratio = max(0.0, min(1.0, x / usable_width))
                line_idx = ratio * self.total_lines
                text = self.tooltip_callback(line_idx)
                if text:
                    QToolTip.showText(event.globalPos(), text, self)

    def set_marks(self, marks, total_lines):
        self.marks = marks
        self.total_lines = total_lines if total_lines > 0 else 1.0
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if not self.marks:
            return
            
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        font = painter.font()
        font.setPointSize(8)
        font.setBold(True)
        painter.setFont(font)
        
        padding = 10 # 기본 노브(Handle) 마진
        usable_width = self.width() - padding * 2
        
        for m in self.marks:
            ratio = m['line_idx'] / self.total_lines
            x = padding + ratio * usable_width
            
            color = QColor(Qt.red) if m['color'] == 'red' else QColor(Qt.darkBlue)
            painter.setPen(QPen(color, 2))
            
            # 슬라이더 중앙 groove 아래에서 위쪽으로 세로선을 확실히 긋도록 지정
            painter.drawLine(int(x), 15, int(x), self.height() - 5)
            
            # 선 색상과 맞춰서 글자를 표시
            painter.setPen(QPen(color))
            
            # 글자가 선 중앙에 오도록 텍스트 길이를 계산해서 위치 튜닝
            font_metrics = painter.fontMetrics()
            tw = font_metrics.width(m['text'])
            
            # 선 위쪽 공간 (y=12) 에 텍스트를 배치
            painter.drawText(int(x - tw/2), 12, m['text'])
            
        painter.end()

class LogAnalysisMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('SKIX A1 Log Analysis Tool')
        self.resize(1600, 1000) # 기본 윈도우 크기 확대
        
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.view.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse) # Zoom 중심
        
        self.map_manager = MapManager(self.scene)
        self.log_manager = LogManager()
        
        # ============================================================
        # 동적 레이어 상태 관리 딕셔너리
        # layer_id → { 'chk': QCheckBox, 'spin': QDoubleSpinBox,
        #              'items': [QGraphicsItem...], 'config': dict }
        # ============================================================
        self.layer_states = {}
        
        self.last_cleared_time = 0.0 # Clear 기능용
        
        self._setup_ui()
        self._connect_signals()
        
    def _setup_ui(self):
        # 중앙 위젯 구성
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # 메인 가로 분할기 (상하 분리)
        main_splitter = QSplitter(Qt.Vertical)
        
        # 상단 세로 분할기 (좌 Map, 우 Controls)
        top_splitter = QSplitter(Qt.Horizontal)
        
        # 왼쪽 창: 맵 뷰
        top_splitter.addWidget(self.view)
        
        # 오른쪽 창: 설정 패널 및 파일 리스트
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(5, 5, 5, 5)
        top_splitter.addWidget(right_panel)
        top_splitter.setStretchFactor(0, 4)
        top_splitter.setStretchFactor(1, 1)
        top_splitter.setSizes([1200, 400])
        
        main_splitter.addWidget(top_splitter)
        
        # 하단 창: Raw 로그 뷰어 존
        bottom_panel = QWidget()
        bottom_layout = QVBoxLayout(bottom_panel)
        bottom_layout.setContentsMargins(5, 5, 5, 5)
        
        bottom_layout.addWidget(QLabel("Raw Log Context:"))
        self.txt_raw_log = QTextEdit()
        self.txt_raw_log.setLineWrapMode(QTextEdit.NoWrap) # 한줄로 길게 보이게 (줄바꿈 방지)
        self.txt_raw_log.setReadOnly(True)
        self.txt_raw_log.setStyleSheet("font-family: monospace; font-size: 10pt; background-color: #f0f0f0;")
        bottom_layout.addWidget(self.txt_raw_log)
        
        # -- Clear Button
        btn_clear = QPushButton("Clear Trajectory & Obstacles")
        btn_clear.clicked.connect(self._on_clear_clicked)
        bottom_layout.addWidget(btn_clear)
        
        main_splitter.addWidget(bottom_panel)
        main_splitter.setStretchFactor(0, 2)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([700, 300])
        
        main_layout.addWidget(main_splitter)
        
        # -- 버튼류
        btn_layout = QHBoxLayout()
        btn_map = QPushButton("Map Load")
        btn_map.clicked.connect(self._open_map_dialog)
        btn_log = QPushButton("Log Load")
        btn_log.clicked.connect(self._open_log_dialog)
        btn_export = QPushButton("Export Frame")
        btn_export.clicked.connect(self._export_view)
        
        btn_layout.addWidget(btn_map)
        btn_layout.addWidget(btn_log)
        btn_layout.addWidget(btn_export)
        right_layout.addLayout(btn_layout)
        
        # -- Map Display Options (Area, Wall, Station은 YAML 레이어가 아닌 맵 고유 요소)
        group_map = QGroupBox("Map Display Options")
        map_layout = QVBoxLayout()
        self.chk_area = QCheckBox("Area")
        self.chk_area.setChecked(True)
        self.chk_area.toggled.connect(lambda c: self.map_manager.set_visibility("area", c))
        
        self.chk_wall = QCheckBox("Wall")
        self.chk_wall.setChecked(True)
        self.chk_wall.toggled.connect(lambda c: self.map_manager.set_visibility("wall", c))
        
        map_layout.addWidget(self.chk_area)
        map_layout.addWidget(self.chk_wall)
        
        self.chk_station, self.spin_station = self._add_layer_control(
            map_layout, "Station", "station", QColor(0, 255, 0), 3.0,
            lambda c: self.map_manager.set_visibility("station", c),
            self._on_station_size_changed
        )
        
        group_map.setLayout(map_layout)
        right_layout.addWidget(group_map)
        
        # -- Log Display Options (YAML 기반 동적 생성)
        group_log = QGroupBox("Log Display Options")
        self.log_display_layout = QVBoxLayout()
        
        self._build_dynamic_layer_ui(self.log_display_layout)
        
        group_log.setLayout(self.log_display_layout)
        right_layout.addWidget(group_log)
        
        # -- 로그 리스트
        self.log_list_widget = QListWidget()
        self.log_list_widget.setDragDropMode(QAbstractItemView.InternalMove)
        self.log_list_widget.model().rowsMoved.connect(self._on_log_reordered)
        self.log_list_widget.itemDoubleClicked.connect(self._on_log_double_clicked)
        
        # 파일 표시 라벨
        self.lbl_current_file = QLabel("Current Log: None")
        self.lbl_current_file.setStyleSheet("font-weight: bold; color: blue;")
        
        right_layout.addWidget(QLabel("Loaded Logs (Drag to reorder):"))
        right_layout.addWidget(self.log_list_widget)
        right_layout.addWidget(self.lbl_current_file)
        
        # -- Robot Trace Limit
        trace_layout = QHBoxLayout()
        trace_layout.addWidget(QLabel("Robot Trace Length:"))
        self.spin_trace = QSpinBox()
        self.spin_trace.setRange(1, 100)
        self.spin_trace.setValue(50) # 기본 50개
        self.spin_trace.valueChanged.connect(self._on_trace_limit_changed)
        trace_layout.addWidget(self.spin_trace)
        right_layout.addLayout(trace_layout)
        
        # 하단 툴바: 컨트롤 패널
        toolbar_panel = QWidget()
        toolbar_layout = QHBoxLayout(toolbar_panel)
        
        self.btn_play_bwd = QPushButton("◀ Play Backward")
        self.btn_play_bwd.clicked.connect(self.log_manager.play_backward)
        
        self.btn_play_fwd = QPushButton("Play Forward ▶")
        self.btn_play_fwd.clicked.connect(self.log_manager.toggle_play)
        
        self.btn_pause = QPushButton("Pause ⏸")
        self.btn_pause.clicked.connect(self.log_manager.pause)
        
        self.combo_speed = QComboBox()
        self.combo_speed.addItems(["1.0x", "2.5x", "5.0x", "10.0x", "25.0x", "50.0x", "100.0x"])
        self.combo_speed.setCurrentText("1.0x")
        self.combo_speed.currentTextChanged.connect(self._on_speed_changed)
        self.log_manager.set_playback_speed(1.0)
        
        # 세세한 재생을 위해 Slider 해상도를 100,000으로 증가. 일반 QSlider 대신 MarkedSlider 사용
        self.slider = MarkedSlider(Qt.Horizontal)
        self.slider.setRange(0, 100000)
        self.slider.sliderPressed.connect(self.log_manager.pause)
        self.slider.valueChanged.connect(self._on_slider_moved)
        self.slider.set_tooltip_callback(self.log_manager.get_time_string_at_line)
        
        self.lbl_time = QLabel("00:00:00 / 00:00:00")
        
        toolbar_layout.addWidget(self.btn_play_bwd)
        toolbar_layout.addWidget(self.btn_play_fwd)
        toolbar_layout.addWidget(self.btn_pause)
        toolbar_layout.addWidget(self.combo_speed)
        toolbar_layout.addWidget(self.slider)
        toolbar_layout.addWidget(self.lbl_time)
        
        # 시간 직접 입력/점프 기능
        jump_layout = QHBoxLayout()
        jump_layout.addStretch()
        jump_layout.addWidget(QLabel("Jump to:"))
        self.dt_jump = QDateTimeEdit()
        self.dt_jump.setDisplayFormat("yyyy-MM-dd HH:mm:ss.zzz")
        self.dt_jump.setCalendarPopup(False)
        btn_jump = QPushButton("Go")
        btn_jump.clicked.connect(self._on_jump_clicked)
        jump_layout.addWidget(self.dt_jump)
        jump_layout.addWidget(btn_jump)
        
        # 메인 윈도우 하단에 도킹
        dock = QToolBar()
        dock.setMovable(False)
        
        controls_wrapper = QWidget()
        cw_layout = QVBoxLayout(controls_wrapper)
        cw_layout.setContentsMargins(0, 0, 0, 0)
        cw_layout.addLayout(jump_layout)
        cw_layout.addWidget(toolbar_panel)
        
        dock.addWidget(controls_wrapper)
        self.addToolBar(Qt.BottomToolBarArea, dock)
    
    # ============================================================
    # YAML 기반 동적 UI 빌드
    # ============================================================
    def _build_dynamic_layer_ui(self, layout):
        """LogParser의 YAML 설정에 따라 체크박스 + 사이즈 스핀박스를 자동으로 생성합니다."""
        for layer_cfg in self.log_manager.parser.layer_configs:
            layer_id = layer_cfg['id']
            color_name = layer_cfg.get('color', 'blue')
            qt_color = COLOR_MAP.get(color_name.lower(), Qt.gray)
            shape = layer_cfg.get('shape', 'circle')
            default_size = layer_cfg.get('default_size', 4.0)
            menu_name = layer_cfg.get('menu_name', layer_id)
            
            chk, spin = self._add_layer_control(
                layout, menu_name, shape, qt_color, default_size,
                self._update_log_visibility
            )
            
            self.layer_states[layer_id] = {
                'chk': chk,
                'spin': spin,
                'items': [],
                'config': layer_cfg,
            }
        
    def _connect_signals(self):
        self.log_manager.time_updated.connect(self._on_time_updated)
        self.log_manager.events_triggered.connect(self._on_events_triggered)
        self.log_manager.playback_status_changed.connect(self._on_play_status_changed)
        self.log_manager.current_file_updated.connect(self.lbl_current_file.setText)
        self.log_manager.raw_log_updated.connect(self._on_raw_log_updated)
        self.log_manager.marks_updated.connect(self.slider.set_marks)
        
    def _add_layer_control(self, layout, text, icon_shape, icon_color, default_size, toggle_cb, size_cb=None):
        row = QHBoxLayout()
        chk = QCheckBox(text)
        chk.setIcon(self._create_icon(icon_color, icon_shape))
        chk.setChecked(True)
        chk.toggled.connect(toggle_cb)
        
        spin = QDoubleSpinBox()
        spin.setRange(0.5, 100.0)
        spin.setSingleStep(0.5)
        spin.setValue(default_size)
        
        if size_cb:
            spin.valueChanged.connect(size_cb)
        else:
            spin.valueChanged.connect(self._on_layer_size_changed)
            
        row.addWidget(chk)
        row.addStretch()
        row.addWidget(QLabel("Size:"))
        row.addWidget(spin)
        
        layout.addLayout(row)
        return chk, spin

    def _on_layer_size_changed(self, val):
        self._clear_dynamic_layers()
        self.log_manager.set_current_time(self.log_manager.current_time, is_jump=True)
        
    def _on_station_size_changed(self, val):
        self.map_manager.redraw_station(val)
        
    # -- 줌 기능
    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.view.scale(1.2, 1.2)
        else:
            self.view.scale(1/1.2, 1/1.2)

    # -- 다이얼로그
    def _open_map_dialog(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Map Folder")
        if folder:
            if self.map_manager.load_map_folder(folder):
                self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
                print(f"Loaded Map from {folder}")
            else:
                print("Failed to load map. Check if files exist.")
                
    def _open_log_dialog(self):
        files, _ = QFileDialog.getOpenFileNames(self, "Select Log Files", filter="Text files (*.txt);;All files (*.*)")
        if files:
            if len(files) > 10:
                print("Warning: Only up to 10 files allowed. Slicing.")
                files = files[:10]
            self.loaded_file_paths = files # 원래 전체 경로를 보존하기 위해 저장
            if self.log_manager.load_files(self.loaded_file_paths):
                self.log_list_widget.clear()
                for f in self.loaded_file_paths:
                    self.log_list_widget.addItem(os.path.basename(f))
                self.slider.setValue(0)
                self.last_cleared_time = 0.0
                
                # Jump용 기준 시간 셋업
                st = datetime.datetime.fromtimestamp(self.log_manager.start_time)
                self.dt_jump.setDateTime(QDateTime(st.year, st.month, st.day, st.hour, st.minute, st.second, int(st.microsecond/1000)))
                
                self._clear_dynamic_layers()
                print("Logs Loaded successfully.")
            
    # -- 컨트롤
    def _on_log_double_clicked(self, item):
        # 파일명을 기반으로 가장 처음 등장하는 이벤트를 찾아 virtual time 점프
        target_name = item.text()
        for ev in self.log_manager.events:
            file_name = self.log_manager.file_names[ev['file_idx']]
            if file_name == target_name:
                self._clear_dynamic_layers()
                self.log_manager.set_current_time(ev['global_line_idx'], is_jump=True)
                break
                
    def _on_log_reordered(self, parent, start, end, destination, row):
        # UI에서 순서가 바뀌었을 때 실제 로딩된 파일 경로 리스트 재배치
        if not hasattr(self, 'loaded_file_paths'):
            return
            
        was_playing = self.log_manager.is_playing
        self.log_manager.pause()
        
        # 새 순서에 맞게 loaded_file_paths 갱신
        new_paths = []
        for i in range(self.log_list_widget.count()):
            item_text = self.log_list_widget.item(i).text()
            # 이름으로 원래 path 찾기 (이름 중복이 없다고 가정)
            for p in self.loaded_file_paths:
                if os.path.basename(p) == item_text:
                    new_paths.append(p)
                    break
                    
        self.loaded_file_paths = new_paths
        last_time = self.log_manager.current_time
        
        if self.log_manager.load_files(self.loaded_file_paths):
            self._clear_dynamic_layers()
            # 시간 보정 (새로운 파일 구성에 따라 타임라인이 달라졌을 수 있음)
            self.log_manager.set_current_time(last_time, is_jump=True)
            if was_playing:
                self.log_manager.play_forward()
                
    def _on_clear_clicked(self):
        self.last_cleared_time = self.log_manager.current_time
        self._clear_dynamic_layers()
    def _on_speed_changed(self, text):
        speed = float(text.replace("x", ""))
        self.log_manager.set_playback_speed(speed)
        
    def _on_trace_limit_changed(self, val):
        # 개수 변경 시 즉시 다시 랜더링 반영하기 위해 시간 강제 업데이트
        self.log_manager.set_current_time(self.log_manager.current_time, is_jump=True)
        
    def _on_jump_clicked(self):
        # 입력된 Real time을 기반으로 적절한 virtual time을 찾음
        js = self.dt_jump.dateTime().toPyDateTime().timestamp()
        
        # 가장 가까운 실제 시간을 찾아 그에 맞는 가상 시간을 설정
        closest_line = 0.0
        min_diff = float('inf')
        for ev in self.log_manager.events:
            diff = abs(ev['timestamp'] - js)
            if diff < min_diff:
                min_diff = diff
                closest_line = ev['global_line_idx']
                
        self._clear_dynamic_layers()
        self.log_manager.set_current_time(closest_line, is_jump=True)
            
    def _on_slider_moved(self, value):
        if self.slider.isSliderDown() and self.log_manager.end_time > 0:
            ratio = value / 100000.0
            t = self.log_manager.start_time + ratio * (self.log_manager.end_time - self.log_manager.start_time)
            self._clear_dynamic_layers()
            self.log_manager.set_current_time(t, is_jump=True)
            
    def _on_time_updated(self, current_time, real_time):
        if self.log_manager.end_time > self.log_manager.start_time:
            ratio = (current_time - self.log_manager.start_time) / (self.log_manager.end_time - self.log_manager.start_time)
            if not self.slider.isSliderDown():
                self.slider.blockSignals(True)
                self.slider.setValue(int(ratio * 100000))
                self.slider.blockSignals(False)
                
        ct = datetime.datetime.fromtimestamp(real_time)
        et = datetime.datetime.fromtimestamp(self.log_manager.events[-1]['timestamp'] if self.log_manager.events else 0)
        self.lbl_time.setText(f"{ct.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]} / {et.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
        
        if not self.dt_jump.hasFocus():
            self.dt_jump.blockSignals(True)
            self.dt_jump.setDateTime(QDateTime(ct.year, ct.month, ct.day, ct.hour, ct.minute, ct.second, int(ct.microsecond/1000)))
            self.dt_jump.blockSignals(False)
            
    def _on_raw_log_updated(self, text):
        self.txt_raw_log.setPlainText(text)
        
        # >> 가 표시된 곳으로 자동 스크롤 하도록 강제 이동
        doc = self.txt_raw_log.document()
        cursor = self.txt_raw_log.textCursor()
        cursor.setPosition(0)
        cursor = doc.find(">>", cursor)
        if not cursor.isNull():
            self.txt_raw_log.setTextCursor(cursor)
            self.txt_raw_log.ensureCursorVisible()
        
    def _on_play_status_changed(self, playing):
        pass
    
    # ============================================================
    # 이벤트 수신 및 범용 렌더링 엔진
    # ============================================================
    def _on_events_triggered(self, events):
        """로그 매니저로부터 누적 수신된 이벤트들을 범용 렌더러로 처리합니다."""
        limit = self.spin_trace.value()
        
        valid_events = [ev for ev in events if ev['global_line_idx'] > self.last_cleared_time]
        
        # is_trace=true 인 레이어(robot_pose 등)는 궤적 제한 적용
        trace_ids = set()
        for lid, state in self.layer_states.items():
            if state['config'].get('is_trace', False):
                trace_ids.add(lid)
        
        trace_events = [ev for ev in valid_events if ev['type'] in trace_ids]
        other_events = [ev for ev in valid_events if ev['type'] not in trace_ids and ev['type'] != 'return_to_charger']
        system_events = [ev for ev in valid_events if ev['type'] == 'return_to_charger']
        
        if len(trace_events) > limit:
            trace_events = trace_events[-limit:]
            
        for ev in other_events + trace_events:
            layer_id = ev['type']
            
            if layer_id in self.layer_states:
                # target_pose의 경우: 가장 최근 목적지만 빨간색으로 표기하는 특수 로직 유지
                if layer_id == 'target_pose':
                    self._add_target_special(ev)
                else:
                    self._render_generic_item(ev)
                    
        # 시스템 이벤트 처리 (ReturnToCharger)
        for ev in system_events:
            self._handle_return_to_charger()
    
    def _render_generic_item(self, ev):
        """YAML 설정에 따라 범용적으로 도형을 그리는 렌더러입니다."""
        layer_id = ev['type']
        state = self.layer_states[layer_id]
        cfg = state['config']
        
        px, py = self.map_manager.to_scene_coords(ev['x'], ev['y'])
        size = state['spin'].value()
        
        color_name = cfg.get('color', 'blue')
        qt_color = COLOR_MAP.get(color_name.lower(), Qt.gray)
        z_val = cfg.get('z_value', 50)
        shape = cfg.get('shape', 'circle')
        is_trace = cfg.get('is_trace', False)
        
        item = None
        if shape == 'circle':
            rad = size
            item = self.scene.addEllipse(px - rad, py - rad, rad*2, rad*2, QPen(Qt.black), QBrush(qt_color))
        elif shape == 'rect':
            item = self.scene.addRect(px - size/2, py - size/2, size, size, QPen(Qt.black), QBrush(qt_color))
        elif shape == 'diamond':
            polygon = QPolygonF([
                QPointF(px, py - size/2),
                QPointF(px + size/2, py),
                QPointF(px, py + size/2),
                QPointF(px - size/2, py)
            ])
            item = self.scene.addPolygon(polygon, QPen(Qt.black), QBrush(qt_color))
        elif shape == 'cross':
            path = QPainterPath()
            path.moveTo(px - size/2, py - size/2)
            path.lineTo(px + size/2, py + size/2)
            path.moveTo(px + size/2, py - size/2)
            path.lineTo(px - size/2, py + size/2)
            item = self.scene.addPath(path, QPen(qt_color, max(2, size/4)))
            
        if item:
            item.setZValue(z_val)
            item.setVisible(state['chk'].isChecked())
            state['items'].append(item)
            
            # Trace 제한 적용
            if is_trace:
                limit = self.spin_trace.value()
                while len(state['items']) > limit:
                    old = state['items'].pop(0)
                    self.scene.removeItem(old)
    
    def _add_target_special(self, ev):
        """target_pose 전용 특수 렌더링 (최신 목적지만 빨간색, 이전은 초록색)."""
        state = self.layer_states['target_pose']
        
        px, py = self.map_manager.to_scene_coords(ev['x'], ev['y'])
        size = state['spin'].value()
        
        path = QPainterPath()
        path.moveTo(px - size/2, py - size/2)
        path.lineTo(px + size/2, py + size/2)
        path.moveTo(px + size/2, py - size/2)
        path.lineTo(px - size/2, py + size/2)
        
        # 이전 최신 목적지를 초록색으로 복귀
        if state['items']:
            state['items'][-1].setPen(QPen(Qt.green, 3))
            state['items'][-1].setZValue(59)
            
        item = self.scene.addPath(path, QPen(Qt.red, 4))
        item.setZValue(60)
        item.setVisible(state['chk'].isChecked())
        state['items'].append(item)
        
        # 다른 목적지가 세팅되었으니 스테이션은 초록색으로 복귀
        if hasattr(self.map_manager, 'station_item') and self.map_manager.station_item:
            self.map_manager.station_item.setBrush(QBrush(QColor(0, 255, 0, 200)))
    
    def _handle_return_to_charger(self):
        """ReturnToCharger 시스템 이벤트: 스테이션을 빨간색으로 변경."""
        # target_pose 레이어가 있으면 마지막 목적지를 초록색으로
        if 'target_pose' in self.layer_states:
            items = self.layer_states['target_pose']['items']
            if items:
                items[-1].setPen(QPen(Qt.green, 3))
                items[-1].setZValue(59)    
        # 충전기 위치(Station)를 빨간색으로 변경
        if hasattr(self.map_manager, 'station_item') and self.map_manager.station_item:
            self.map_manager.station_item.setBrush(QBrush(QColor(255, 0, 0, 200)))
    
    # ============================================================
    # 동적 레이어 초기화/가시성
    # ============================================================
    def _clear_dynamic_layers(self):
        """모든 동적 레이어의 아이템을 Scene에서 제거합니다."""
        for lid, state in self.layer_states.items():
            for item in state['items']:
                self.scene.removeItem(item)
            state['items'].clear()
        
        if hasattr(self.map_manager, 'station_item') and self.map_manager.station_item:
            self.map_manager.station_item.setBrush(QBrush(QColor(0, 255, 0, 200)))

    def _update_log_visibility(self):
        """모든 동적 레이어의 가시성을 체크박스 상태와 동기화합니다."""
        for lid, state in self.layer_states.items():
            visible = state['chk'].isChecked()
            for it in state['items']:
                it.setVisible(visible)
            
    # -- Export
    def _export_view(self):
        """QGraphicsScene의 현재 표시 영역을 파일로 Export 합니다."""
        path, _ = QFileDialog.getSaveFileName(self, "Save Scene as Image", "map_screenshot.png", "PNG (*.png);;JPG (*.jpg)")
        if not path:
            return
            
        self.view.scene().clearSelection()
        
        rect = self.scene.sceneRect()
        pixmap = QPixmap(int(rect.width()), int(rect.height()))
        pixmap.fill(Qt.transparent)
        
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        self.scene.render(painter)
        painter.end()
        
        pixmap.save(path)
        print(f"Exported view to {path}")
        
    def _create_icon(self, color, shape="rect"):
        pixmap = QPixmap(16, 16)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        
        if shape == "cross":
            painter.setPen(QPen(color, 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawLine(2, 2, 14, 14)
            painter.drawLine(14, 2, 2, 14)
        elif shape == "station":
            painter.setPen(QPen(Qt.black))
            painter.setBrush(QBrush(color))
            painter.drawRect(2, 4, 8, 8)
            poly = QPolygonF([QPointF(10, 2), QPointF(15, 8), QPointF(10, 14)])
            painter.setBrush(QBrush(Qt.yellow))
            painter.drawPolygon(poly)
        else:
            painter.setPen(QPen(Qt.black))
            painter.setBrush(QBrush(color))
            if shape == "rect":
                painter.drawRect(2, 2, 12, 12)
            elif shape == "circle":
                painter.drawEllipse(2, 2, 12, 12)
            elif shape == "diamond":
                polygon = QPolygonF([QPointF(8, 2), QPointF(14, 8), QPointF(8, 14), QPointF(2, 8)])
                painter.drawPolygon(polygon)
                
        painter.end()
        return QIcon(pixmap)
