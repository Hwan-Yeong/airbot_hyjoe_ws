import os
import datetime
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QSlider, QFileDialog, QGraphicsView, QGraphicsScene,
                             QCheckBox, QGroupBox, QFormLayout, QLabel, QListWidget,
                             QAction, QToolBar, QSpinBox, QComboBox, QTextEdit, QAbstractItemView,
                             QSplitter, QDateTimeEdit)
from PyQt5.QtCore import Qt, QDateTime
from PyQt5.QtGui import QPen, QBrush, QColor, QTransform, QPolygonF, QPainter
from PyQt5.QtCore import QPointF

from .map_manager import MapManager
from .log_manager import LogManager

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
        
        # 로그 렌더링에 사용할 변수들
        self.robot_path_items = [] # 궤적 아이템들 리스트
        
        # 장애물들을 개별 아이템으로 관리
        self.drop_off_items = []
        self.tof_items = []
        
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
        
        # -- 체크박스 그룹
        group_map = QGroupBox("Map Display Options")
        map_layout = QVBoxLayout()
        self.chk_area = QCheckBox("Area")
        self.chk_area.setChecked(True)
        self.chk_area.toggled.connect(lambda c: self.map_manager.set_visibility("area", c))
        
        self.chk_wall = QCheckBox("Wall")
        self.chk_wall.setChecked(True)
        self.chk_wall.toggled.connect(lambda c: self.map_manager.set_visibility("wall", c))
        
        self.chk_station = QCheckBox("Station")
        self.chk_station.setChecked(True)
        self.chk_station.toggled.connect(lambda c: self.map_manager.set_visibility("station", c))
        
        map_layout.addWidget(self.chk_area)
        map_layout.addWidget(self.chk_wall)
        map_layout.addWidget(self.chk_station)
        group_map.setLayout(map_layout)
        right_layout.addWidget(group_map)
        
        group_log = QGroupBox("Log Display Options")
        log_layout = QVBoxLayout()
        self.chk_robot = QCheckBox("Robot Pose")
        self.chk_robot.setChecked(True)
        self.chk_robot.toggled.connect(self._update_log_visibility)
        
        self.chk_drop_off = QCheckBox("Drop off Obstacles")
        self.chk_drop_off.setChecked(True)
        self.chk_drop_off.toggled.connect(self._update_log_visibility)
        
        self.chk_tof = QCheckBox("1D ToF Obstacles")
        self.chk_tof.setChecked(True)
        self.chk_tof.toggled.connect(self._update_log_visibility)
        
        log_layout.addWidget(self.chk_robot)
        log_layout.addWidget(self.chk_drop_off)
        log_layout.addWidget(self.chk_tof)
        group_log.setLayout(log_layout)
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
        self.combo_speed.addItems(["1.0x", "10.0x", "50.0x", "100.0x", "250.0x", "500.0x", "1000.0x"])
        self.combo_speed.setCurrentText("10.0x")
        self.combo_speed.currentTextChanged.connect(self._on_speed_changed)
        self.log_manager.set_playback_speed(10.0) # 기본값을 10x로
        
        # 세세한 재생을 위해 Slider 해상도를 100,000으로 증가
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 100000)
        self.slider.sliderPressed.connect(self.log_manager.pause)
        self.slider.valueChanged.connect(self._on_slider_moved)
        
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
        
    def _connect_signals(self):
        self.log_manager.time_updated.connect(self._on_time_updated)
        self.log_manager.events_triggered.connect(self._on_events_triggered)
        self.log_manager.playback_status_changed.connect(self._on_play_status_changed)
        self.log_manager.current_file_updated.connect(self.lbl_current_file.setText)
        self.log_manager.raw_log_updated.connect(self._on_raw_log_updated)
        
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
                self.log_manager.set_current_time(ev['global_line_idx'])
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
            self.log_manager.set_current_time(last_time)
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
        self.log_manager.set_current_time(self.log_manager.current_time)
        
    def _on_jump_clicked(self):
        # 입력된 Real time을 기반으로 적절한 virtual time을 찾음
        js = self.dt_jump.dateTime().toPyDateTime().timestamp()
        
        # 가장 가까운 실제 시간을 찾아 그에 맞는 가상 시간을 설정. 시간 역순이 없다는 보장이 사라졌으므로 직접 검색.
        closest_line = 0.0
        min_diff = float('inf')
        for ev in self.log_manager.events:
            diff = abs(ev['timestamp'] - js)
            if diff < min_diff:
                min_diff = diff
                closest_line = ev['global_line_idx']
                
        self._clear_dynamic_layers()
        self.log_manager.set_current_time(closest_line)
            
    def _on_slider_moved(self, value):
        # 실시간성: 마우스를 드래그하는 중에도 로봇 위치 업데이트
        # timer 등에 의한 업데이트와 무한 루프 방지 위해 판단
        if self.slider.isSliderDown() and self.log_manager.end_time > 0:
            ratio = value / 100000.0
            t = self.log_manager.start_time + ratio * (self.log_manager.end_time - self.log_manager.start_time)
            self._clear_dynamic_layers()
            self.log_manager.set_current_time(t)
            
    def _on_time_updated(self, current_time, real_time):
        if self.log_manager.end_time > self.log_manager.start_time:
            ratio = (current_time - self.log_manager.start_time) / (self.log_manager.end_time - self.log_manager.start_time)
            # 슬라이더가 눌려져 있지 않을 때만 업데이트
            if not self.slider.isSliderDown():
                self.slider.blockSignals(True)
                self.slider.setValue(int(ratio * 100000))
                self.slider.blockSignals(False)
                
        # 타임 스트링 포맷 연도-월-일 시간:분:초.000 로 수락 (real_time 기준)
        ct = datetime.datetime.fromtimestamp(real_time)
        et = datetime.datetime.fromtimestamp(self.log_manager.events[-1]['timestamp'] if self.log_manager.events else 0)
        self.lbl_time.setText(f"{ct.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]} / {et.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
        
        # Jump DateTime 값도 동기화 (단, 슬라이더 변경 시 너무 잦은 업데이트가 부담될 수 있으나 편의를 위해 시도)
        if not self.dt_jump.hasFocus(): # 사용자가 입력 중이 아닐 때만
            self.dt_jump.blockSignals(True)
            self.dt_jump.setDateTime(QDateTime(ct.year, ct.month, ct.day, ct.hour, ct.minute, ct.second, int(ct.microsecond/1000)))
            self.dt_jump.blockSignals(False)
            
    def _on_raw_log_updated(self, text):
        self.txt_raw_log.setPlainText(text)
        
        # >> 가 표시된 곳으로 자동 스크롤 하도록 강제 이동
        doc = self.txt_raw_log.document()
        cursor = self.txt_raw_log.textCursor()
        cursor.setPosition(0) # 처음부터 검색
        cursor = doc.find(">>", cursor)
        if not cursor.isNull():
            self.txt_raw_log.setTextCursor(cursor)
            self.txt_raw_log.ensureCursorVisible()
        
    def _on_play_status_changed(self, playing):
        # playing 상태에 따라 버튼 활성화/비활성화 처리 등 가능
        pass
        
    def _on_events_triggered(self, events):
        """
        로그 매니저로부터 누적 수신된 이벤트들.
        """
        # events 중 현재 타임라인데 그려져야 할 것만 필터링 (last_cleared_time 기준 + robot 궤적 제한 처리)
        # 로봇은 궤적 개수 제한때문에 따로 수집
        limit = self.spin_trace.value()
        
        valid_events = [ev for ev in events if ev['global_line_idx'] > self.last_cleared_time]
        
        robot_events = [ev for ev in valid_events if ev['type'] == 'robot_pose']
        other_events = [ev for ev in valid_events if ev['type'] != 'robot_pose']
        
        if len(robot_events) > limit:
            robot_events = robot_events[-limit:]
            
        for ev in other_events + robot_events:
            if ev['type'] == 'robot_pose':
                self._update_robot_pose(ev['x'], ev['y'], ev['yaw'])
            elif ev['type'] == 'drop_off':
                self._add_drop_off(ev['x'], ev['y'])
            elif ev['type'] == '1d_tof':
                self._add_1d_tof(ev['x'], ev['y'])
                
    # -- 렌더링
    def _clear_dynamic_layers(self):
        for item in self.robot_path_items:
            self.scene.removeItem(item)
        self.robot_path_items.clear()
        
        for item in self.drop_off_items:
            self.scene.removeItem(item)
        self.drop_off_items.clear()
        
        for item in self.tof_items:
            self.scene.removeItem(item)
        self.tof_items.clear()

    def _update_robot_pose(self, rx, ry, yaw):
        px, py = self.map_manager.to_scene_coords(rx, ry)
        
        # 로봇 크기 축소 (원래 8에서 1로 줄임)
        rad = 1.0 # (크기 설정하는 부분) 
        
        # 이전 좌표를 모두 표시하되, 가장 최근 것은 다른 색상으로 표시하는 등의 효과도 가능하지만
        # 일단 모두 파란 원으로 궤적을 그리도록 합니다.
        item = self.scene.addEllipse(px - rad, py - rad, rad*2, rad*2, QPen(Qt.black), QBrush(Qt.blue))
        item.setZValue(100)
        item.setVisible(self.chk_robot.isChecked())
        
        self.robot_path_items.append(item)
        
        # limit 처리 (실시간 재생 시 이벤트가 들어올 때 초과분 삭제)
        limit = self.spin_trace.value()
        while len(self.robot_path_items) > limit:
            old_item = self.robot_path_items.pop(0)
            self.scene.removeItem(old_item)

    def _add_drop_off(self, rx, ry):
        px, py = self.map_manager.to_scene_coords(rx, ry)
        # Drop off은 모양을 ▼(삼각형) 또는 보라색 네모등으로 지정
        size = 6
        item = self.scene.addRect(px - size/2, py - size/2, size, size, QPen(Qt.black), QBrush(Qt.magenta))
        item.setZValue(50)
        item.setVisible(self.chk_drop_off.isChecked())
        self.drop_off_items.append(item)
        
    def _add_1d_tof(self, rx, ry):
        px, py = self.map_manager.to_scene_coords(rx, ry)
        # ToF는 노란색 다이아몬드 또는 마름모
        size = 8
        polygon = QPolygonF([QPointF(px, py-size/2), QPointF(px+size/2, py), QPointF(px, py+size/2), QPointF(px-size/2, py)])
        item = self.scene.addPolygon(polygon, QPen(Qt.black), QBrush(Qt.yellow))
        item.setZValue(51)
        item.setVisible(self.chk_tof.isChecked())
        self.tof_items.append(item)
        
    def _update_log_visibility(self):
        # 로봇 궤적
        for it in self.robot_path_items:
            it.setVisible(self.chk_robot.isChecked())
        # 장애물
        for it in self.drop_off_items:
            it.setVisible(self.chk_drop_off.isChecked())
        for it in self.tof_items:
            it.setVisible(self.chk_tof.isChecked())
            
    # -- Export
    def _export_view(self):
        """
        QGraphicsScene의 현재 표시 영역을 파일로 Export 합니다.
        """
        path, _ = QFileDialog.getSaveFileName(self, "Save Scene as Image", "map_screenshot.png", "PNG (*.png);;JPG (*.jpg)")
        if not path:
            return
            
        self.view.scene().clearSelection()
        
        rect = self.scene.sceneRect()
        pixmap = QPixmap(int(rect.width()), int(rect.height()))
        pixmap.fill(Qt.transparent)
        
        painter = QPainter(pixmap)
        # 랜더링 품질 설정
        painter.setRenderHint(QPainter.Antialiasing)
        self.scene.render(painter)
        painter.end()
        
        pixmap.save(path)
        print(f"Exported view to {path}")
