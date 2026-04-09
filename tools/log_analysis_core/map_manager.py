import os
import json
import yaml
from PyQt5.QtWidgets import QGraphicsPixmapItem, QGraphicsPolygonItem, QGraphicsEllipseItem, QGraphicsPathItem
from PyQt5.QtGui import QPixmap, QColor, QPolygonF, QPen, QBrush, QTransform, QPainterPath
from PyQt5.QtCore import QPointF, Qt

class MapManager:
    def __init__(self, scene):
        self.scene = scene
        
        self.map_item = None
        self.wall_items = []
        self.area_items = []
        self.station_item = None
        
        self.origin = [0, 0, 0]
        self.resolution = 0.05
        # 맵의 높이 값 (px). 이 값이 있어야 map 시점 y축이 거꾸로 된 것을 ROS 좌표로 맞출 수 있음.
        self.map_height = 0 
        
    def load_map_folder(self, folder_path):
        """
        주어진 폴더 안의 맵 메타 자산들을 로드합니다.
        airbot_map_00.pgm / .yaml, area.json, wall.json, station_pose.json 등
        """
        self.scene.clear()
        self.wall_items.clear()
        self.area_items.clear()
        self.station_item = None
        self.map_item = None
        
        # 1. yaml 및 pgm 로드
        yaml_path = self._find_file(folder_path, '.yaml')
        if not yaml_path:
            return False
            
        with open(yaml_path, 'r') as f:
            map_info = yaml.safe_load(f)
            self.resolution = float(map_info.get('resolution', 0.05))
            self.origin = map_info.get('origin', [0.0, 0.0, 0.0])
            img_file = map_info.get('image', 'airbot_map_00.pgm')
            
        img_path = os.path.join(folder_path, img_file)
        if not os.path.isfile(img_path):
            return False
            
        pixmap = QPixmap(img_path)
        if pixmap.isNull():
            return False
            
        self.map_height = pixmap.height()
        
        self.map_item = QGraphicsPixmapItem(pixmap)
        self.map_item.setZValue(-100) # 바닥
        self.scene.addItem(self.map_item)
        
        # 2. area.json 로드
        area_path = os.path.join(folder_path, 'area.json')
        if os.path.isfile(area_path):
            self._load_area(area_path)
            
        # 3. wall.json 로드
        wall_path = os.path.join(folder_path, 'wall.json')
        if os.path.isfile(wall_path):
            self._load_wall(wall_path)
            
        # 4. station_pose.json 로드
        station_path = os.path.join(folder_path, 'station_pose.json')
        if os.path.isfile(station_path):
            self._load_station(station_path)
            
        return True
        
    def _find_file(self, folder, ext):
        for file in os.listdir(folder):
            if file.endswith(ext):
                return os.path.join(folder, file)
        return None
        
    def to_scene_coords(self, x, y):
        """
        ROS 좌표계 (x,y) -> QGraphicsScene 픽셀 좌표 (px, py)
        ROS Odom 좌표계를 맵 이미지 좌표계로 변환.
        통상적으로 px = (x - origin_x) / res
        py = map_height - (y - origin_y) / res
        """
        px = (x - self.origin[0]) / self.resolution
        py = self.map_height - ((y - self.origin[1]) / self.resolution)
        return px, py
        
    def _load_area(self, path):
        """
        area.json 파싱: 금지구역(keepout_zones) 처리
        """
        with open(path, 'r') as f:
            try:
                data = json.load(f)
                zones = data.get('keepout_zones', [])
                for zone in zones:
                    polygon = QPolygonF()
                    coords = zone.get('coordinates', [])
                    for pt in coords:
                        if isinstance(pt, dict) and 'x' in pt and 'y' in pt:
                            px, py = self.to_scene_coords(pt['x'], pt['y'])
                            polygon.append(QPointF(px, py))
                            
                    if not polygon.isEmpty():
                        item = QGraphicsPolygonItem(polygon)
                        item.setBrush(QBrush(QColor(255, 0, 0, 80))) # 반투명 빨간색
                        item.setPen(QPen(Qt.NoPen))
                        item.setZValue(-50)
                        self.scene.addItem(item)
                        self.area_items.append(item)
            except Exception as e:
                print(f"Error loading area.json: {e}")

    def _load_wall(self, path):
        """
        wall.json 파싱: 선분들의 집합. 금지된 벽
        """
        with open(path, 'r') as f:
            try:
                data = json.load(f)
                pen = QPen(QColor(255, 165, 0, 150))
                pen.setWidth(3)
                
                if isinstance(data, list):
                    for w in data:
                        # 구조 추정치
                        points = w.get('points', w) if isinstance(w, dict) else w
                        if len(points) >= 2:
                            path_item = QPainterPath()
                            for i, pt in enumerate(points):
                                x = pt.get('x') if isinstance(pt, dict) else pt[0]
                                y = pt.get('y') if isinstance(pt, dict) else pt[1]
                                px, py = self.to_scene_coords(x, y)
                                if i == 0:
                                    path_item.moveTo(px, py)
                                else:
                                    path_item.lineTo(px, py)
                            
                            item = QGraphicsPathItem(path_item)
                            item.setPen(pen)
                            item.setZValue(-50)
                            self.scene.addItem(item)
                            self.wall_items.append(item)
            except Exception as e:
                print(f"Error loading wall.json: {e}")

    def _load_station(self, path):
        with open(path, 'r') as f:
            try:
                data = json.load(f)
                station_data = data.get('station_pose', data)
                self.station_x = station_data.get('x', 0)
                self.station_y = station_data.get('y', 0)
                self.station_theta = station_data.get('theta', 0.0)
                
                self.redraw_station(3.0) # 기본 사이즈 3.0
            except Exception as e:
                print(f"Error loading station_pose.json: {e}")

    def redraw_station(self, size):
        if self.station_item is not None:
            if self.station_item in self.scene.items():
                self.scene.removeItem(self.station_item)
            self.station_item = None
            
        if not hasattr(self, 'station_x'):
            return
            
        px, py = self.to_scene_coords(self.station_x, self.station_y)
        
        width = size * 2.0
        height = size * 1.5
        arrow_size = size
        
        path = QPainterPath()
        # 직사각형 몸통 추가
        path.addRect(-width/2, -height/2, width, height)
        
        # 앞쪽 화살표 폴리곤 추가
        arrow_poly = QPolygonF([
            QPointF(width/2, -arrow_size/2),
            QPointF(width/2 + arrow_size, 0),
            QPointF(width/2, arrow_size/2)
        ])
        path.addPolygon(arrow_poly)
        
        self.station_item = self.scene.addPath(path, QPen(Qt.black), QBrush(QColor(0, 255, 0, 200)))
        self.station_item.setPos(px, py)
        
        import math
        # PyQt 좌표계는 y가 아래를 향하므로 회전 방향 반전 적용
        self.station_item.setRotation(-math.degrees(self.station_theta))
        self.station_item.setZValue(-40)

    def set_visibility(self, layer_name, visible):
        if layer_name == "area":
            for i in self.area_items:
                i.setVisible(visible)
        elif layer_name == "wall":
            for i in self.wall_items:
                i.setVisible(visible)
        elif layer_name == "station":
            if self.station_item:
                self.station_item.setVisible(visible)
