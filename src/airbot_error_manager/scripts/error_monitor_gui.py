#!/usr/bin/env python3
import sys
import math
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                                 QGroupBox, QPushButton, QCheckBox, QLabel, QGridLayout, QScrollArea)
    from PyQt5.QtCore import Qt, QTimer
except ImportError as e:
    print(f"Import Error: {e}")
    print("Please make sure you have sourced your ROS workspace and installed PyQt5.")
    print("sudo apt install python3-pyqt5")
    sys.exit(1)

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from robot_custom_msgs.msg import BatteryStatus, StationData, ApTemperature, BottomIrData, TofData, RobotState, AiTemperature
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class ErrorMockNode(Node):
    def __init__(self):
        super().__init__('error_mock_gui_node')

        qos_profile_ai = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        qos_state_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.pub_bat = self.create_publisher(BatteryStatus, '/battery_status', 10)
        self.pub_st = self.create_publisher(StationData, '/station_data', 10)
        self.pub_ap = self.create_publisher(ApTemperature, '/ap_temperature_data', 10)
        self.pub_ir = self.create_publisher(BottomIrData, 'bottom_ir_data', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu_data', 10)
        self.pub_tof = self.create_publisher(TofData, '/tof_data', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_rs = self.create_publisher(RobotState, '/state_datas', qos_state_profile)
        self.pub_ai_v = self.create_publisher(String, '/ai_version', qos_profile_ai)
        self.pub_ai_t = self.create_publisher(AiTemperature, '/aitemperature_data', 10)

        # Timers (publish loops)
        self.timer_10ms = self.create_timer(0.01, self.pub_10ms_cb)
        self.timer_40ms = self.create_timer(0.04, self.pub_40ms_cb)
        self.timer_100ms = self.create_timer(0.1,  self.pub_100ms_cb)

        # Global sensor states
        self.sensor_data = {
            'bat_pct': 50,
            'is_station': False,
            'ir_adc': 100,
            'imu_roll': 0.0,
            'imu_pitch': 0.0,
            'imu_z': 9.8,
            'odom_x': 0.0,
            'tof_dist': 0.02,
            'ap_temp': 40.0,
            
            # Tx Pause controls (True means paused/delayed)
            'pause_bat_st': False,
            'pause_ir_imu': False,
            'pause_odom': False,
            'pause_tof': False,
            'pause_ap': False,
            'pause_ai': False,
        }

    def trigger_robot_state_event(self):
        msg = RobotState()
        # Event driven, just publish once
        self.pub_rs.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(math.radians(roll) * 0.5)
        sr = math.sin(math.radians(roll) * 0.5)
        cp = math.cos(math.radians(pitch) * 0.5)
        sp = math.sin(math.radians(pitch) * 0.5)
        cy = math.cos(math.radians(yaw) * 0.5)
        sy = math.sin(math.radians(yaw) * 0.5)
        return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)

    def pub_10ms_cb(self):
        # Battery & Station
        if not self.sensor_data['pause_bat_st']:
            msg_bat = BatteryStatus()
            msg_bat.battery_percent = self.sensor_data['bat_pct']
            self.pub_bat.publish(msg_bat)

            msg_st = StationData()
            msg_st.docking_status = 0x30 if self.sensor_data['is_station'] else 0x00
            self.pub_st.publish(msg_st)

        # IR & IMU
        if not self.sensor_data['pause_ir_imu']:
            msg_ir = BottomIrData()
            msg_ir.adc_ff = self.sensor_data['ir_adc']
            msg_ir.adc_fl = self.sensor_data['ir_adc']
            msg_ir.adc_fr = self.sensor_data['ir_adc']
            msg_ir.adc_bb = self.sensor_data['ir_adc']
            msg_ir.adc_bl = self.sensor_data['ir_adc']
            msg_ir.adc_br = self.sensor_data['ir_adc']
            self.pub_ir.publish(msg_ir)

            msg_imu = Imu()
            q = self.euler_to_quaternion(self.sensor_data['imu_roll'], self.sensor_data['imu_pitch'], 0.0)
            msg_imu.orientation.w, msg_imu.orientation.x, msg_imu.orientation.y, msg_imu.orientation.z = q
            msg_imu.linear_acceleration.z = self.sensor_data['imu_z']
            self.pub_imu.publish(msg_imu)

        # Odom
        if not self.sensor_data['pause_odom']:
            msg_odom = Odometry()
            msg_odom.pose.pose.position.x = self.sensor_data['odom_x']
            self.pub_odom.publish(msg_odom)
            
        # Tof
        if not self.sensor_data['pause_tof']:
            msg_tof = TofData()
            msg_tof.top = self.sensor_data['tof_dist']
            self.pub_tof.publish(msg_tof)

    def pub_40ms_cb(self):
        if not self.sensor_data['pause_ai']:
            self.pub_ai_v.publish(String())
            self.pub_ai_t.publish(AiTemperature())

    def pub_100ms_cb(self):
        if not self.sensor_data['pause_ap']:
            msg_ap = ApTemperature()
            msg_ap.ap = self.sensor_data['ap_temp']
            self.pub_ap.publish(msg_ap)


class ErrorMonitorGui(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Airbot Error Monitor Debug GUI')
        self.setGeometry(100, 100, 600, 800)
        
        main_layout = QVBoxLayout()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)

        # 1. Low Battery / Discharging / Charging
        grp_bat = QGroupBox("Battery / Station / Charging Group")
        l_bat = QGridLayout()
        btn_low_err = QPushButton("LowBattery Error (Bat 5%)")
        btn_low_err.clicked.connect(lambda: self.set_bat(5))
        btn_low_rel = QPushButton("LowBattery Release (Bat 25%)")
        btn_low_rel.clicked.connect(lambda: self.set_bat(25))
        
        btn_st_on = QPushButton("Dock to Station")
        btn_st_on.clicked.connect(lambda: self.set_st(True))
        btn_st_off = QPushButton("Undock from Station")
        btn_st_off.clicked.connect(lambda: self.set_st(False))
        
        chk_pause_bat = QCheckBox("Pause Tx (Simulate Timeout)")
        chk_pause_bat.toggled.connect(lambda c: self.set_pause('pause_bat_st', c))

        l_bat.addWidget(btn_low_err, 0, 0)
        l_bat.addWidget(btn_low_rel, 0, 1)
        l_bat.addWidget(btn_st_on, 1, 0)
        l_bat.addWidget(btn_st_off, 1, 1)
        l_bat.addWidget(chk_pause_bat, 2, 0, 1, 2)
        grp_bat.setLayout(l_bat)
        layout.addWidget(grp_bat)

        # 2. Fall Down / Lift
        grp_imu = QGroupBox("FallDown & Lift (IMU/IR) Group")
        l_imu = QGridLayout()
        btn_fall_err = QPushButton("FallDown Error (Roll 70, Pitch 70, IR 100)")
        btn_fall_err.clicked.connect(lambda: self.set_imu_ir(70.0, 70.0, 9.8, 100))
        btn_fall_rel = QPushButton("FallDown Release (Roll 0, Pitch 0, IR 7000)")
        btn_fall_rel.clicked.connect(lambda: self.set_imu_ir(0.0, 0.0, 9.8, 7000))
        
        btn_lift_err = QPushButton("Lift Error (IR 10, IMU Z 8.0)")
        btn_lift_err.clicked.connect(lambda: self.set_imu_ir(0.0, 0.0, 8.0, 10))

        chk_pause_imu = QCheckBox("Pause Tx (Simulate Timeout)")
        chk_pause_imu.toggled.connect(lambda c: self.set_pause('pause_ir_imu', c))

        l_imu.addWidget(btn_fall_err, 0, 0)
        l_imu.addWidget(btn_fall_rel, 0, 1)
        l_imu.addWidget(btn_lift_err, 1, 0)
        l_imu.addWidget(chk_pause_imu, 2, 0, 1, 2)
        grp_imu.setLayout(l_imu)
        layout.addWidget(grp_imu)

        # 3. Board Overheat
        grp_ap = QGroupBox("Board Overheat Group")
        l_ap = QGridLayout()
        btn_ap_err = QPushButton("Overheat Error (Temp 90)")
        btn_ap_err.clicked.connect(lambda: self.set_ap(90.0))
        btn_ap_rel = QPushButton("Overheat Release (Temp 40)")
        btn_ap_rel.clicked.connect(lambda: self.set_ap(40.0))
        
        chk_pause_ap = QCheckBox("Pause Tx (Simulate Timeout)")
        chk_pause_ap.toggled.connect(lambda c: self.set_pause('pause_ap', c))

        l_ap.addWidget(btn_ap_err, 0, 0)
        l_ap.addWidget(btn_ap_rel, 0, 1)
        l_ap.addWidget(chk_pause_ap, 1, 0, 1, 2)
        grp_ap.setLayout(l_ap)
        layout.addWidget(grp_ap)

        # 4. Tof
        grp_tof = QGroupBox("ToF Group")
        l_tof = QGridLayout()
        btn_tof_err = QPushButton("ToF Error (Dist 0.005)")
        btn_tof_err.clicked.connect(lambda: self.set_tof(0.005))
        btn_tof_rel = QPushButton("ToF Release (Dist 0.02)")
        btn_tof_rel.clicked.connect(lambda: self.set_tof(0.02))
        
        chk_pause_tof = QCheckBox("Pause Tx (Simulate Timeout)")
        chk_pause_tof.toggled.connect(lambda c: self.set_pause('pause_tof', c))

        l_tof.addWidget(btn_tof_err, 0, 0)
        l_tof.addWidget(btn_tof_rel, 0, 1)
        l_tof.addWidget(chk_pause_tof, 1, 0, 1, 2)
        grp_tof.setLayout(l_tof)
        layout.addWidget(grp_tof)

        # 5. Cliff Detection
        grp_cliff = QGroupBox("Cliff Detection (Odom) Group")
        l_cliff = QGridLayout()
        btn_cliff_err = QPushButton("Cliff Error (+0.4m Odom)")
        btn_cliff_err.clicked.connect(lambda: self.set_odom(0.4))
        btn_cliff_rel = QPushButton("Cliff Reset (0.0m Odom)")
        btn_cliff_rel.clicked.connect(lambda: self.set_odom(0.0))
        
        chk_pause_odom = QCheckBox("Pause Tx (Simulate Timeout)")
        chk_pause_odom.toggled.connect(lambda c: self.set_pause('pause_odom', c))

        l_cliff.addWidget(btn_cliff_err, 0, 0)
        l_cliff.addWidget(btn_cliff_rel, 0, 1)
        l_cliff.addWidget(chk_pause_odom, 1, 0, 1, 2)
        grp_cliff.setLayout(l_cliff)
        layout.addWidget(grp_cliff)

        # 6. AI Communication
        grp_ai = QGroupBox("AI Communication Group")
        l_ai = QGridLayout()
        btn_ai_trigger = QPushButton("Trigger Event: RobotState")
        btn_ai_trigger.clicked.connect(self.node.trigger_robot_state_event)
        
        chk_pause_ai = QCheckBox("Pause AI Tx (Timeout after 3~180s)")
        chk_pause_ai.toggled.connect(lambda c: self.set_pause('pause_ai', c))

        l_ai.addWidget(btn_ai_trigger, 0, 0)
        l_ai.addWidget(chk_pause_ai, 1, 0, 1, 2)
        grp_ai.setLayout(l_ai)
        layout.addWidget(grp_ai)

        # Status Label
        self.status_label = QLabel("Ready. Click buttons to change simulated sensor publishing states.")
        layout.addWidget(self.status_label)

        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

    # Handlers
    def set_bat(self, val):
        self.node.sensor_data['bat_pct'] = val
        self.status_label.setText(f"Set Battery to {val}%")
        
    def set_st(self, val):
        self.node.sensor_data['is_station'] = val
        self.status_label.setText(f"Set Station to {val}")

    def set_imu_ir(self, roll, pitch, z_acc, ir):
        self.node.sensor_data['imu_roll'] = roll
        self.node.sensor_data['imu_pitch'] = pitch
        self.node.sensor_data['imu_z'] = z_acc
        self.node.sensor_data['ir_adc'] = ir
        self.status_label.setText(f"Set IMU(r:{roll}, p:{pitch}, z:{z_acc}) and IR({ir})")

    def set_ap(self, temp):
        self.node.sensor_data['ap_temp'] = temp
        self.status_label.setText(f"Set AP Temp to {temp}")

    def set_tof(self, dist):
        self.node.sensor_data['tof_dist'] = dist
        self.status_label.setText(f"Set ToF Dist to {dist}")

    def set_odom(self, x):
        self.node.sensor_data['odom_x'] = x
        self.node.sensor_data['ir_adc'] = 1000 # Cliff uses IR + Odom
        self.status_label.setText(f"Set Odom X to {x} and IR to 1000")

    def set_pause(self, key, state):
        self.node.sensor_data[key] = state
        action = "PAUSED (Simulating Timeout/Delay)" if state else "RESUMED (Normal Publish)"
        self.status_label.setText(f"Topic Group '{key}' Tx {action}")


def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = ErrorMockNode()
    
    # Run ROS completely in the background thread
    t = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    t.start()

    app = QApplication(sys.argv)
    ex = ErrorMonitorGui(node)
    ex.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
