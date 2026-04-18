#!/usr/bin/env python3
"""
Techman TM5-900 Robot HMI — PyQt5 (Lightweight Control Panel)
==================================================================
This version removes the embedded web viewer. Use RViz2 for 3D visualization.
Architecture:
  HMI (PyQt5) <──ROS 2──> Webots Simulation & /joint_states
                          RViz2 (runs separately for 3D view)
Launch helpers (separate terminals):
  1. ros2 launch tm5_900 webots.launch.py
  2. ros2 launch tm5_moveit_config demo.launch.py (or RViz2 standalone)
  3. python3 tm5_900_hmi.py
Dependencies:
  sudo apt install python3-pyqt5
"""
import  os, sys, math, random, threading
import subprocess
from turtle import mode
os.environ["QTWEBENGINE_DISABLE_GPU"] = "1"
os.environ["QT_QPA_PLATFORM"] = "xcb"

from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QSlider, QFrame,
    QScrollArea, QFileDialog, QProgressBar, QTextEdit, QStackedWidget,
    QSizePolicy
)
from PyQt5.QtCore  import Qt, QTimer, QThread, pyqtSignal, pyqtSlot,QUrl
from PyQt5.QtGui   import (QColor, QPainter, QPen, QBrush, QLinearGradient, QFontDatabase)
from PyQt5.QtWebEngineWidgets import (QWebEngineView, QWebEngineSettings)
# ─────────────────────────────────────────────────────────────────────────────
#  COLOUR PALETTE
# ─────────────────────────────────────────────────────────────────────────────
os.environ["QTWEBENGINE_DISABLE_GPU"] = "1"
C = {
    "bg":    "#060b10", "card":  "#0d1825", "card2": "#0b1520",
    "b":     "#16293e", "bb":    "#1c3650",
    "a":     "#00d4aa", "a2":    "#00ff9d", "a3":    "#ff6b35",
    "t1":    "#c8dced", "t2":    "#4e7290", "t3":    "#273d52",
    "red":   "#ff3d5a", "warn":  "#ffb830", "ok":    "#00e676",
}
STYLE_BASE = f"""
QMainWindow, QWidget {{ background:{C['bg']}; color:{C['t1']}; font-family:'Exo 2','Rajdhani',sans-serif; font-size:11px; }}
QScrollBar:vertical   {{ background:transparent; width:4px; border-radius:2px; }}
QScrollBar::handle:vertical {{ background:{C['bb']}; border-radius:2px; min-height:20px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0; }}
QScrollBar:horizontal {{ background:transparent; height:4px; }}
QScrollBar::handle:horizontal {{ background:{C['bb']}; border-radius:2px; }}
QScrollBar::add-line:horizontal,QScrollBar::sub-line:horizontal {{ width:0; }}
"""
TM5_JOINTS = [
    {"id": "J1", "ros": "link1_joint", "nm": "Shoulder 1", "mn": -270, "mx":  270, "v":   0},
    {"id": "J2", "ros": "link2_joint", "nm": "Shoulder 2", "mn": -180, "mx":  180, "v":   0},
    {"id": "J3", "ros": "link3_joint", "nm": "Elbow",      "mn": -155, "mx":  155, "v":  90},
    {"id": "J4", "ros": "link4_joint", "nm": "Wrist 1",    "mn": -180, "mx":  180, "v":   0},
    {"id": "J5", "ros": "link5_joint", "nm": "Wrist 2",    "mn": -180, "mx":  180, "v":  90},
    {"id": "J6", "ros": "link6_joint", "nm": "Wrist 3",    "mn": -270, "mx":  270, "v":   0},
]
# ─────────────────────────────────────────────────────────────────────────────
#  ROS 2 WORKER
# ─────────────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
class RosWorker(QThread):
    joint_state_received = pyqtSignal(list, list, list)
    connection_changed   = pyqtSignal(bool)
    log_message          = pyqtSignal(str, str)
    JOINT_NAMES = [j["ros"] for j in TM5_JOINTS]
    def __init__(self):
        super().__init__()
        self._node = None; self._active = True
        self._pub_j = self._pub_t = self._pub_c = None
        self._lock  = threading.Lock()
    def run(self):
        if not ROS_AVAILABLE:
            self.connection_changed.emit(False)
            self.log_message.emit("rclpy not found — demo mode", "warn"); return
        try:
            rclpy.init()
            self._node = Node("tm5_hmi")
            self._pub_j = self._node.create_publisher(JointTrajectory, "/tm_arm_controller/joint_trajectory", 10)
            self._pub_t = self._node.create_publisher(Twist,  "/cmd_vel", 10)
            self._pub_c = self._node.create_publisher(String, "/robot_command", 10)
            self._node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
            self.connection_changed.emit(True)
            self.log_message.emit("ROS 2 connected — tm5_hmi node online", "ok")
            while self._active and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception as exc:
            self.connection_changed.emit(False)
            self.log_message.emit(f"ROS 2 error: {exc}", "err")
    def _js_cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        pos, vel, eff = [], [], []
        for jn in self.JOINT_NAMES:
            i = idx.get(jn)
            pos.append(msg.position[i] if i is not None and msg.position else 0.0)
            vel.append(msg.velocity[i] if i is not None and msg.velocity else 0.0)
            eff.append(msg.effort[i]   if i is not None and msg.effort   else 0.0)
        self.joint_state_received.emit(pos, vel, eff)
    def publish_joints(self, angles_deg):
        if not self._pub_j: return
        n  = len(TM5_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions       = [math.radians(a) for a in angles_deg[:n]]
        pt.velocities      = [0.0]*n; pt.accelerations = [0.0]*n
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES; msg.points = [pt]
        with self._lock: self._pub_j.publish(msg)
    def publish_twist(self, lx, ly, lz, az):
        if not self._pub_t: return
        msg = Twist()
        msg.linear.x = lx; msg.linear.y = ly; msg.linear.z = lz; msg.angular.z = az
        with self._lock: self._pub_t.publish(msg)
    def publish_cmd(self, cmd):
        if not self._pub_c: return
        msg = String(); msg.data = cmd
        with self._lock: self._pub_c.publish(msg)
    def stop(self):
        self._active = False
        try:
            if self._node: self._node.destroy_node()
        except: pass
        try:
            if ROS_AVAILABLE and rclpy.ok(): rclpy.shutdown()
        except: pass
# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS & COMPONENTS
# ─────────────────────────────────────────────────────────────────────────────
def make_label(text, color=C["t1"], size=11, bold=False, mono=False):
    lbl = QLabel(text)
    fam = "'Share Tech Mono'" if mono else "'Rajdhani',sans-serif"
    lbl.setStyleSheet(f"color:{color}; font-size:{size}px; font-weight:{'bold' if bold else 'normal'}; font-family:{fam}; background:transparent;")
    return lbl
def slider_style(color):
    return (f"QSlider::groove:horizontal{{height:4px;background:rgba(255,255,255,0.07);border-radius:2px;}}"
            f"QSlider::handle:horizontal{{width:13px;height:13px;margin:-5px 0;border-radius:7px;background:{color};border:2px solid {C['bg']};}}"
            f"QSlider::sub-page:horizontal{{background:{color};border-radius:2px;}}")
class Panel(QFrame):
    def __init__(self, title, tag="", parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"Panel{{background:{C['card']};border:1px solid {C['b']};border-radius:7px;}}")
        outer = QVBoxLayout(self); outer.setContentsMargins(0,0,0,0); outer.setSpacing(0)
        hdr = QFrame(); hdr.setFixedHeight(24)
        hdr.setStyleSheet(f"background:rgba(0,0,0,0.2);border-bottom:1px solid {C['b']};border-radius:0;")
        hl = QHBoxLayout(hdr); hl.setContentsMargins(11,0,11,0)
        hl.addWidget(make_label(title, C["a"], 10, True))
        hl.addStretch()
        if tag: hl.addWidget(make_label(tag, C["t3"], 8, mono=True))
        outer.addWidget(hdr)
        self.content = QWidget(); self.content.setStyleSheet("background:transparent;")
        self.body = QVBoxLayout(self.content)
        self.body.setContentsMargins(10,10,10,10); self.body.setSpacing(8)
        outer.addWidget(self.content)
class StatCard(QFrame):
    ACCENT = {"g":C["a2"],"b":C["a"],"o":C["a3"],"r":C["red"]}
    def __init__(self, label, value, sub, kind="b", parent=None):
        super().__init__(parent); self._kind = kind
        self.setMinimumHeight(60)
        self.setStyleSheet(f"StatCard{{background:{C['card2']};border:1px solid {C['b']};border-radius:6px;}}")
        ly = QVBoxLayout(self); ly.setContentsMargins(10,8,10,8); ly.setSpacing(2)
        self.lbl = make_label(label,C["t3"],10,mono=True)
        self.val = make_label(value,C["t1"],18,True)
        self.sub = make_label(sub,  C["t3"], 8,mono=True)
        ly.addWidget(self.lbl); ly.addWidget(self.val); ly.addWidget(self.sub)
    def set_value(self, v, color=None):
        self.val.setText(str(v))
        if color: self.val.setStyleSheet(f"color:{color};font-size:18px;font-weight:bold;background:transparent;")
class JointCard(QFrame):
    def __init__(self, jid, ros_name, display_name, mn, mx, parent=None):
        super().__init__(parent); self._mn=mn; self._mx=mx
        self.setMinimumHeight(90)
        self.setStyleSheet(f"JointCard{{background:{C['card2']};border:1px solid {C['b']};border-radius:7px;}}")
        ly = QVBoxLayout(self); ly.setContentsMargins(10,8,10,8); ly.setSpacing(4)
        hr = QHBoxLayout()
        left = QVBoxLayout(); left.setSpacing(0)
        self.id_lbl = make_label(jid, C["a"], 13, True)
        self.nm_lbl = make_label(ros_name.upper(), C["t3"], 7, mono=True)
        left.addWidget(self.id_lbl); left.addWidget(self.nm_lbl)
        hr.addLayout(left); hr.addStretch()
        self.dot = QLabel(); self.dot.setFixedSize(8,8); self._set_dot("ok")
        hr.addWidget(self.dot); ly.addLayout(hr)
        self.disp_lbl = make_label(display_name, C["t2"], 9); ly.addWidget(self.disp_lbl)
        self.pos_lbl  = make_label("0.0°", C["t1"], 14, True); ly.addWidget(self.pos_lbl)
        self.pos_bar  = QProgressBar()
        self.pos_bar.setRange(0,1000); self.pos_bar.setValue(500)
        self.pos_bar.setTextVisible(False); self.pos_bar.setFixedHeight(4)
        self.pos_bar.setStyleSheet(
            f"QProgressBar{{background:rgba(255,255,255,0.05);border-radius:2px;border:none;}}"
            f"QProgressBar::chunk{{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 {C['a']},stop:1 {C['a2']});border-radius:2px;}}")
        ly.addWidget(self.pos_bar)
        row = QHBoxLayout()
        vl = QVBoxLayout(); vl.setSpacing(0)
        vl.addWidget(make_label("VEL",C["t3"],7,mono=True))
        self.vel_val = make_label("0.00°/s",C["a2"],9,True); vl.addWidget(self.vel_val)
        el = QVBoxLayout(); el.setSpacing(0)
        el.addWidget(make_label("EFF",C["t3"],7,mono=True))
        self.eff_val = make_label("0.00Nm",C["a3"],9,True); el.addWidget(self.eff_val)
        row.addLayout(vl); row.addLayout(el); ly.addLayout(row)
    def _set_dot(self, state):
        c = {"ok":C["ok"],"w":C["warn"],"e":C["red"]}.get(state,C["ok"])
        self.dot.setStyleSheet(f"background:{c};border-radius:4px;")
    def update_data(self, deg, vel, eff):
        self.pos_lbl.setText(f"{deg:.1f}°")
        pct = int((deg-self._mn)/(self._mx-self._mn)*1000)
        self.pos_bar.setValue(max(0,min(1000,pct)))
        self.vel_val.setText(f"{vel:.2f}°/s"); self.eff_val.setText(f"{eff:.2f}Nm")
        self._set_dot("w" if abs(vel)>4 else "ok")
class TcpCell(QFrame):
    def __init__(self, axis, color, unit="mm", parent=None):
        super().__init__(parent); self._unit=unit
        self.setStyleSheet(f"TcpCell{{background:rgba(0,0,0,0.25);border:1px solid {C['b']};border-radius:5px;}}")
        ly = QVBoxLayout(self); ly.setContentsMargins(8,6,8,6); ly.setSpacing(2)
        ly.addWidget(make_label(axis,color,9,mono=True))
        self.val = make_label(f"0.0 {unit}",C["t1"],12,True)
        self.bar = QProgressBar(); self.bar.setRange(0,1000); self.bar.setValue(500)
        self.bar.setTextVisible(False); self.bar.setFixedHeight(3)
        self.bar.setStyleSheet(f"QProgressBar{{background:rgba(255,255,255,0.04);border-radius:1px;border:none;}} QProgressBar::chunk{{background:{color};border-radius:1px;}}")
        ly.addWidget(self.val); ly.addWidget(self.bar)
    def set_value(self, v, mn=-900, mx=900):
        self.val.setText(f"{v:.1f} {self._unit}")
        pct = int((v-mn)/(mx-mn)*1000); self.bar.setValue(max(0,min(1000,pct)))
class CmdButton(QPushButton):
    def __init__(self, icon, label, kind="normal", parent=None):
        super().__init__(parent)
        ly = QVBoxLayout(self); ly.setContentsMargins(4,9,4,9); ly.setAlignment(Qt.AlignCenter)
        il = QLabel(icon); il.setAlignment(Qt.AlignCenter)
        il.setStyleSheet("font-size:16px;background:transparent;")
        ll = QLabel(label); ll.setAlignment(Qt.AlignCenter)
        ll.setStyleSheet("font-size:8.5px;letter-spacing:0.15em;font-family:'Share Tech Mono';background:transparent;")
        ly.addWidget(il); ly.addWidget(ll)
        fc,bg = {"start":(C["a2"],"rgba(0,255,157,0.07)"),"stop":(C["red"],"rgba(255,61,90,0.05)"),"normal":(C["t2"],C["card2"])}.get(kind,(C["t2"],C["card2"]))
        self.setStyleSheet(f"CmdButton{{color:{fc};background:{bg};border:1px solid {C['b']};border-radius:5px;min-height:50px;}} CmdButton:hover{{border-color:{C['a']};color:{C['a']};background:rgba(0,212,170,0.09);}}")
class LogWidget(QTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent); self.setReadOnly(True)
        self.setStyleSheet(f"QTextEdit{{background:#030810;border:1px solid {C['b']};border-radius:4px;font-family:'Share Tech Mono';font-size:9px;color:{C['t2']};}}")
        self._count = 0
    def add(self, msg, kind="info"):
        col = {"ok":C["ok"],"warn":C["warn"],"err":C["red"],"info":C["a"]}.get(kind,C["a"])
        ts  = datetime.now().strftime("%H:%M:%S"); self._count += 1
        self.append(f'<span style="color:{C["t3"]}">{ts}</span> <span style="color:{col}">{msg}</span>')
        sb = self.verticalScrollBar(); sb.setValue(sb.maximum())
        return self._count
    
class ViewerFrame(QFrame):
    def __init__(self, title="TM5-900 · LIVE WEBOTS SCENE", parent=None):
        super().__init__(parent)

        self.setStyleSheet(f"background:#040c14;border:1px solid {C['b']};border-radius:7px;")

        ly = QVBoxLayout(self)
        ly.setContentsMargins(0,0,0,0)

        self.browser = QWebEngineView()

        # get Windows IP (WSL gateway)
        windows_ip = "127.0.0.1"
        try:
            cmd = "ip route show default | awk '{print $3}'"
            windows_ip = subprocess.check_output(cmd, shell=True).decode().strip()
        except:
            pass

        self.windows_ip = windows_ip
        print(f"[INFO] Windows IP: {windows_ip}")

        # 🔥 IMPORTANT: load WITHOUT ?url
        self.browser.loadFinished.connect(self._on_loaded)
        self.browser.setUrl(QUrl(f"http://{windows_ip}:1234/index.html"))

        ly.addWidget(self.browser)

    def _on_loaded(self, ok):
        if not ok:
            print("[ERROR] Page failed to load")
            return

        ip = self.windows_ip

        print("[INFO] Injecting WebSocket fix...")

        js = f"""
        (function() {{
            const input = document.getElementById('IP-input');
            const btn = document.getElementById('connect-button');

            if (input) {{
                input.value = "ws://{ip}:1234";
                console.log("Fixed WS URL:", input.value);
            }}
            if (mode) {{
            mode.value = "mjpeg";
            mode.dispatchEvent(new Event('change', {{ bubbles: true }}));
            console.log("Forced mode: mjpeg");
        }}
            if (btn) {{
                setTimeout(() => {{
                    btn.click();
                    console.log("Auto connect clicked");
                }}, 500);
            }}
        }})();
        """

        self.browser.page().runJavaScript(js)
# ---------------------------------------------------------
    

# ─────────────────────────────────────────────────────────────────────────────
#  TOP BAR
# ─────────────────────────────────────────────────────────────────────────────
class TopBar(QFrame):
    nav_changed   = pyqtSignal(str)
    estop_pressed = pyqtSignal()
    def __init__(self, parent=None):
        super().__init__(parent); self.setFixedHeight(56)
        self.setStyleSheet(f"TopBar{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #0b1926,stop:1 {C['bg']});border-bottom:1px solid {C['b']};}}")
        main = QHBoxLayout(self); main.setContentsMargins(18,0,18,0)
        logo = QLabel("TM"); logo.setFixedSize(32,32); logo.setAlignment(Qt.AlignCenter)
        logo.setStyleSheet(f"color:{C['a']};border:2px solid {C['a']};border-radius:5px;font-family:'Rajdhani';font-size:14px;font-weight:700;background:transparent;")
        name = QLabel(f"TM5-900 <span style='color:{C['a']}'> CONTROL</span>")
        name.setTextFormat(Qt.RichText)
        name.setStyleSheet(f"font-family:'Rajdhani';font-size:20px;font-weight:700;color:{C['t1']};background:transparent;")
        
        # Proje detaylarına ve şase yapısına atıfta bulunulan güncellenmiş alt başlık
        sub = QLabel("6-DOF COBOT (3D PRINTED CHASSIS) · HMI v2.0  |  DEV: HAMZA & AHMET")
        sub.setStyleSheet(f"font-family:'Share Tech Mono';font-size:9px;color:{C['t3']};letter-spacing:.18em;background:transparent;")
        info_col = QVBoxLayout(); info_col.setSpacing(0)
        info_col.addWidget(name); info_col.addWidget(sub)
        main.addWidget(logo); main.addLayout(info_col); main.addStretch()
        self._btn_dash = QPushButton("DASHBOARD")
        self._btn_ctrl = QPushButton("CONTROL")
        for btn, pg in [(self._btn_dash,"dash"),(self._btn_ctrl,"ctrl")]:
            btn.setFixedHeight(30)
            btn.clicked.connect(lambda _, p=pg: self._nav(p))
            main.addWidget(btn)
        self._active = "dash"; self._update_nav(); main.addSpacing(12)
        self.pill = QLabel("● CONNECTING")
        self.pill.setStyleSheet(f"background:rgba(255,61,90,0.07);border:1px solid rgba(255,61,90,0.22);border-radius:12px;color:{C['red']};font-family:'Share Tech Mono';font-size:10px;padding:4px 12px;")
        main.addWidget(self.pill)
        self.clock = QLabel("--:--:--")
        self.clock.setStyleSheet(f"font-family:'Share Tech Mono';font-size:12px;color:{C['t2']};min-width:70px;background:transparent;")
        main.addWidget(self.clock)
        es = QPushButton("⬛ E-STOP")
        es.setStyleSheet(f"QPushButton{{background:rgba(255,61,90,0.1);border:2px solid {C['red']};border-radius:4px;color:{C['red']};font-family:'Rajdhani';font-size:12px;font-weight:700;letter-spacing:.15em;padding:6px 15px;}} QPushButton:hover{{background:rgba(255,61,90,0.22);}} QPushButton:pressed{{background:{C['red']};color:#fff;}}")
        es.clicked.connect(self.estop_pressed); main.addWidget(es)
    def _nav_style(self, active=False):
        if active:
            return f"QPushButton{{padding:6px 18px;background:rgba(0,212,170,0.09);border:1px solid {C['a']};border-radius:4px;color:{C['a']};font-family:'Rajdhani';font-size:12px;font-weight:600;letter-spacing:.1em;}}"
        return f"QPushButton{{padding:6px 18px;background:transparent;border:1px solid {C['b']};border-radius:4px;color:{C['t2']};font-family:'Rajdhani';font-size:12px;font-weight:600;letter-spacing:.1em;}} QPushButton:hover{{border-color:{C['a']};color:{C['a']};}}"
    def _update_nav(self):
        self._btn_dash.setStyleSheet(self._nav_style(self._active=="dash"))
        self._btn_ctrl.setStyleSheet(self._nav_style(self._active=="ctrl"))
    def _nav(self, pg):
        self._active = pg; self._update_nav(); self.nav_changed.emit(pg)
    def set_ros_status(self, online):
        if online:
            self.pill.setText("● ROS 2 ONLINE")
            self.pill.setStyleSheet(f"background:rgba(0,230,118,0.07);border:1px solid rgba(0,230,118,0.22);border-radius:12px;color:{C['ok']};font-family:'Share Tech Mono';font-size:10px;padding:4px 12px;")
        else:
            self.pill.setText("● OFFLINE")
            self.pill.setStyleSheet(f"background:rgba(255,61,90,0.07);border:1px solid rgba(255,61,90,0.22);border-radius:12px;color:{C['red']};font-family:'Share Tech Mono';font-size:10px;padding:4px 12px;")
    def tick(self): self.clock.setText(datetime.now().strftime("%H:%M:%S"))
# ─────────────────────────────────────────────────────────────────────────────
#  DASHBOARD PAGE (Optimized Layout without WebEngine)
# ─────────────────────────────────────────────────────────────────────────────
class DashPage(QWidget):
    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent); self._ros=ros; self._joints=joints; self._build_ui()
    def _build_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)
        # 1. SÜTUN: Webots 3D Görüntüsü (En solda ve en geniş)
        self._viewer_frame = ViewerFrame()
        main_layout.addWidget(self._viewer_frame, 2) # 2 birim genişlik alır
        # 2. SÜTUN: İstatistikler ve Eklem Durumları (Ortada)
        mid_col = QVBoxLayout()
        mid_col.setSpacing(15)
        mid_col.addWidget(self._make_stats())
        mid_col.addWidget(self._make_tcp())
        mid_col.addWidget(self._make_joints())
        mid_col.addStretch()
        main_layout.addLayout(mid_col, 1) # 1 birim genişlik alır
        # 3. SÜTUN: Dosya yükleme ve Log ekranı (En sağda)
        right_col = QVBoxLayout()
        right_col.setSpacing(15)
        right_col.addWidget(self._make_right_tools())
        main_layout.addLayout(right_col, 1) # 1 birim genişlik alır
    def _make_stats(self):
        pnl = Panel("SYSTEM OVERVIEW", "REAL-TIME")
        g = QGridLayout(); g.setSpacing(8)
        self._sc_state = StatCard("ROBOT STATE",    "STANDBY","READY FOR COMMAND","g")
        self._sc_spd   = StatCard("TCP SPEED",      "0 mm/s", "MAX 2500 mm/s",   "b")
        self._sc_prog  = StatCard("ACTIVE PROGRAM", "NONE",   "NO FILE LOADED",  "b")
        self._sc_reach = StatCard("REACH",          "900 mm", "TM5-900 MAX",     "o")
        self._sc_up    = StatCard("UPTIME",         "00:00",  "SESSION TIME",    "g")
        self._sc_err   = StatCard("ERRORS",         "0",      "ACTIVE FAULTS",   "r")
        
        for i, c in enumerate([self._sc_state, self._sc_spd, self._sc_prog, self._sc_reach, self._sc_up, self._sc_err]):
            g.addWidget(c, i//3, i%3)
        pnl.body.addLayout(g)
        return pnl
    def _make_tcp(self):
        pnl = Panel("TCP CARTESIAN POSITION", "WORLD FRAME")
        grid = QGridLayout(); grid.setSpacing(8)
        self._tcp = {
            "X": TcpCell("X AXIS","#ff4d6d"),  "Y": TcpCell("Y AXIS",C["a2"]),
            "Z": TcpCell("Z AXIS",C["a"]),      "RX":TcpCell("RX",C["a3"],"°"),
            "RY":TcpCell("RY",C["a3"],"°"),     "RZ":TcpCell("RZ",C["a3"],"°"),
        }
        for i,(_, w) in enumerate(self._tcp.items()): 
            grid.addWidget(w, i//3, i%3)
        pnl.body.addLayout(grid)
        return pnl
    def _make_joints(self):
        pnl = Panel("JOINT STATES", "6-DOF KINEMATICS")
        g = QGridLayout(); g.setSpacing(8)
        self._jcards = []
        for i, j in enumerate(self._joints):
            card = JointCard(j["id"],j["ros"],j["nm"],j["mn"],j["mx"])
            g.addWidget(card, i//3, i%3)
            self._jcards.append(card)
        pnl.body.addLayout(g)
        return pnl
    def _make_right_tools(self):
        w = QWidget(); w.setStyleSheet("background:transparent;")
        ly = QVBoxLayout(w); ly.setContentsMargins(0,0,0,0); ly.setSpacing(15)
        # Program loader
        prog = Panel("PROGRAM LOADER","NO FILE")
        self._file_lbl = make_label("Drop or click to load",C["t2"],10,mono=True)
        self._file_lbl.setAlignment(Qt.AlignCenter); self._file_lbl.setWordWrap(True)
        load_btn = QPushButton("📁  BROWSE FILE")
        load_btn.setStyleSheet(f"QPushButton{{background:transparent;border:2px dashed {C['bb']};border-radius:5px;color:{C['t2']};font-family:'Share Tech Mono';font-size:10px;padding:8px;letter-spacing:.12em;}} QPushButton:hover{{border-color:{C['a']};color:{C['a']};background:rgba(0,212,170,0.04);}}")
        load_btn.clicked.connect(self._browse)
        self._pbar = QProgressBar(); self._pbar.setRange(0,100); self._pbar.setValue(0)
        self._pbar.setTextVisible(False); self._pbar.setFixedHeight(6)
        self._pbar.setStyleSheet(f"QProgressBar{{background:rgba(255,255,255,0.04);border-radius:3px;border:none;}} QProgressBar::chunk{{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 {C['a2']},stop:1 {C['a']});border-radius:3px;}}")
        self._pbar_lbl = make_label("READY",C["t3"],8,mono=True)
        
        run_btn = QPushButton("▶  START"); run_btn.setStyleSheet(f"QPushButton{{background:rgba(0,255,157,0.1);border:2px solid {C['a2']};border-radius:6px;color:{C['a2']};font-family:'Rajdhani';font-size:12px;font-weight:700;letter-spacing:.12em;padding:8px;}} QPushButton:hover{{background:rgba(0,255,157,0.2);}}")
        stop_btn = QPushButton("⏹  STOP"); stop_btn.setStyleSheet(f"QPushButton{{background:rgba(255,61,90,0.1);border:2px solid {C['red']};border-radius:6px;color:{C['red']};font-family:'Rajdhani';font-size:12px;font-weight:700;letter-spacing:.12em;padding:8px;}} QPushButton:hover{{background:rgba(255,61,90,0.2);}}")
        run_btn.clicked.connect(self._run_prog); stop_btn.clicked.connect(self._stop_prog)
        btns = QHBoxLayout(); btns.addWidget(run_btn); btns.addWidget(stop_btn)
        
        prog.body.addWidget(load_btn); prog.body.addWidget(self._file_lbl)
        prog.body.addWidget(self._pbar_lbl); prog.body.addWidget(self._pbar)
        prog.body.addLayout(btns); ly.addWidget(prog)
        # Diagnostics
        diag = Panel("DIAGNOSTICS", "HARDWARE"); self._diag = {}
        for key, label, color in [("temp","JOINT TEMP AVG",C["a"]), ("pwr", "POWER DRAW", C["a2"]), ("lat", "ROS LATENCY", C["a"])]:
            row = QHBoxLayout()
            lbl = make_label(label,C["t3"],8,mono=True); lbl.setFixedWidth(90)
            bar = QProgressBar(); bar.setRange(0,100); bar.setValue(40)
            bar.setTextVisible(False); bar.setFixedHeight(4)
            bar.setStyleSheet(f"QProgressBar{{background:rgba(255,255,255,0.04);border-radius:2px;border:none;}} QProgressBar::chunk{{background:{color};border-radius:2px;}}")
            val = make_label("--",C["t1"],11,True); val.setFixedWidth(40); val.setAlignment(Qt.AlignRight)
            row.addWidget(lbl); row.addWidget(bar); row.addWidget(val)
            self._diag[key]=(bar,val); diag.body.addLayout(row)
        ly.addWidget(diag)
        # Log
        log_pnl = Panel("SYSTEM LOG", "RUNTIME"); self._log = LogWidget()
        log_pnl.body.addWidget(self._log); ly.addWidget(log_pnl, 1) # 1 ensures it stretches to fill bottom
        return w
    _prog_file=None; _prog_pct=0.0; _prog_timer=None
    def _browse(self):
        path,_ = QFileDialog.getOpenFileName(self,"Load Program","", "Robot Programs (*.script *.txt *.json *.py *.xml);;All Files (*)")
        if path:
            self._prog_file=path; self._file_lbl.setText(f"📄 {path.split('/')[-1]}")
            self._pbar_lbl.setText("LOADED"); self.log(f"Loaded: {path.split('/')[-1]}","ok")
    def _run_prog(self):
        if not self._prog_file: self.log("No program loaded","err"); return
        self._prog_pct=0.0; self._pbar_lbl.setText("RUNNING")
        self._sc_state.set_value("RUNNING",C["a2"])
        self._prog_timer=QTimer(self); self._prog_timer.timeout.connect(self._tick_prog)
        self._prog_timer.start(200); self._ros.publish_cmd(f"RUN:{self._prog_file}")
    def _tick_prog(self):
        self._prog_pct=min(100.0,self._prog_pct+random.uniform(0.5,2.5))
        self._pbar.setValue(int(self._prog_pct))
        if self._prog_pct>=100.0:
            self._prog_timer.stop(); self._pbar_lbl.setText("COMPLETE")
            self._sc_state.set_value("DONE",C["a"]); self.log("Program complete","ok")
    def _stop_prog(self):
        if self._prog_timer: self._prog_timer.stop()
        self._pbar.setValue(0); self._prog_pct=0.0; self._pbar_lbl.setText("STOPPED")
        self._sc_state.set_value("STOPPED",C["red"]); self.log("Program stopped","warn")
        self._ros.publish_cmd("STOP")
    def update_from_ros(self, pos, vel, eff):
        for i, card in enumerate(self._jcards):
            if i >= len(pos): break
            deg = math.degrees(pos[i])
            self._joints[i]["v"] = deg
            card.update_data(deg, math.degrees(vel[i]) if i<len(vel) else 0.0, eff[i] if i<len(eff) else 0.0)
    def update_demo(self, t):
        for i, j in enumerate(self._joints):
            j["v"] = max(j["mn"],min(j["mx"],j["v"]+(random.random()-0.5)*0.3))
            self._jcards[i].update_data(j["v"],(random.random()-0.5)*2,(random.random()-0.5)*5)
        x=math.sin(t/3)*300; y=math.cos(t/4)*200; z=650+math.sin(t/5)*80
        self._tcp["X"].set_value(x); self._tcp["Y"].set_value(y)
        self._tcp["Z"].set_value(z,0,1400)
        self._tcp["RX"].set_value(math.sin(t/6)*30,-180,180)
        self._tcp["RY"].set_value(math.cos(t/7)*20,-180,180)
        self._tcp["RZ"].set_value(math.sin(t/8)*45,-180,180)
        self._sc_spd.set_value(f"{abs(math.sin(t/2)*100):.0f} mm/s")
        tmp=38+random.random()*8; pwr=80+random.random()*60; lat=5+random.random()*15
        self._diag["temp"][0].setValue(int(tmp));  self._diag["temp"][1].setText(f"{tmp:.0f}°C")
        self._diag["pwr"][0].setValue(int(pwr/300*100)); self._diag["pwr"][1].setText(f"{pwr:.0f}W")
        self._diag["lat"][0].setValue(int(lat));   self._diag["lat"][1].setText(f"{lat:.0f}ms")
    def log(self, msg, kind="info"): return self._log.add(msg,kind)

# ─────────────────────────────────────────────────────────────────────────────
#  CONTROL PAGE (Optimized Two-Column Layout)
# ─────────────────────────────────────────────────────────────────────────────
class CtrlPage(QWidget):
    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent); self._ros=ros; self._joints=joints
        self._jx=self._jy=self._jz=self._jwz=0.0
        self._joy_timer=QTimer(self); self._joy_timer.timeout.connect(self._pub_twist)
        self._build_ui()
    def _build_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)
        # Left Column (Mode & Sliders)
        left_col = QVBoxLayout()
        left_col.setSpacing(15)
        
        mode_pnl = Panel("OPERATION MODE")
        mg = QGridLayout(); mg.setSpacing(6); self._mode_btns=[]
        for i, m in enumerate(["MANUAL","AUTO","TEACH","SIMULATION"]):
            b = QPushButton(m); b.setCheckable(True); b.setChecked(m=="MANUAL")
            b.setStyleSheet(self._mode_style())
            b.clicked.connect(lambda _, btn=b, md=m: self._set_mode(btn,md))
            b.setMinimumHeight(35)
            mg.addWidget(b,i//2,i%2); self._mode_btns.append(b)
        mode_pnl.body.addLayout(mg); left_col.addWidget(mode_pnl)
        s_pnl = Panel("MANUAL JOINT CONTROL","6-DOF · DEG")
        sw = QWidget(); sw.setStyleSheet("background:transparent;")
        sl_ly = QVBoxLayout(sw); sl_ly.setSpacing(15); self._sliders=[]
        for i, j in enumerate(self._joints):
            box = QVBoxLayout(); box.setSpacing(5)
            hdr = QHBoxLayout()
            hdr.addWidget(make_label(f"{j['id']} · {j['ros']}",C["t2"],12,True))
            hdr.addStretch()
            sv = make_label(f"{j['v']:.1f}°",C["a"],12,mono=True); hdr.addWidget(sv)
            s = QSlider(Qt.Horizontal)
            s.setRange(int(j["mn"]*10),int(j["mx"]*10)); s.setValue(int(j["v"]*10))
            s.setStyleSheet(slider_style(C["a"]))
            lims = QHBoxLayout()
            lims.addWidget(make_label(f"{j['mn']}°",C["t3"],9,mono=True))
            lims.addStretch()
            lims.addWidget(make_label(f"{j['mx']}°",C["t3"],9,mono=True))
            s.valueChanged.connect(lambda val,idx=i,lbl=sv: self._on_slider(idx,val/10,lbl))
            box.addLayout(hdr); box.addWidget(s); box.addLayout(lims); sl_ly.addLayout(box)
            self._sliders.append((s,sv))
        s_pnl.body.addWidget(sw); left_col.addWidget(s_pnl, 1)
        # Right Column (Commands, Limits, Tasks)
        right_col = QVBoxLayout()
        right_col.setSpacing(15)
        cmd_pnl = Panel("QUICK COMMANDS"); cg = QGridLayout(); cg.setSpacing(8)
        for i,(ic,lb,kd) in enumerate([
            ("▶","START","start"),("⏹","STOP","stop"),("⏸","PAUSE","normal"),
            ("⌂","HOME","normal"),("○","ZERO","normal"),("↺","RESET","normal")]):
            b = CmdButton(ic,lb,kd); b.clicked.connect(lambda _,c=lb: self._cmd(c))
            cg.addWidget(b,i//3,i%3)
        cmd_pnl.body.addLayout(cg); right_col.addWidget(cmd_pnl)
        lim_pnl = Panel("SPEED & TORQUE LIMITS", "SAFETY")
        for lbl,mn,mx,val,unit,color in [
            ("JOINT SPEED",    0, 100, 100,"%",   C["a2"]),
            ("TCP SPEED LIMIT",0,2500,2500,"mm/s",C["a2"]),
            ("TORQUE LIMIT",   0, 100, 100,"%",   C["a3"]),
            ("COLLISION SENSE",0, 100,  50,"%",   C["a3"])]:
            lh=QHBoxLayout()
            lh.addWidget(make_label(lbl,C["t2"],10,mono=True)); lh.addStretch()
            vl=make_label(f"{val}{unit}",C["t1"],14,True); lh.addWidget(vl)
            s=QSlider(Qt.Horizontal); s.setRange(mn,mx); s.setValue(val)
            s.setStyleSheet(slider_style(color))
            s.valueChanged.connect(lambda v,u=unit,l=vl: l.setText(f"{v}{u}"))
            lims2=QHBoxLayout()
            lims2.addWidget(make_label(str(mn),C["t3"],8,mono=True)); lims2.addStretch()
            lims2.addWidget(make_label(str(mx),C["t3"],8,mono=True))
            lim_pnl.body.addLayout(lh); lim_pnl.body.addWidget(s); lim_pnl.body.addLayout(lims2)
        right_col.addWidget(lim_pnl)
        task_pnl = Panel("PRESET TASKS","WEBOTS")
        for no,nm,st_ in [("01","Home Position","rdy"),("02","Pick & Place A→B","idle"),
                           ("03","Inspection Scan","idle"),("04","Calibration Routine","done"),
                           ("05","Custom Trajectory","idle")]:
            f=QFrame()
            f.setStyleSheet(f"QFrame{{background:rgba(0,0,0,0.25);border:1px solid {C['b']};border-radius:5px;}} QFrame:hover{{border-color:{C['bb']};}}")
            f.setCursor(Qt.PointingHandCursor)
            ly2=QHBoxLayout(f); ly2.setContentsMargins(12,10,12,10)
            ly2.addWidget(make_label(no,C["t3"],10,mono=True)); ly2.addWidget(make_label(nm,C["t1"],12))
            ly2.addStretch()
            sc,bg={"rdy":(C["a2"],"rgba(0,255,157,0.07)"),"done":(C["a"],"rgba(0,212,170,0.07)"),"idle":(C["t3"],"rgba(255,255,255,0.02)")}.get(st_,(C["t3"],"transparent"))
            sl2=QLabel(st_.upper()); sl2.setStyleSheet(f"color:{sc};background:{bg};border:1px solid {sc}40;border-radius:10px;font-size:9px;padding:3px 8px;font-family:'Share Tech Mono';")
            ly2.addWidget(sl2); task_pnl.body.addWidget(f)
        right_col.addWidget(task_pnl)
        right_col.addStretch()
        main_layout.addLayout(left_col, 1)
        main_layout.addLayout(right_col, 1)
    def _mode_style(self):
        return (f"QPushButton{{background:transparent;border:1px solid {C['b']};border-radius:4px;color:{C['t2']};font-family:'Share Tech Mono';font-size:11px;letter-spacing:.12em;padding:5px 10px;}}"
                f"QPushButton:checked{{background:rgba(0,212,170,0.09);border-color:{C['a']};color:{C['a']};}}"
                f"QPushButton:hover{{border-color:{C['a']};color:{C['a']};}}")
    def _on_slider(self, i, val, lbl):
        self._joints[i]["v"]=val; lbl.setText(f"{val:.1f}°")
        self._ros.publish_joints([j["v"] for j in self._joints])
    def update_canvas_from_ros(self, angles_rad):
        for i, (s, lbl) in enumerate(self._sliders):
            if i < len(angles_rad):
                deg = math.degrees(angles_rad[i])
                s.blockSignals(True); s.setValue(int(deg*10)); s.blockSignals(False)
                lbl.setText(f"{deg:.1f}°")
    def _joy_stop(self):
        self._jx=self._jy=self._jz=self._jwz=0.0
        self._joy_timer.stop(); self._ros.publish_twist(0,0,0,0)
    def _pub_twist(self):
        self._ros.publish_twist(self._jx*0.25,self._jy*0.25,self._jz*0.25,self._jwz*0.25)
    def _set_mode(self, active_btn, mode):
        for b in self._mode_btns: b.setChecked(b is active_btn)
        self._ros.publish_cmd(f"MODE:{mode}")
    def _cmd(self, c): self._ros.publish_cmd(c)
# ─────────────────────────────────────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────────────────────────────────────
class HMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Techman TM5-900 | Lightweight HMI v2.0")
        self.setMinimumSize(900,700); self.resize(1100,800)
        self.setStyleSheet(STYLE_BASE)
        import copy
        self._joints = copy.deepcopy(TM5_JOINTS)
        self._ros = RosWorker()
        self._ros.connection_changed.connect(self._on_ros_status)
        self._ros.joint_state_received.connect(self._on_joint_state)
        self._ros.log_message.connect(self._on_ros_log)
        self._ros.start()
        central = QWidget(); self.setCentralWidget(central)
        root = QVBoxLayout(central); root.setContentsMargins(0,0,0,0); root.setSpacing(0)
        self._tb = TopBar()
        self._tb.nav_changed.connect(self._nav)
        self._tb.estop_pressed.connect(self._estop)
        root.addWidget(self._tb)
        self._stack = QStackedWidget(); self._stack.setStyleSheet("background:transparent;")
        root.addWidget(self._stack, 1)
        self._dash = DashPage(self._ros, self._joints)
        self._ctrl = CtrlPage(self._ros, self._joints)
        self._stack.addWidget(_padded(self._dash))
        self._stack.addWidget(_padded(self._ctrl))
        self._pages = {"dash":0, "ctrl":1}
        self._clk_tmr = QTimer(self); self._clk_tmr.timeout.connect(self._tb.tick)
        self._clk_tmr.start(1000)
        self._up=0
        self._up_tmr = QTimer(self); self._up_tmr.timeout.connect(self._tick_up)
        self._up_tmr.start(1000)
        self._demo_t=0.0; self._demo_on=True
        self._demo_tmr = QTimer(self); self._demo_tmr.timeout.connect(self._tick_demo)
        self._demo_tmr.start(400)
        self._dash.log("Lightweight HMI v2.0 initialized","ok")
        self._dash.log("Web viewer removed. Run RViz2 separately for 3D view.","info")
        self._dash.log("Joint names: "+" · ".join(j["ros"] for j in TM5_JOINTS),"info")
    def _nav(self, pg): self._stack.setCurrentIndex(self._pages.get(pg,0))
    def _estop(self):
        self._dash.log("⚠ EMERGENCY STOP TRIGGERED","err")
        self._ros.publish_twist(0,0,0,0); self._ros.publish_cmd("ESTOP")
        self._dash._sc_state.set_value("E-STOP",C["red"])
    @pyqtSlot(bool)
    def _on_ros_status(self, online):
        self._tb.set_ros_status(online); self._demo_on = not online
    @pyqtSlot(list,list,list)
    def _on_joint_state(self, pos, vel, eff):
        self._dash.update_from_ros(pos, vel, eff)
        self._ctrl.update_canvas_from_ros(pos)
    @pyqtSlot(str,str)
    def _on_ros_log(self, msg, kind): self._dash.log(msg, kind)
    def _tick_up(self):
        self._up += 1; m,s = divmod(self._up,60)
        self._dash._sc_up.set_value(f"{m:02d}:{s:02d}")
    def _tick_demo(self):
        if not self._demo_on: return
        self._demo_t += 0.4; self._dash.update_demo(self._demo_t)
    def closeEvent(self, e):
        self._ros.stop(); self._ros.wait(2000); super().closeEvent(e)
def _padded(w):
    c = QWidget(); c.setStyleSheet("background:transparent;")
    ly = QVBoxLayout(c); ly.setContentsMargins(0,0,0,0); ly.setSpacing(0)
    ly.addWidget(w); return c
if __name__ == "__main__":
    sys.argv.append("--disable-gpu") 
    app = QApplication(sys.argv)
    app.setApplicationName("Techman TM5-900 Lightweight HMI")
    app.setStyleSheet(STYLE_BASE)
    
    # Fontların çalışması için ttf dosyalarının dizinde bulunduğundan emin ol.
    QFontDatabase.addApplicationFont("Rajdhani-Regular.ttf")
    QFontDatabase.addApplicationFont("Exo2-Regular.ttf")
    
    win = HMI(); win.show()
    sys.exit(app.exec_())
