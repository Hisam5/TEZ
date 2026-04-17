#!/usr/bin/env python3
"""
Techman TM5-900 Robot HMI — PyQt5  ·  with live ros3djs 3D viewer
==================================================================
3D view architecture:
  Webots  ──ROS 2──▶  /joint_states
                  ──▶  rosbridge_websocket (ws://localhost:9090)
                  ──▶  robot_description param  (URDF)
  QWebEngineView loads an embedded HTML page that uses ros3djs + Three.js
  to render the real robot mesh, with full orbit/pan/zoom camera control.

Pre-requisites (run once in your workspace):
  sudo apt install ros-humble-rosbridge-suite ros-humble-robot-state-publisher
  # serve URDF mesh files:
  pip install flask                   # or any static http server

Launch helpers (separate terminals):
  source ~/panda_ws/install/setup.bash
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # port 9090
  python3 -m http.server 8080 --directory /path/to/tm5_meshes   # port 8080

Then run:
  python3 tm5_hmi_3d.py

Dependencies:
  sudo apt install python3-pyqt5 python3-pyqt5.qtwebengine
"""

import sys, math, random, threading
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QSlider, QFrame,
    QScrollArea, QFileDialog, QProgressBar, QTextEdit, QStackedWidget,
)
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
    from PyQt5.QtCore import QUrl
    WEBENGINE_AVAILABLE = True
except ImportError:
    WEBENGINE_AVAILABLE = False
    QWebEngineView = QWidget          # fallback stub

from PyQt5.QtCore  import Qt, QTimer, QThread, pyqtSignal, pyqtSlot, QPoint
from PyQt5.QtGui   import (QColor, QPainter, QPen, QBrush, QPolygon, QPixmap,
                            QLinearGradient, QRadialGradient, QFontDatabase)
# Add --disable-web-security and --allow-file-access-from-files



# ─────────────────────────────────────────────────────────────────────────────
#  COLOUR PALETTE
# ─────────────────────────────────────────────────────────────────────────────

C = {
    "bg":    "#060b10",
    "card":  "#0d1825",
    "card2": "#0b1520",
    "b":     "#16293e",
    "bb":    "#1c3650",
    "a":     "#00d4aa",   # TM teal
    "a2":    "#00ff9d",
    "a3":    "#ff6b35",
    "t1":    "#c8dced",
    "t2":    "#4e7290",
    "t3":    "#273d52",
    "red":   "#ff3d5a",
    "warn":  "#ffb830",
    "ok":    "#00e676",
}

STYLE_BASE = f"""
QMainWindow, QWidget {{
    background:{C['bg']}; color:{C['t1']};
    font-family:'Exo 2','Rajdhani',sans-serif; font-size:11px;
}}
QScrollBar:vertical   {{ background:transparent; width:4px; border-radius:2px; }}
QScrollBar::handle:vertical {{ background:{C['bb']}; border-radius:2px; min-height:20px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0; }}
QScrollBar:horizontal {{ background:transparent; height:4px; }}
QScrollBar::handle:horizontal {{ background:{C['bb']}; border-radius:2px; }}
QScrollBar::add-line:horizontal,QScrollBar::sub-line:horizontal {{ width:0; }}
QSlider::groove:horizontal {{ height:4px; background:rgba(255,255,255,0.07); border-radius:2px; }}
QSlider::handle:horizontal {{
    width:13px; height:13px; margin:-5px 0;
    border-radius:7px; background:{C['a']}; border:2px solid {C['bg']};
}}
QSlider::sub-page:horizontal {{ background:{C['a']}; border-radius:2px; }}
"""

# ─────────────────────────────────────────────────────────────────────────────
#  TM5-900 JOINT DEFINITIONS
# ─────────────────────────────────────────────────────────────────────────────
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
            self._pub_j = self._node.create_publisher(JointTrajectory,
                "/tm_arm_controller/joint_trajectory", 10)
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
#  HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def make_label(text, color=C["t1"], size=11, bold=False, mono=False):
    lbl = QLabel(text)
    fam = "'Share Tech Mono'" if mono else "'Rajdhani',sans-serif"
    lbl.setStyleSheet(
        f"color:{color}; font-size:{size}px; font-weight:{'bold' if bold else 'normal'};"
        f"font-family:{fam}; background:transparent;")
    return lbl

def slider_style(color):
    return (
        f"QSlider::groove:horizontal{{height:4px;background:rgba(255,255,255,0.07);border-radius:2px;}}"
        f"QSlider::handle:horizontal{{width:13px;height:13px;margin:-5px 0;border-radius:7px;"
        f"background:{color};border:2px solid {C['bg']};}}"
        f"QSlider::sub-page:horizontal{{background:{color};border-radius:2px;}}")


# ─────────────────────────────────────────────────────────────────────────────
#  PANEL
# ─────────────────────────────────────────────────────────────────────────────
class Panel(QFrame):
    def __init__(self, title, tag="", parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"Panel{{background:{C['card']};border:1px solid {C['b']};border-radius:7px;}}")
        outer = QVBoxLayout(self); outer.setContentsMargins(0,0,0,0); outer.setSpacing(0)
        hdr = QFrame(); hdr.setFixedHeight(24)
        hdr.setStyleSheet(
            f"background:rgba(0,0,0,0.2);border-bottom:1px solid {C['b']};border-radius:0;")
        hl = QHBoxLayout(hdr); hl.setContentsMargins(11,0,11,0)
        hl.addWidget(make_label(title, C["a"], 10, True))
        hl.addStretch()
        if tag: hl.addWidget(make_label(tag, C["t3"], 8, mono=True))
        outer.addWidget(hdr)
        self.content = QWidget(); self.content.setStyleSheet("background:transparent;")
        self.body = QVBoxLayout(self.content)
        self.body.setContentsMargins(6,5,6,5); self.body.setSpacing(4)
        outer.addWidget(self.content)

    def paintEvent(self, e):
        super().paintEvent(e)
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        g = QLinearGradient(0,0,self.width(),0)
        g.setColorAt(0, QColor(0,212,170,0)); g.setColorAt(0.5, QColor(0,212,170,97))
        g.setColorAt(1, QColor(0,212,170,0))
        p.setPen(QPen(QBrush(g),2)); p.drawLine(0,1,self.width(),1)


# ─────────────────────────────────────────────────────────────────────────────
#  STAT CARD
# ─────────────────────────────────────────────────────────────────────────────
class StatCard(QFrame):
    ACCENT = {"g":C["a2"],"b":C["a"],"o":C["a3"],"r":C["red"]}
    def __init__(self, label, value, sub, kind="b", parent=None):
        super().__init__(parent); self._kind = kind
        self.setMinimumHeight(50); self.setMaximumHeight(125)
        self.setStyleSheet(
            f"StatCard{{background:{C['card2']};border:1px solid {C['b']};border-radius:6px;}}")
        ly = QVBoxLayout(self); ly.setContentsMargins(10,8,10,8); ly.setSpacing(2)
        self.lbl = make_label(label,C["t3"],10,mono=True)
        self.val = make_label(value,C["t1"],18,True)
        self.sub = make_label(sub,  C["t3"], 8,mono=True)
        ly.addWidget(self.lbl); ly.addWidget(self.val); ly.addWidget(self.sub)
    def set_value(self, v, color=None):
        self.val.setText(str(v))
        if color: self.val.setStyleSheet(
            f"color:{color};font-size:18px;font-weight:bold;background:transparent;")
    def paintEvent(self, e):
        super().paintEvent(e); p = QPainter(self)
        p.setPen(Qt.NoPen); p.setBrush(QColor(self.ACCENT.get(self._kind,C["a"])))
        r = self.rect(); p.drawRoundedRect(r.x()+1,r.bottom()-3,r.width()-2,2,1,1)


# ─────────────────────────────────────────────────────────────────────────────
#  JOINT CARD
# ─────────────────────────────────────────────────────────────────────────────
class JointCard(QFrame):
    def __init__(self, jid, ros_name, display_name, mn, mx, parent=None):
        super().__init__(parent); self._mn=mn; self._mx=mx
        self.setMaximumHeight(90)
        self.setStyleSheet(
            f"JointCard{{background:{C['card2']};border:1px solid {C['b']};border-radius:7px;}}")
        ly = QVBoxLayout(self); ly.setContentsMargins(5,3,5,3); ly.setSpacing(2)
        hr = QHBoxLayout()
        left = QVBoxLayout(); left.setSpacing(0)
        self.id_lbl = make_label(jid, C["a"], 13, True)
        self.nm_lbl = make_label(ros_name.upper(), C["t3"], 7, mono=True)
        left.addWidget(self.id_lbl); left.addWidget(self.nm_lbl)
        hr.addLayout(left); hr.addStretch()
        self.dot = QLabel(); self.dot.setFixedSize(8,8); self._set_dot("ok")
        hr.addWidget(self.dot); ly.addLayout(hr)
        self.disp_lbl = make_label(display_name, C["t2"], 8); ly.addWidget(self.disp_lbl)
        self.pos_lbl  = make_label("0.0°", C["t1"], 12, True); ly.addWidget(self.pos_lbl)
        self.pos_bar  = QProgressBar()
        self.pos_bar.setRange(0,1000); self.pos_bar.setValue(500)
        self.pos_bar.setTextVisible(False); self.pos_bar.setFixedHeight(4)
        self.pos_bar.setStyleSheet(
            f"QProgressBar{{background:rgba(255,255,255,0.05);border-radius:2px;border:none;}}"
            f"QProgressBar::chunk{{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            f"stop:0 {C['a']},stop:1 {C['a2']});border-radius:2px;}}")
        ly.addWidget(self.pos_bar)
        row = QHBoxLayout()
        vl = QVBoxLayout(); vl.setSpacing(0)
        vl.addWidget(make_label("VEL",C["t3"],7,mono=True))
        self.vel_val = make_label("0.00°/s",C["a2"],8,True); vl.addWidget(self.vel_val)
        el = QVBoxLayout(); el.setSpacing(0)
        el.addWidget(make_label("EFF",C["t3"],7,mono=True))
        self.eff_val = make_label("0.00Nm",C["a3"],8,True); el.addWidget(self.eff_val)
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

    def paintEvent(self, e):
        super().paintEvent(e); p = QPainter(self)
        g = QLinearGradient(0,0,self.width(),0)
        g.setColorAt(0,QColor(0,212,170,0)); g.setColorAt(0.5,QColor(0,212,170,77))
        g.setColorAt(1,QColor(0,212,170,0))
        p.setPen(QPen(QBrush(g),2)); p.drawLine(0,1,self.width(),1)


# ─────────────────────────────────────────────────────────────────────────────
#  TCP CELL
# ─────────────────────────────────────────────────────────────────────────────
class TcpCell(QFrame):
    def __init__(self, axis, color, unit="mm", parent=None):
        super().__init__(parent); self._unit=unit
        self.setStyleSheet(
            f"TcpCell{{background:rgba(0,0,0,0.25);border:1px solid {C['b']};border-radius:5px;}}")
        ly = QVBoxLayout(self); ly.setContentsMargins(5,4,5,4); ly.setSpacing(1)
        ly.addWidget(make_label(axis,color,8,mono=True))
        self.val = make_label(f"0.0 {unit}",C["t1"],10,True)
        self.bar = QProgressBar(); self.bar.setRange(0,1000); self.bar.setValue(500)
        self.bar.setTextVisible(False); self.bar.setFixedHeight(2)
        self.bar.setStyleSheet(
            f"QProgressBar{{background:rgba(255,255,255,0.04);border-radius:1px;border:none;}}"
            f"QProgressBar::chunk{{background:{color};border-radius:1px;}}")
        ly.addWidget(self.val); ly.addWidget(self.bar)

    def set_value(self, v, mn=-900, mx=900):
        self.val.setText(f"{v:.1f} {self._unit}")
        pct = int((v-mn)/(mx-mn)*1000); self.bar.setValue(max(0,min(1000,pct)))


# ─────────────────────────────────────────────────────────────────────────────
#  CMD BUTTON
# ─────────────────────────────────────────────────────────────────────────────
class CmdButton(QPushButton):
    def __init__(self, icon, label, kind="normal", parent=None):
        super().__init__(parent)
        ly = QVBoxLayout(self); ly.setContentsMargins(4,9,4,9); ly.setAlignment(Qt.AlignCenter)
        il = QLabel(icon); il.setAlignment(Qt.AlignCenter)
        il.setStyleSheet("font-size:14px;background:transparent;")
        ll = QLabel(label); ll.setAlignment(Qt.AlignCenter)
        ll.setStyleSheet("font-size:7.5px;letter-spacing:0.15em;"
                         "font-family:'Share Tech Mono';background:transparent;")
        ly.addWidget(il); ly.addWidget(ll)
        fc,bg = {"start":(C["a2"],"rgba(0,255,157,0.07)"),"stop":(C["red"],"rgba(255,61,90,0.05)"),
                 "normal":(C["t2"],C["card2"])}.get(kind,(C["t2"],C["card2"]))
        self.setStyleSheet(
            f"CmdButton{{color:{fc};background:{bg};border:1px solid {C['b']};"
            f"border-radius:5px;min-height:36px;}}"
            f"CmdButton:hover{{border-color:{C['a']};color:{C['a']};background:rgba(0,212,170,0.09);}}")


# ─────────────────────────────────────────────────────────────────────────────
#  LOG WIDGET
# ─────────────────────────────────────────────────────────────────────────────
class LogWidget(QTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent); self.setReadOnly(True)
        self.setStyleSheet(
            f"QTextEdit{{background:#030810;border:1px solid {C['b']};"
            f"border-radius:4px;font-family:'Share Tech Mono';font-size:8px;color:{C['t2']};}}")
        self._count = 0

    def add(self, msg, kind="info"):
        col = {"ok":C["ok"],"warn":C["warn"],"err":C["red"],"info":C["a"]}.get(kind,C["a"])
        ts  = datetime.now().strftime("%H:%M:%S"); self._count += 1
        self.append(f'<span style="color:{C["t3"]}">{ts}</span> '
                    f'<span style="color:{col}">{msg}</span>')
        sb = self.verticalScrollBar(); sb.setValue(sb.maximum())
        return self._count


# ─────────────────────────────────────────────────────────────────────────────
#  ███████████████████████████████████████████████████████████████████████████
#  3D ROS3D VIEWER WIDGET  ← THE KEY COMPONENT
#  ███████████████████████████████████████████████████████████████████████████
#
#  How it works:
#  ┌─────────────────────────────────────────────────────────────────────────┐
#  │  QWebEngineView                                                         │
#  │    └─ Embedded HTML (no file on disk, loaded via setHtml())             │
#  │         ├─ Three.js      — 3D renderer                                  │
#  │         ├─ roslib.js     — WebSocket bridge to ROS 2                    │
#  │         ├─ ros3djs       — URDF/mesh rendering, TF frames               │
#  │         └─ OrbitControls — mouse orbit/pan/zoom (built into ros3djs)    │
#  │                                                                         │
#  │  Python ──JS bridge──▶  setJointAngles([…rad…])                        │
#  │  via page().runJavaScript()  — called from _on_joint_state signal       │
#  └─────────────────────────────────────────────────────────────────────────┘
#
#  Camera presets exposed as Python methods so the HMI buttons can call them.
# ─────────────────────────────────────────────────────────────────────────────
class Ros3DViewerWidget(QWidget):
    """
    Embeds a ros3djs 3D robot viewer inside a QWebEngineView.

    Public API
    ----------
    set_joint_angles(angles_rad: list[float])
        Push new joint angles; the 3D model updates immediately.
    set_view_preset(name: str)
        Move the camera to a named preset: 'front','side','top','iso'.
    set_ros_url(url: str)
        Change the rosbridge WebSocket URL (default ws://localhost:9090).
    set_urdf_url(url: str)
        Change the mesh/URDF HTTP base URL (default http://localhost:8080).
    """

    status_changed = pyqtSignal(str, str)    # (message, color_hex)

    # rosbridge + mesh server URLs (change if your ports differ)
    ROS_WS_URL   = "ws://localhost:9090"
    URDF_BASE_URL = "http://localhost:8080"
    URDF_PARAM   = "robot_description"
    FIXED_FRAME  = "base"          # TF fixed frame — change to your robot's base link

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ready = False
        ly = QVBoxLayout(self); ly.setContentsMargins(0,0,0,0); ly.setSpacing(0)

        if not WEBENGINE_AVAILABLE:
            # Friendly fallback when QtWebEngine is not installed
            fb = QLabel(
                "⚠  PyQt5.QtWebEngineWidgets not found.\n\n"
                "Install with:\n  sudo apt install python3-pyqt5.qtwebengine\n\n"
                "Then restart the HMI."
            )
            fb.setAlignment(Qt.AlignCenter)
            fb.setStyleSheet(
                f"color:{C['warn']};background:#040c14;font-family:'Share Tech Mono';"
                f"font-size:10px;border:2px dashed {C['warn']};border-radius:6px;padding:20px;")
            ly.addWidget(fb)
            return

        self._web = QWebEngineView(self)

        # Allow local resource requests, JS, etc.
        s = self._web.settings()
        s.setAttribute(QWebEngineSettings.JavascriptEnabled,            True)
        s.setAttribute(QWebEngineSettings.LocalStorageEnabled,          True)
        s.setAttribute(QWebEngineSettings.AllowRunningInsecureContent,  True)
        s.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)

        ly.addWidget(self._web)

        # Load page — setHtml() with a base URL that allows fetching CDN scripts
        self._web.loadFinished.connect(self._on_load_finished)
        self._web.setHtml(self._build_html(),
                          QUrl("http://localhost/"))   # base URL for CORS

        self.status_changed.emit("CONNECTING TO ROSBRIDGE...", C["t3"])

    # ── Build the full HTML page ──────────────────────────────────────────────
    def _build_html(self):
        return f"""<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8"/>
<style>
  * {{ margin:0; padding:0; box-sizing:border-box; }}
  html, body {{ width:100%; height:100%; background:#040c14; overflow:hidden; }}
  #viewer {{ width:100%; height:100%; position:relative; }}

  /* ── status overlay ──────────────────────────────── */
  #status {{
    position:absolute; top:7px; right:10px; z-index:99;
    font-family:'Share Tech Mono',monospace; font-size:9px;
    color:#273d52; letter-spacing:.12em; pointer-events:none;
    background:rgba(4,12,20,0.72); padding:3px 8px; border-radius:4px;
    border:1px solid rgba(0,212,170,0.15);
  }}

  /* ── camera preset buttons ───────────────────────── */
  #cam-btns {{
    position:absolute; bottom:10px; left:10px; z-index:99;
    display:flex; gap:5px;
  }}
  .cam-btn {{
    background:rgba(0,212,170,0.10); border:1px solid rgba(0,212,170,0.35);
    border-radius:4px; color:#00d4aa; font-family:'Share Tech Mono',monospace;
    font-size:8px; padding:4px 9px; cursor:pointer; letter-spacing:.1em;
    transition:background 0.15s;
  }}
  .cam-btn:hover {{ background:rgba(0,212,170,0.28); }}

  /* ── coord-axis legend ───────────────────────────── */
  #axis-legend {{
    position:absolute; bottom:10px; right:10px; z-index:99;
    font-family:'Share Tech Mono',monospace; font-size:8px;
    display:flex; flex-direction:column; gap:2px; pointer-events:none;
  }}
  .ax {{ display:flex; align-items:center; gap:4px; }}
  .ax-dot {{ width:8px; height:8px; border-radius:50%; display:inline-block; }}
</style>
</head>
<body>
<div id="viewer"></div>
<div id="status">INITIALIZING…</div>

<div id="cam-btns">
  <button class="cam-btn" onclick="camPreset('iso')">ISO</button>
  <button class="cam-btn" onclick="camPreset('front')">FRONT</button>
  <button class="cam-btn" onclick="camPreset('side')">SIDE</button>
  <button class="cam-btn" onclick="camPreset('top')">TOP</button>
  <button class="cam-btn" onclick="resetView()">RESET</button>
</div>

<div id="axis-legend">
  <div class="ax"><span class="ax-dot" style="background:#ff4d6d"></span>X</div>
  <div class="ax"><span class="ax-dot" style="background:#00ff9d"></span>Y</div>
  <div class="ax"><span class="ax-dot" style="background:#00d4aa"></span>Z</div>
</div>

<!-- ── CDN scripts ─────────────────────────────────────────────────────── -->
<script src="https://cdn.jsdelivr.net/npm/three@0.89.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.4/lib/eventemitter2.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
<script src="https://cdn.jsdelivr.net/gh/RobotWebTools/ros3djs@develop/build/ros3d.min.js"></script>

<script>
// ── Configuration (injected from Python) ──────────────────────────────
var ROS_URL   = '{self.ROS_WS_URL}';
var URDF_URL  = '{self.URDF_BASE_URL}';
var URDF_PARAM = '{self.URDF_PARAM}';
var FIXED_FRAME = '{self.FIXED_FRAME}';

// ── Globals ───────────────────────────────────────────────────────────
var viewer, tfClient, ros;
var jointStateTopicPub;

function st(msg, col) {{
  var el = document.getElementById('status');
  el.innerText = msg;
  el.style.color = col || '#273d52';
}}

// ── ROS connection ────────────────────────────────────────────────────
function connectRos() {{
  ros = new ROSLIB.Ros({{ url: ROS_URL }});
  ros.on('connection', function() {{
    st('ROS 2 ● ONLINE', '#00e676');
    if (!viewer) init3D();
  }});
  ros.on('error', function(e) {{
    st('ROSBRIDGE ERROR — retrying…', '#ff3d5a');
    setTimeout(connectRos, 3000);
  }});
  ros.on('close', function() {{
    st('DISCONNECTED — retrying…', '#ffb830');
    setTimeout(connectRos, 3000);
  }});
}}

// ── 3D scene setup ────────────────────────────────────────────────────
function init3D() {{
  var el = document.getElementById('viewer');

  viewer = new ROS3D.Viewer({{
    divID:     'viewer',
    width:     el.offsetWidth  || window.innerWidth,
    height:    el.offsetHeight || window.innerHeight,
    antialias: true,
    background:'#040c14',
    cameraPose:{{ x:1.4, y:1.0, z:1.4 }}
  }});

  // ── Grid ────────────────────────────────────────────────────────────
  viewer.addObject(new ROS3D.Grid({{
    color:'#0a1f2e', cellSize:0.1, num_cells:30
  }}));

  // ── Lighting ─────────────────────────────────────────────────────────
  var amb = new THREE.AmbientLight(0xffffff, 0.55);
  viewer.scene.add(amb);
  var dir1 = new THREE.DirectionalLight(0xffffff, 0.85);
  dir1.position.set(2, 3, 2); viewer.scene.add(dir1);
  var dir2 = new THREE.DirectionalLight(0x88ccff, 0.30);
  dir2.position.set(-2, 1, -1); viewer.scene.add(dir2);

  // ── TF client ────────────────────────────────────────────────────────
  tfClient = new ROSLIB.TFClient({{
    ros:         ros,
    fixedFrame:  FIXED_FRAME,
    angularThres:0.005,
    transThres:  0.002,
    rate:        30.0
  }});

  // ── URDF robot model ─────────────────────────────────────────────────
  new ROS3D.UrdfClient({{
    ros:        ros,
    tfClient:   tfClient,
    path:       URDF_URL,
    rootObject: viewer.scene,
    param:      URDF_PARAM,
    loader:     ROS3D.COLLADA_LOADER
  }});

  // ── Axes helper (world origin) ────────────────────────────────────────
  var axes = new THREE.AxesHelper(0.3);
  viewer.scene.add(axes);

  // ── Resize observer — keeps viewer filling its container ──────────────
  new ResizeObserver(function() {{
    viewer.resize(el.offsetWidth, el.offsetHeight);
  }}).observe(el);

  st('frame: ' + FIXED_FRAME, '#273d52');
}}

// ── Camera presets ────────────────────────────────────────────────────
var PRESETS = {{
  iso:   {{ x:1.4,  y:1.0, z:1.4  }},
  front: {{ x:0.0,  y:0.8, z:2.2  }},
  side:  {{ x:2.2,  y:0.8, z:0.0  }},
  top:   {{ x:0.01, y:2.5, z:0.01 }}
}};

function camPreset(name) {{
  if (!viewer) return;
  var p = PRESETS[name] || PRESETS.iso;
  viewer.camera.position.set(p.x, p.y, p.z);
  viewer.camera.lookAt(new THREE.Vector3(0, 0.5, 0));
  if (viewer.cameraControls) viewer.cameraControls.update();
}}

function resetView() {{
  camPreset('iso');
}}

// ── Python→JS bridge — called by Python via runJavaScript() ───────────
// Receives joint angles in RADIANS as a JSON array string, e.g.
//   '[0.0, -0.5, 1.2, 0.0, 0.8, 0.0]'
// and publishes them back to /joint_states so ros3djs can update TF.
// (robot_state_publisher must be running to convert joints → TF frames)
window.setJointAngles = function(anglesJson) {{
  if (!ros || !ros.isConnected) return;
  try {{
    var angles = JSON.parse(anglesJson);
    var msg = new ROSLIB.Message({{
      header: {{ stamp: {{ sec:0, nanosec:0 }}, frame_id:'' }},
      name:   [
        'shoulder_1_joint','shoulder_2_joint','elbow_joint',
        'wrist_1_joint','wrist_2_joint','wrist_3_joint'
      ],
      position: angles,
      velocity: [0,0,0,0,0,0],
      effort:   [0,0,0,0,0,0]
    }});
    if (!window._jsPub) {{
      window._jsPub = new ROSLIB.Topic({{
        ros:          ros,
        name:         '/joint_states',
        messageType:  'sensor_msgs/JointState'
      }});
    }}
    window._jsPub.publish(msg);
  }} catch(e) {{ console.error('setJointAngles error', e); }}
}};

// ── Camera control — called by Python ────────────────────────────────
window.setCameraPreset = function(name) {{ camPreset(name); }};

// ── Boot ──────────────────────────────────────────────────────────────
window.addEventListener('load', connectRos);
</script>
</body>
</html>"""

    # ── Python API ───────────────────────────────────────────────────────────
    def _on_load_finished(self, ok):
        self._ready = ok
        if not ok:
            self.status_changed.emit("PAGE LOAD FAILED", C["red"])

    def set_joint_angles(self, angles_rad: list):
        """Push new joint angles (radians) into the 3D viewer."""
        if not WEBENGINE_AVAILABLE or not self._ready:
            return
        import json
        js = f"if(window.setJointAngles){{setJointAngles('{json.dumps(angles_rad)}');}}"
        self._web.page().runJavaScript(js)

    def set_view_preset(self, name: str):
        """Move camera to a preset: 'iso','front','side','top'."""
        if not WEBENGINE_AVAILABLE or not self._ready:
            return
        self._web.page().runJavaScript(
            f"if(window.setCameraPreset){{setCameraPreset('{name}');}}")

    def set_ros_url(self, url: str):
        self.ROS_WS_URL = url

    def set_urdf_url(self, url: str):
        self.URDF_BASE_URL = url


# ─────────────────────────────────────────────────────────────────────────────
#  3D VIEWER FRAME  (header + camera buttons + Ros3DViewerWidget)
# ─────────────────────────────────────────────────────────────────────────────
class ViewerFrame(QFrame):
    """
    Wraps Ros3DViewerWidget with a styled header and camera-preset buttons
    that are also accessible from Python (for keyboard shortcuts etc.).
    """
    def __init__(self, title="TM5-900 3D VIEW", parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"ViewerFrame{{background:#040c14;border:1px solid {C['b']};border-radius:7px;}}")
        ly = QVBoxLayout(self); ly.setContentsMargins(0,0,0,0); ly.setSpacing(0)

        # ── header ──────────────────────────────────────────────────────────
        hdr = QFrame(); hdr.setFixedHeight(32)
        hdr.setStyleSheet(
            "background:rgba(4,12,20,0.92);"
            "border-bottom:1px solid rgba(0,212,170,0.12);")
        hl = QHBoxLayout(hdr); hl.setContentsMargins(10,0,10,0); hl.setSpacing(8)
        hl.addWidget(make_label(title, C["a"], 9, True))
        hl.addStretch()

        # camera preset pill-buttons inside header
        for preset, label in [("iso","ISO"),("front","FRONT"),("side","SIDE"),("top","TOP")]:
            b = QPushButton(label)
            b.setFixedHeight(20)
            b.setStyleSheet(
                f"QPushButton{{background:rgba(0,212,170,0.07);border:1px solid rgba(0,212,170,0.25);"
                f"border-radius:3px;color:{C['t2']};font-family:'Share Tech Mono';"
                f"font-size:7px;padding:0 7px;letter-spacing:.1em;}}"
                f"QPushButton:hover{{background:rgba(0,212,170,0.20);color:{C['a']};"
                f"border-color:{C['a']};}}")
            b.clicked.connect(lambda _, p=preset: self.viewer.set_view_preset(p))
            hl.addWidget(b)

        # connection status label
        self._status = make_label("CONNECTING…", C["t3"], 8, mono=True)
        hl.addWidget(self._status)
        ly.addWidget(hdr)

        # ── 3D viewer ────────────────────────────────────────────────────────
        self.viewer = Ros3DViewerWidget(self)
        self.viewer.status_changed.connect(self._on_status)
        ly.addWidget(self.viewer, 1)

        # thin teal scan line at bottom
        scan = QLabel(); scan.setFixedHeight(1)
        scan.setStyleSheet(
            "background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            "stop:0 transparent,stop:0.5 rgba(0,212,170,60),stop:1 transparent);")
        ly.addWidget(scan)

    def _on_status(self, msg, col):
        self._status.setText(msg)
        self._status.setStyleSheet(
            f"color:{col};font-size:8px;font-family:'Share Tech Mono';background:transparent;")

    # convenience passthrough
    def set_joint_angles(self, angles_rad):
        self.viewer.set_joint_angles(angles_rad)

    def paintEvent(self, e):
        super().paintEvent(e)
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        g = QLinearGradient(0,0,self.width(),0)
        g.setColorAt(0,QColor(0,212,170,0)); g.setColorAt(0.5,QColor(0,212,170,60))
        g.setColorAt(1,QColor(0,212,170,0))
        p.setPen(QPen(QBrush(g),1))
        p.drawLine(0,self.height()-1,self.width(),self.height()-1)


# ─────────────────────────────────────────────────────────────────────────────
#  TOP BAR
# ─────────────────────────────────────────────────────────────────────────────
class TopBar(QFrame):
    nav_changed   = pyqtSignal(str)
    estop_pressed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent); self.setFixedHeight(52)
        self.setStyleSheet(
            f"TopBar{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            f"stop:0 #0b1926,stop:1 {C['bg']});border-bottom:1px solid {C['b']};}}")
        main = QHBoxLayout(self); main.setContentsMargins(18,0,18,0)

        logo = QLabel("TM"); logo.setFixedSize(28,28); logo.setAlignment(Qt.AlignCenter)
        logo.setStyleSheet(
            f"color:{C['a']};border:2px solid {C['a']};border-radius:5px;"
            f"font-family:'Rajdhani';font-size:12px;font-weight:700;background:transparent;")
        name = QLabel(f"TM5-900 <span style='color:{C['a']}'> CONTROL</span>")
        name.setTextFormat(Qt.RichText)
        name.setStyleSheet(
            f"font-family:'Rajdhani';font-size:18px;font-weight:700;"
            f"color:{C['t1']};background:transparent;")
        sub = QLabel("TECHMAN ROBOT · 6-DOF COBOT · HMI v2.0  |  tm_arm_controller · ros3djs viewer")
        sub.setStyleSheet(
            f"font-family:'Share Tech Mono';font-size:8px;color:{C['t3']};"
            f"letter-spacing:.18em;background:transparent;")
        info_col = QVBoxLayout(); info_col.setSpacing(0)
        info_col.addWidget(name); info_col.addWidget(sub)
        main.addWidget(logo); main.addLayout(info_col); main.addStretch()

        self._btn_dash = QPushButton("DASHBOARD")
        self._btn_ctrl = QPushButton("CONTROL")
        for btn, pg in [(self._btn_dash,"dash"),(self._btn_ctrl,"ctrl")]:
            btn.setFixedHeight(28)
            btn.clicked.connect(lambda _, p=pg: self._nav(p))
            main.addWidget(btn)
        self._active = "dash"; self._update_nav(); main.addSpacing(12)

        self.pill = QLabel("● CONNECTING")
        self.pill.setStyleSheet(
            f"background:rgba(255,61,90,0.07);border:1px solid rgba(255,61,90,0.22);"
            f"border-radius:12px;color:{C['red']};font-family:'Share Tech Mono';"
            f"font-size:9px;padding:3px 10px;")
        main.addWidget(self.pill)

        self.clock = QLabel("--:--:--")
        self.clock.setStyleSheet(
            f"font-family:'Share Tech Mono';font-size:11px;color:{C['t2']};"
            f"min-width:66px;background:transparent;")
        main.addWidget(self.clock)

        es = QPushButton("⬛ E-STOP")
        es.setStyleSheet(
            f"QPushButton{{background:rgba(255,61,90,0.1);border:2px solid {C['red']};"
            f"border-radius:4px;color:{C['red']};font-family:'Rajdhani';"
            f"font-size:11px;font-weight:700;letter-spacing:.15em;padding:5px 13px;}}"
            f"QPushButton:hover{{background:rgba(255,61,90,0.22);}}"
            f"QPushButton:pressed{{background:{C['red']};color:#fff;}}")
        es.clicked.connect(self.estop_pressed); main.addWidget(es)

    def _nav_style(self, active=False):
        if active:
            return (f"QPushButton{{padding:5px 16px;background:rgba(0,212,170,0.09);"
                    f"border:1px solid {C['a']};border-radius:4px;color:{C['a']};"
                    f"font-family:'Rajdhani';font-size:11px;font-weight:600;letter-spacing:.1em;}}")
        return (f"QPushButton{{padding:5px 16px;background:transparent;"
                f"border:1px solid {C['b']};border-radius:4px;color:{C['t2']};"
                f"font-family:'Rajdhani';font-size:11px;font-weight:600;letter-spacing:.1em;}}"
                f"QPushButton:hover{{border-color:{C['a']};color:{C['a']};}}")

    def _update_nav(self):
        self._btn_dash.setStyleSheet(self._nav_style(self._active=="dash"))
        self._btn_ctrl.setStyleSheet(self._nav_style(self._active=="ctrl"))

    def _nav(self, pg):
        self._active = pg; self._update_nav(); self.nav_changed.emit(pg)

    def set_ros_status(self, online):
        if online:
            self.pill.setText("● ROS 2 ONLINE")
            self.pill.setStyleSheet(
                f"background:rgba(0,230,118,0.07);border:1px solid rgba(0,230,118,0.22);"
                f"border-radius:12px;color:{C['ok']};font-family:'Share Tech Mono';"
                f"font-size:9px;padding:3px 10px;")
        else:
            self.pill.setText("● OFFLINE")
            self.pill.setStyleSheet(
                f"background:rgba(255,61,90,0.07);border:1px solid rgba(255,61,90,0.22);"
                f"border-radius:12px;color:{C['red']};font-family:'Share Tech Mono';"
                f"font-size:9px;padding:3px 10px;")

    def tick(self): self.clock.setText(datetime.now().strftime("%H:%M:%S"))

    def paintEvent(self, e):
        super().paintEvent(e); p = QPainter(self)
        g = QLinearGradient(0,self.height(),self.width(),self.height())
        g.setColorAt(0,QColor(0,212,170,0)); g.setColorAt(0.5,QColor(0,212,170,82))
        g.setColorAt(1,QColor(0,212,170,0))
        p.setPen(QPen(QBrush(g),1))
        p.drawLine(0,self.height()-1,self.width(),self.height()-1)


# ─────────────────────────────────────────────────────────────────────────────
#  DASHBOARD PAGE
# ─────────────────────────────────────────────────────────────────────────────
class DashPage(QWidget):
    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent); self._ros=ros; self._joints=joints; self._build_ui()

    def _build_ui(self):
        g = QGridLayout(self); g.setContentsMargins(0,0,0,0); g.setSpacing(5)
        g.setColumnStretch(1,1); g.setColumnMinimumWidth(0,650); g.setColumnMinimumWidth(2,195)

        # ── LEFT: 3D viewer (full height) ────────────────────────────────────
        self._viewer_frame = ViewerFrame("TM5-900  ·  6-DOF  ·  LIVE 3D VIEW")
        g.addWidget(self._viewer_frame, 0, 0, 3, 1)

        # ── CENTER ───────────────────────────────────────────────────────────
        g.addWidget(self._make_stats(),  0, 1)
        g.addWidget(self._make_tcp(),    1, 1)
        g.addWidget(self._make_joints(), 2, 1)

        # ── RIGHT ────────────────────────────────────────────────────────────
        g.addWidget(self._make_right(), 0, 2, 3, 1)

    def _make_stats(self):
        w = QWidget(); w.setStyleSheet("background:transparent;"); w.setMaximumHeight(267)
        g = QGridLayout(w); g.setSpacing(6); g.setContentsMargins(0,0,0,0)
        self._sc_state = StatCard("ROBOT STATE",    "STANDBY","READY FOR COMMAND","g")
        self._sc_spd   = StatCard("TCP SPEED",      "0 mm/s", "MAX 2500 mm/s",   "b")
        self._sc_prog  = StatCard("ACTIVE PROGRAM", "NONE",   "NO FILE LOADED",  "b")
        self._sc_reach = StatCard("REACH",          "900 mm", "TM5-900 MAX",     "o")
        self._sc_up    = StatCard("UPTIME",         "00:00",  "SESSION TIME",    "g")
        self._sc_err   = StatCard("ERRORS",         "0",      "ACTIVE FAULTS",   "r")
        for i, c in enumerate([self._sc_state,self._sc_spd,self._sc_prog,
                                self._sc_reach,self._sc_up,self._sc_err]):
            g.addWidget(c,i//3,i%3); g.setColumnStretch(i%3,1)
        return w

    def _make_tcp(self):
        pnl = Panel("TCP POSITION","CARTESIAN"); pnl.setMaximumHeight(267)
        grid = QGridLayout(); grid.setSpacing(3)
        self._tcp = {
            "X": TcpCell("X AXIS","#ff4d6d"),  "Y": TcpCell("Y AXIS",C["a2"]),
            "Z": TcpCell("Z AXIS",C["a"]),      "RX":TcpCell("RX",C["a3"],"°"),
            "RY":TcpCell("RY",C["a3"],"°"),     "RZ":TcpCell("RZ",C["a3"],"°"),
        }
        for i,(_, w) in enumerate(self._tcp.items()): grid.addWidget(w, i//3, i%3)
        pnl.body.addLayout(grid); return pnl

    def _make_joints(self):
        w = QWidget(); w.setStyleSheet("background:transparent;")
        from PyQt5.QtWidgets import QSizePolicy
        w.setSizePolicy(w.sizePolicy().horizontalPolicy(), QSizePolicy.Fixed)
        g = QGridLayout(w); g.setSpacing(3); g.setContentsMargins(0,0,0,0)
        self._jcards = []
        for i, j in enumerate(self._joints):
            card = JointCard(j["id"],j["ros"],j["nm"],j["mn"],j["mx"])
            g.addWidget(card,i//3,i%3); g.setColumnStretch(i%3,1)
            self._jcards.append(card)
        return w

    def _make_right(self):
        w = QWidget(); w.setStyleSheet("background:transparent;"); w.setFixedWidth(200)
        ly = QVBoxLayout(w); ly.setContentsMargins(0,0,0,0); ly.setSpacing(5)

        # Program loader
        prog = Panel("PROGRAM LOADER","NO FILE")
        self._file_lbl = make_label("Drop or click to load",C["t2"],8,mono=True)
        self._file_lbl.setAlignment(Qt.AlignCenter); self._file_lbl.setWordWrap(True)
        load_btn = QPushButton("📁  BROWSE FILE")
        load_btn.setStyleSheet(
            f"QPushButton{{background:transparent;border:2px dashed {C['bb']};"
            f"border-radius:5px;color:{C['t2']};font-family:'Share Tech Mono';"
            f"font-size:8px;padding:5px;letter-spacing:.12em;}}"
            f"QPushButton:hover{{border-color:{C['a']};color:{C['a']};background:rgba(0,212,170,0.04);}}")
        load_btn.clicked.connect(self._browse)
        self._pbar = QProgressBar(); self._pbar.setRange(0,100); self._pbar.setValue(0)
        self._pbar.setTextVisible(False); self._pbar.setFixedHeight(3)
        self._pbar.setStyleSheet(
            f"QProgressBar{{background:rgba(255,255,255,0.04);border-radius:2px;border:none;}}"
            f"QProgressBar::chunk{{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            f"stop:0 {C['a2']},stop:1 {C['a']});border-radius:2px;}}")
        self._pbar_lbl = make_label("READY",C["t3"],7,mono=True)
        run_btn = QPushButton("▶  START"); run_btn.setStyleSheet(
            f"QPushButton{{background:rgba(0,255,157,0.1);border:2px solid {C['a2']};"
            f"border-radius:6px;color:{C['a2']};font-family:'Rajdhani';"
            f"font-size:11px;font-weight:700;letter-spacing:.12em;padding:5px;}}"
            f"QPushButton:hover{{background:rgba(0,255,157,0.2);}}")
        stop_btn = QPushButton("⏹  STOP"); stop_btn.setStyleSheet(
            f"QPushButton{{background:rgba(255,61,90,0.1);border:2px solid {C['red']};"
            f"border-radius:6px;color:{C['red']};font-family:'Rajdhani';"
            f"font-size:11px;font-weight:700;letter-spacing:.12em;padding:5px;}}"
            f"QPushButton:hover{{background:rgba(255,61,90,0.2);}}")
        run_btn.clicked.connect(self._run_prog); stop_btn.clicked.connect(self._stop_prog)
        btns = QHBoxLayout(); btns.addWidget(run_btn); btns.addWidget(stop_btn)
        prog.body.addWidget(load_btn); prog.body.addWidget(self._file_lbl)
        prog.body.addWidget(self._pbar_lbl); prog.body.addWidget(self._pbar)
        prog.body.addLayout(btns); ly.addWidget(prog, 0)

        # Diagnostics
        diag = Panel("DIAGNOSTICS"); self._diag = {}
        for key, label, color in [("temp","JOINT TEMP AVG",C["a"]),
                                   ("pwr", "POWER DRAW",    C["a2"]),
                                   ("lat", "ROS LATENCY",   C["a"])]:
            row = QHBoxLayout()
            lbl = make_label(label,C["t3"],6,mono=True); lbl.setFixedWidth(64)
            bar = QProgressBar(); bar.setRange(0,100); bar.setValue(40)
            bar.setTextVisible(False); bar.setFixedHeight(3)
            bar.setStyleSheet(
                f"QProgressBar{{background:rgba(255,255,255,0.04);border-radius:2px;border:none;}}"
                f"QProgressBar::chunk{{background:{color};border-radius:2px;}}")
            val = make_label("--",C["t1"],9,True); val.setFixedWidth(30); val.setAlignment(Qt.AlignRight)
            row.addWidget(lbl); row.addWidget(bar); row.addWidget(val)
            self._diag[key]=(bar,val); diag.body.addLayout(row)
        ly.addWidget(diag, 0)

        # Log
        log_pnl = Panel("SYSTEM LOG"); self._log = LogWidget()
        log_pnl.body.addWidget(self._log); ly.addWidget(log_pnl, 1)
        return w

    # ── program helpers ───────────────────────────────────────────────────────
    _prog_file=None; _prog_pct=0.0; _prog_timer=None

    def _browse(self):
        path,_ = QFileDialog.getOpenFileName(self,"Load Program","",
            "Robot Programs (*.script *.txt *.json *.py *.xml);;All Files (*)")
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

    # ── public API ────────────────────────────────────────────────────────────
    def update_from_ros(self, pos, vel, eff):
        """Called from HMI when /joint_states arrives (values in radians)."""
        for i, card in enumerate(self._jcards):
            if i >= len(pos): break
            deg = math.degrees(pos[i])
            self._joints[i]["v"] = deg
            card.update_data(deg,
                             math.degrees(vel[i]) if i<len(vel) else 0.0,
                             eff[i] if i<len(eff) else 0.0)
        # Push raw radians into the 3D viewer
        self._viewer_frame.set_joint_angles(list(pos[:6]))

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
        # Push demo angles into the 3D viewer (convert deg→rad)
        angles_rad = [math.radians(j["v"]) for j in self._joints]
        self._viewer_frame.set_joint_angles(angles_rad)

    def log(self, msg, kind="info"): return self._log.add(msg,kind)


# ─────────────────────────────────────────────────────────────────────────────
#  CONTROL PAGE
# ─────────────────────────────────────────────────────────────────────────────
class CtrlPage(QWidget):
    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent); self._ros=ros; self._joints=joints
        self._jx=self._jy=self._jz=self._jwz=0.0
        self._joy_timer=QTimer(self); self._joy_timer.timeout.connect(self._pub_twist)
        self._build_ui()

    def _build_ui(self):
        g = QGridLayout(self); g.setContentsMargins(0,0,0,0); g.setSpacing(7)
        g.setColumnStretch(1,1); g.setColumnMinimumWidth(0,650); g.setColumnMinimumWidth(2,200)

        # ── LEFT: 3D viewer ──────────────────────────────────────────────────
        self._viewer_frame = ViewerFrame("TM5-900  ·  CONTROL VIEW")
        g.addWidget(self._viewer_frame, 0, 0, 2, 1)

        # ── CENTER: mode + sliders + commands ────────────────────────────────
        center = QScrollArea(); center.setWidgetResizable(True)
        center.setStyleSheet("background:transparent;border:none;")
        cw = QWidget(); cw.setStyleSheet("background:transparent;")
        cl = QVBoxLayout(cw); cl.setSpacing(7)

        mode_pnl = Panel("OPERATION MODE")
        mg = QGridLayout(); mg.setSpacing(4); self._mode_btns=[]
        for i, m in enumerate(["MANUAL","AUTO","TEACH","SIMULATION"]):
            b = QPushButton(m); b.setCheckable(True); b.setChecked(m=="MANUAL")
            b.setStyleSheet(self._mode_style())
            b.clicked.connect(lambda _, btn=b, md=m: self._set_mode(btn,md))
            mg.addWidget(b,i//2,i%2); self._mode_btns.append(b)
        mode_pnl.body.addLayout(mg); cl.addWidget(mode_pnl)

        s_pnl = Panel("JOINT CONTROL","6-DOF · DEG")
        sa = QScrollArea(); sa.setWidgetResizable(True)
        sa.setStyleSheet("background:transparent;border:none;")
        sw = QWidget(); sw.setStyleSheet("background:transparent;")
        sl_ly = QVBoxLayout(sw); sl_ly.setSpacing(9); self._sliders=[]
        for i, j in enumerate(self._joints):
            box = QVBoxLayout(); box.setSpacing(3)
            hdr = QHBoxLayout()
            hdr.addWidget(make_label(f"{j['id']} · {j['ros']}",C["t2"],10,True))
            hdr.addStretch()
            sv = make_label(f"{j['v']:.1f}°",C["a"],10,mono=True); hdr.addWidget(sv)
            s = QSlider(Qt.Horizontal)
            s.setRange(int(j["mn"]*10),int(j["mx"]*10)); s.setValue(int(j["v"]*10))
            s.setStyleSheet(slider_style(C["a"]))
            lims = QHBoxLayout()
            lims.addWidget(make_label(f"{j['mn']}°",C["t3"],7,mono=True))
            lims.addStretch()
            lims.addWidget(make_label(f"{j['mx']}°",C["t3"],7,mono=True))
            s.valueChanged.connect(lambda val,idx=i,lbl=sv: self._on_slider(idx,val/10,lbl))
            box.addLayout(hdr); box.addWidget(s); box.addLayout(lims); sl_ly.addLayout(box)
            self._sliders.append((s,sv))
        sa.setWidget(sw); s_pnl.body.addWidget(sa); cl.addWidget(s_pnl,1)

        cmd_pnl = Panel("ROBOT COMMANDS"); cg = QGridLayout(); cg.setSpacing(5)
        for i,(ic,lb,kd) in enumerate([
            ("▶","START","start"),("⏹","STOP","stop"),("⏸","PAUSE","normal"),
            ("⌂","HOME","normal"),("○","ZERO","normal"),("↺","RESET","normal")]):
            b = CmdButton(ic,lb,kd); b.clicked.connect(lambda _,c=lb: self._cmd(c))
            cg.addWidget(b,i//3,i%3)
        cmd_pnl.body.addLayout(cg); cl.addWidget(cmd_pnl)
        center.setWidget(cw); g.addWidget(center, 0, 1, 2, 1)

        self._jxy=None; self._joy_spd=None

        # ── RIGHT: limits + tasks ─────────────────────────────────────────────
        rs = QScrollArea(); rs.setWidgetResizable(True)
        rs.setStyleSheet("background:transparent;border:none;")
        rw = QWidget(); rw.setStyleSheet("background:transparent;")
        rl = QVBoxLayout(rw); rl.setSpacing(7)
        lim_pnl = Panel("SPEED & TORQUE","LIMITS")
        for lbl,mn,mx,val,unit,color in [
            ("JOINT SPEED",    0, 100, 100,"%",   C["a2"]),
            ("TCP SPEED LIMIT",0,2500,2500,"mm/s",C["a2"]),
            ("TORQUE LIMIT",   0, 100, 100,"%",   C["a3"]),
            ("COLLISION SENSE",0, 100,  50,"%",   C["a3"])]:
            lh=QHBoxLayout()
            lh.addWidget(make_label(lbl,C["t2"],8,mono=True)); lh.addStretch()
            vl=make_label(f"{val}{unit}",C["t1"],13,True); lh.addWidget(vl)
            s=QSlider(Qt.Horizontal); s.setRange(mn,mx); s.setValue(val)
            s.setStyleSheet(slider_style(color))
            s.valueChanged.connect(lambda v,u=unit,l=vl: l.setText(f"{v}{u}"))
            lims2=QHBoxLayout()
            lims2.addWidget(make_label(str(mn),C["t3"],7,mono=True)); lims2.addStretch()
            lims2.addWidget(make_label(str(mx),C["t3"],7,mono=True))
            lim_pnl.body.addLayout(lh); lim_pnl.body.addWidget(s); lim_pnl.body.addLayout(lims2)
        rl.addWidget(lim_pnl)

        task_pnl = Panel("TASK PROGRAMS","Webots")
        for no,nm,st_ in [("01","Home Position","rdy"),("02","Pick & Place A→B","idle"),
                           ("03","Inspection Scan","idle"),("04","Calibration Routine","done"),
                           ("05","Custom Trajectory","idle")]:
            f=QFrame()
            f.setStyleSheet(
                f"QFrame{{background:rgba(0,0,0,0.25);border:1px solid {C['b']};border-radius:5px;}}"
                f"QFrame:hover{{border-color:{C['bb']};}}")
            f.setCursor(Qt.PointingHandCursor)
            ly2=QHBoxLayout(f); ly2.setContentsMargins(9,6,9,6)
            ly2.addWidget(make_label(no,C["t3"],8,mono=True)); ly2.addWidget(make_label(nm,C["t1"],10))
            ly2.addStretch()
            sc,bg={"rdy":(C["a2"],"rgba(0,255,157,0.07)"),"done":(C["a"],"rgba(0,212,170,0.07)"),
                   "idle":(C["t3"],"rgba(255,255,255,0.02)")}.get(st_,(C["t3"],"transparent"))
            sl2=QLabel(st_.upper()); sl2.setStyleSheet(
                f"color:{sc};background:{bg};border:1px solid {sc}40;"
                f"border-radius:10px;font-size:7px;padding:2px 5px;font-family:'Share Tech Mono';")
            ly2.addWidget(sl2); task_pnl.body.addWidget(f)
        rl.addWidget(task_pnl); rl.addStretch(); rs.setWidget(rw)
        g.addWidget(rs, 0, 2, 2, 1)

    def _mode_style(self):
        return (f"QPushButton{{background:transparent;border:1px solid {C['b']};border-radius:4px;"
                f"color:{C['t2']};font-family:'Share Tech Mono';font-size:8.5px;"
                f"letter-spacing:.12em;padding:3px 6px;}}"
                f"QPushButton:checked{{background:rgba(0,212,170,0.09);border-color:{C['a']};color:{C['a']};}}"
                f"QPushButton:hover{{border-color:{C['a']};color:{C['a']};}}")

    def _on_slider(self, i, val, lbl):
        self._joints[i]["v"]=val; lbl.setText(f"{val:.1f}°")
        self._ros.publish_joints([j["v"] for j in self._joints])
        # Update 3D viewer from slider
        angles_rad = [math.radians(j["v"]) for j in self._joints]
        self._viewer_frame.set_joint_angles(angles_rad)

    def update_canvas_from_ros(self, angles_rad):
        """Called from HMI with raw radian values from /joint_states."""
        self._viewer_frame.set_joint_angles(angles_rad)
        # Sync sliders without triggering valueChanged
        for i, (s, lbl) in enumerate(self._sliders):
            if i < len(angles_rad):
                deg = math.degrees(angles_rad[i])
                s.blockSignals(True); s.setValue(int(deg*10)); s.blockSignals(False)
                lbl.setText(f"{deg:.1f}°")

    def _joy_stop(self):
        self._jx=self._jy=self._jz=self._jwz=0.0
        self._joy_timer.stop(); self._ros.publish_twist(0,0,0,0)

    def _pub_twist(self):
        s=(self._joy_spd.value()/100*0.5) if self._joy_spd else 0.25
        self._ros.publish_twist(self._jx*s,self._jy*s,self._jz*s,self._jwz*s)

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
        self.setWindowTitle("Techman TM5-900 | HMI v2.0 — ros3djs 3D View")
        self.setMinimumSize(1200,700); self.resize(1440,860)
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

        self._dash.log("HMI v2.0 initialized — Techman TM5-900 6-DOF","ok")
        self._dash.log("3D viewer: ros3djs + Three.js via QWebEngineView","info")
        self._dash.log("Rosbridge required: ros2 launch rosbridge_server rosbridge_websocket_launch.xml","warn")
        self._dash.log("Mesh server required: python3 -m http.server 8080 --directory <urdf_mesh_dir>","warn")
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
        self._dash.update_from_ros(pos, vel, eff)   # includes 3D viewer update
        self._ctrl.update_canvas_from_ros(pos)       # includes slider sync

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


# ─────────────────────────────────────────────────────────────────────────────
def _padded(w):
    c = QWidget(); c.setStyleSheet("background:transparent;")
    ly = QVBoxLayout(c); ly.setContentsMargins(7,7,7,7); ly.setSpacing(0)
    ly.addWidget(w); return c

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("Techman TM5-900 HMI")
    app.setStyleSheet(STYLE_BASE)
    QFontDatabase.addApplicationFont("Rajdhani-Regular.ttf")
    QFontDatabase.addApplicationFont("Exo2-Regular.ttf")
    win = HMI(); win.show()
    sys.exit(app.exec_())
