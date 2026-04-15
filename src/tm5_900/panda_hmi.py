#!/usr/bin/env python3
"""
Franka Panda Robot HMI — PyQt5
Full interface for panda_ws / Webots simulation.

Dependencies:
    sudo apt install python3-pyqt5
    (rclpy comes from your ROS 2 workspace — source it before running)

ROS 2 topics:
    SUB  /joint_states                              sensor_msgs/JointState
    PUB  /panda_arm_controller/joint_trajectory     trajectory_msgs/JointTrajectory
    PUB  /cmd_vel                                   geometry_msgs/Twist
    PUB  /robot_command                             std_msgs/String

Run:
    source ~/panda_ws/install/setup.bash
    python3 panda_hmi.py
"""

import sys
import math
import random
import threading
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QSlider, QFrame,
    QScrollArea, QFileDialog, QProgressBar, QTextEdit,
    QStackedWidget, QSplitter,
)
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    from PyQt5.QtCore import QUrl
    WEBENGINE_AVAILABLE = True
except ImportError:
    WEBENGINE_AVAILABLE = False
from PyQt5.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, pyqtSlot, QPoint,
)
from PyQt5.QtGui import (
    QColor, QPainter, QPen, QBrush, QPolygon, QPixmap,
    QLinearGradient, QRadialGradient, QFontDatabase,
)

# ─────────────────────────────────────────────────────────────
#  COLOUR PALETTE
# ─────────────────────────────────────────────────────────────
C = {
    "bg":    "#060b10",
    "card":  "#0d1825",
    "card2": "#0b1520",
    "b":     "#16293e",
    "bb":    "#1c3650",
    "a":     "#00c8ff",
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
    background: {C['bg']};
    color: {C['t1']};
    font-family: 'Exo 2', 'Rajdhani', sans-serif;
    font-size: 11px;
}}
QScrollBar:vertical {{
    background: transparent; width: 4px; border-radius: 2px;
}}
QScrollBar::handle:vertical {{
    background: {C['bb']}; border-radius: 2px; min-height: 20px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar:horizontal {{
    background: transparent; height: 4px;
}}
QScrollBar::handle:horizontal {{
    background: {C['bb']}; border-radius: 2px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}
QSlider::groove:horizontal {{
    height: 4px; background: rgba(255,255,255,0.07); border-radius: 2px;
}}
QSlider::handle:horizontal {{
    width: 13px; height: 13px; margin: -5px 0;
    border-radius: 7px; background: {C['a']}; border: 2px solid {C['bg']};
}}
QSlider::sub-page:horizontal {{
    background: {C['a']}; border-radius: 2px;
}}
"""

# ─────────────────────────────────────────────────────────────
#  PANDA JOINT DEFINITIONS  (from panda_webots.yaml)
# ─────────────────────────────────────────────────────────────
PANDA_JOINTS = [
    {"id": "J1", "ros": "panda_joint1", "nm": "Joint 1", "mn": -166, "mx":  166, "v":   0},
    {"id": "J2", "ros": "panda_joint2", "nm": "Joint 2", "mn": -101, "mx":  101, "v":   0},
    {"id": "J3", "ros": "panda_joint3", "nm": "Joint 3", "mn": -166, "mx":  166, "v":   0},
    {"id": "J4", "ros": "panda_joint4", "nm": "Joint 4", "mn": -176, "mx":   -4, "v": -90},
    {"id": "J5", "ros": "panda_joint5", "nm": "Joint 5", "mn": -166, "mx":  166, "v":   0},
    {"id": "J6", "ros": "panda_joint6", "nm": "Joint 6", "mn":   -1, "mx":  215, "v":  90},
    {"id": "J7", "ros": "panda_joint7", "nm": "Joint 7", "mn": -166, "mx":  166, "v":   0},
]

# ─────────────────────────────────────────────────────────────
#  ROS 2 WORKER THREAD
# ─────────────────────────────────────────────────────────────
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

    # ── exact names from panda_webots.yaml
    JOINT_NAMES = [j["ros"] for j in PANDA_JOINTS]

    def __init__(self):
        super().__init__()
        self._node   = None
        self._active = True
        self._pub_j  = None
        self._pub_t  = None
        self._pub_c  = None
        self._lock   = threading.Lock()

    def run(self):
        if not ROS_AVAILABLE:
            self.connection_changed.emit(False)
            self.log_message.emit("rclpy not found — demo mode active", "warn")
            return
        try:
            rclpy.init()
            self._node = Node("panda_hmi")

            # publisher: joint trajectory → panda_arm_controller
            self._pub_j = self._node.create_publisher(
                JointTrajectory,
                "/panda_arm_controller/joint_trajectory",
                10,
            )
            self._pub_t = self._node.create_publisher(Twist,  "/cmd_vel",       10)
            self._pub_c = self._node.create_publisher(String, "/robot_command",  10)

            # subscriber: joint_state_broadcaster → /joint_states
            self._node.create_subscription(
                JointState, "/joint_states", self._js_cb, 10
            )

            self.connection_changed.emit(True)
            self.log_message.emit("ROS 2 connected — panda_hmi node online", "ok")
            self.log_message.emit(
                f"Listening on /joint_states  |  Publishing on "
                f"/panda_arm_controller/joint_trajectory", "info"
            )

            while self._active and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)

        except Exception as exc:
            self.connection_changed.emit(False)
            self.log_message.emit(f"ROS 2 error: {exc}", "err")

    def _js_cb(self, msg):
        # msg.name may arrive in any order — re-order to match PANDA_JOINTS
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        pos, vel, eff = [], [], []
        for jname in self.JOINT_NAMES:
            i = name_to_idx.get(jname)
            if i is None:
                pos.append(0.0); vel.append(0.0); eff.append(0.0)
            else:
                pos.append(msg.position[i] if msg.position else 0.0)
                vel.append(msg.velocity[i] if msg.velocity else 0.0)
                eff.append(msg.effort[i]   if msg.effort   else 0.0)
        self.joint_state_received.emit(pos, vel, eff)

    def publish_joints(self, angles_deg):
        if not self._pub_j:
            return
        n = len(PANDA_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions        = [math.radians(a) for a in angles_deg[:n]]
        pt.velocities       = [0.0] * n
        pt.accelerations    = [0.0] * n
        pt.time_from_start  = Duration(sec=1, nanosec=0)
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES
        msg.points = [pt]
        with self._lock:
            self._pub_j.publish(msg)

    def publish_twist(self, lx, ly, lz, az):
        if not self._pub_t:
            return
        msg = Twist()
        msg.linear.x = lx; msg.linear.y = ly; msg.linear.z = lz
        msg.angular.z = az
        with self._lock:
            self._pub_t.publish(msg)

    def publish_cmd(self, cmd: str):
        if not self._pub_c:
            return
        msg = String(); msg.data = cmd
        with self._lock:
            self._pub_c.publish(msg)

    def stop(self):
        self._active = False
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if ROS_AVAILABLE and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────
#  SMALL HELPERS
# ─────────────────────────────────────────────────────────────
def make_label(text, color=C["t1"], size=11, bold=False, mono=False):
    lbl = QLabel(text)
    family = "'Share Tech Mono'" if mono else "'Rajdhani', sans-serif"
    weight = "bold" if bold else "normal"
    lbl.setStyleSheet(
        f"color:{color}; font-size:{size}px; font-weight:{weight};"
        f"font-family:{family}; background:transparent;"
    )
    return lbl


def slider_style(color):
    return (
        f"QSlider::groove:horizontal {{ height:4px; background:rgba(255,255,255,0.07); border-radius:2px; }}"
        f"QSlider::handle:horizontal {{ width:13px; height:13px; margin:-5px 0; border-radius:7px;"
        f"  background:{color}; border:2px solid {C['bg']}; }}"
        f"QSlider::sub-page:horizontal {{ background:{color}; border-radius:2px; }}"
    )


# ─────────────────────────────────────────────────────────────
#  PANEL
# ─────────────────────────────────────────────────────────────
class Panel(QFrame):
    def __init__(self, title, tag="", parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"Panel {{ background:{C['card']}; border:1px solid {C['b']}; border-radius:7px; }}"
        )
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        hdr = QFrame()
        hdr.setFixedHeight(24)
        hdr.setStyleSheet(
            f"background:rgba(0,0,0,0.2); border-bottom:1px solid {C['b']}; border-radius:0;"
        )
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(11, 0, 11, 0)
        hl.addWidget(make_label(title, C["a"], 10, True))
        hl.addStretch()
        if tag:
            hl.addWidget(make_label(tag, C["t3"], 8, mono=True))
        outer.addWidget(hdr)

        self.content = QWidget()
        self.content.setStyleSheet("background:transparent;")
        self.body = QVBoxLayout(self.content)
        self.body.setContentsMargins(6, 5, 6, 5)
        self.body.setSpacing(4)
        outer.addWidget(self.content)

    def paintEvent(self, e):
        super().paintEvent(e)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        grad = QLinearGradient(0, 0, self.width(), 0)
        grad.setColorAt(0,   QColor(0, 200, 255, 0))
        grad.setColorAt(0.5, QColor(0, 200, 255, 97))
        grad.setColorAt(1,   QColor(0, 200, 255, 0))
        p.setPen(QPen(QBrush(grad), 2))
        p.drawLine(0, 1, self.width(), 1)


# ─────────────────────────────────────────────────────────────
#  STAT CARD
# ─────────────────────────────────────────────────────────────
class StatCard(QFrame):
    ACCENT = {"g": C["a2"], "b": C["a"], "o": C["a3"], "r": C["red"]}

    def __init__(self, label, value, sub, kind="b", parent=None):
        super().__init__(parent)
        self._kind = kind
        self.setMinimumHeight(50)
        self.setMaximumHeight(125)
        self.setStyleSheet(
            f"StatCard {{ background:{C['card2']}; border:1px solid {C['b']}; border-radius:6px; }}"
        )
        ly = QVBoxLayout(self)
        ly.setContentsMargins(10, 8, 10, 8)
        ly.setSpacing(2)
        self.lbl = make_label(label, C["t3"], 10, mono=True)
        self.val = make_label(value, C["t1"], 18, True)
        self.sub = make_label(sub,   C["t3"],  8, mono=True)
        ly.addWidget(self.lbl)
        ly.addWidget(self.val)
        ly.addWidget(self.sub)

    def set_value(self, v, color=None):
        self.val.setText(str(v))
        if color:
            self.val.setStyleSheet(
                f"color:{color}; font-size:18px; font-weight:bold; background:transparent;"
            )

    def paintEvent(self, e):
        super().paintEvent(e)
        p = QPainter(self)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(self.ACCENT.get(self._kind, C["a"])))
        r = self.rect()
        p.drawRoundedRect(r.x() + 1, r.bottom() - 3, r.width() - 2, 2, 1, 1)


# ─────────────────────────────────────────────────────────────
#  JOINT CARD  (one per panda_joint)
# ─────────────────────────────────────────────────────────────
class JointCard(QFrame):
    def __init__(self, jid, ros_name, display_name, mn, mx, parent=None):
        super().__init__(parent)
        self._mn = mn
        self._mx = mx
        self.setMaximumHeight(90)
        self.setStyleSheet(
            f"JointCard {{ background:{C['card2']}; border:1px solid {C['b']}; border-radius:7px; }}"
        )
        ly = QVBoxLayout(self)
        ly.setContentsMargins(5, 3, 5, 3)
        ly.setSpacing(2)

        # ── header row
        hr = QHBoxLayout()
        left = QVBoxLayout(); left.setSpacing(0)
        self.id_lbl = make_label(jid,              C["a"],  13, True)
        self.nm_lbl = make_label(ros_name.upper(), C["t3"],  7, mono=True)
        left.addWidget(self.id_lbl)
        left.addWidget(self.nm_lbl)
        hr.addLayout(left)
        hr.addStretch()
        self.dot = QLabel()
        self.dot.setFixedSize(8, 8)
        self._set_dot("ok")
        hr.addWidget(self.dot)
        ly.addLayout(hr)

        # ── display name
        self.disp_lbl = make_label(display_name, C["t2"], 8)
        ly.addWidget(self.disp_lbl)

        # ── position value
        self.pos_lbl = make_label("0.0°", C["t1"], 12, True)
        ly.addWidget(self.pos_lbl)

        # ── position bar
        self.pos_bar = QProgressBar()
        self.pos_bar.setRange(0, 1000)
        self.pos_bar.setValue(500)
        self.pos_bar.setTextVisible(False)
        self.pos_bar.setFixedHeight(4)
        self.pos_bar.setStyleSheet(
            f"QProgressBar {{ background:rgba(255,255,255,0.05); border-radius:2px; border:none; }}"
            f"QProgressBar::chunk {{ background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            f"stop:0 {C['a']},stop:1 {C['a2']}); border-radius:2px; }}"
        )
        ly.addWidget(self.pos_bar)

        # ── vel + eff row
        row = QHBoxLayout()
        vl = QVBoxLayout(); vl.setSpacing(0)
        vl.addWidget(make_label("VEL", C["t3"], 7, mono=True))
        self.vel_val = make_label("0.00°/s", C["a2"],  8, True)
        vl.addWidget(self.vel_val)
        el = QVBoxLayout(); el.setSpacing(0)
        el.addWidget(make_label("EFF", C["t3"], 7, mono=True))
        self.eff_val = make_label("0.00Nm", C["a3"],  8, True)
        el.addWidget(self.eff_val)
        row.addLayout(vl); row.addLayout(el)
        ly.addLayout(row)

    def _set_dot(self, state):
        c = {"ok": C["ok"], "w": C["warn"], "e": C["red"]}.get(state, C["ok"])
        self.dot.setStyleSheet(f"background:{c}; border-radius:4px;")

    def update_data(self, deg, vel, eff):
        self.pos_lbl.setText(f"{deg:.1f}°")
        pct = int((deg - self._mn) / (self._mx - self._mn) * 1000)
        self.pos_bar.setValue(max(0, min(1000, pct)))
        self.vel_val.setText(f"{vel:.2f}°/s")
        self.eff_val.setText(f"{eff:.2f}Nm")
        self._set_dot("w" if abs(vel) > 4 else "ok")

    def paintEvent(self, e):
        super().paintEvent(e)
        p = QPainter(self)
        grad = QLinearGradient(0, 0, self.width(), 0)
        grad.setColorAt(0,   QColor(0, 200, 255, 0))
        grad.setColorAt(0.5, QColor(0, 200, 255, 77))
        grad.setColorAt(1,   QColor(0, 200, 255, 0))
        p.setPen(QPen(QBrush(grad), 2))
        p.drawLine(0, 1, self.width(), 1)


# ─────────────────────────────────────────────────────────────
#  TCP CELL
# ─────────────────────────────────────────────────────────────
class TcpCell(QFrame):
    def __init__(self, axis, color, unit="mm", parent=None):
        super().__init__(parent)
        self._unit = unit
        self.setStyleSheet(
            f"TcpCell {{ background:rgba(0,0,0,0.25); border:1px solid {C['b']}; border-radius:5px; }}"
        )
        ly = QVBoxLayout(self)
        ly.setContentsMargins(5, 4, 5, 4)
        ly.setSpacing(1)
        ly.addWidget(make_label(axis, color, 8, mono=True))
        self.val = make_label(f"0.0 {unit}", C["t1"], 10, True)
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000); self.bar.setValue(500)
        self.bar.setTextVisible(False); self.bar.setFixedHeight(2)
        self.bar.setStyleSheet(
            f"QProgressBar {{ background:rgba(255,255,255,0.04); border-radius:1px; border:none; }}"
            f"QProgressBar::chunk {{ background:{color}; border-radius:1px; }}"
        )
        ly.addWidget(self.val); ly.addWidget(self.bar)

    def set_value(self, v, mn=-900, mx=900):
        self.val.setText(f"{v:.1f} {self._unit}")
        pct = int((v - mn) / (mx - mn) * 1000)
        self.bar.setValue(max(0, min(1000, pct)))


# ─────────────────────────────────────────────────────────────
#  JOYSTICK XY
# ─────────────────────────────────────────────────────────────
class JoystickXY(QWidget):
    moved    = pyqtSignal(float, float)
    released = pyqtSignal()

    def __init__(self, size=145, parent=None):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self._R  = size / 2 - 18
        self._cx = size / 2
        self._cy = size / 2
        self._tx = self._cx
        self._ty = self._cy
        self._drag = False
        self.setCursor(Qt.CrossCursor)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._drag = True; self._move_to(e.pos())

    def mouseMoveEvent(self, e):
        if self._drag: self._move_to(e.pos())

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._drag = False
            self._tx = self._cx; self._ty = self._cy
            self.update()
            self.moved.emit(0.0, 0.0)
            self.released.emit()

    def _move_to(self, pos):
        dx = pos.x() - self._cx
        dy = pos.y() - self._cy
        d  = math.sqrt(dx * dx + dy * dy)
        if d > self._R:
            dx = dx / d * self._R; dy = dy / d * self._R
        self._tx = self._cx + dx; self._ty = self._cy + dy
        self.update()
        self.moved.emit(round(dx / self._R, 3), round(-dy / self._R, 3))

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy = int(self._cx), int(self._cy)
        R = int(self._R) + 17
        p.setPen(QPen(QColor(C["b"]), 2))
        p.setBrush(QBrush(QColor(0, 200, 255, 13)))
        p.drawEllipse(cx - R, cy - R, R * 2, R * 2)
        p.setPen(QPen(QColor(0, 200, 255, 23), 1, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        r2 = int(R * 0.56)
        p.drawEllipse(cx - r2, cy - r2, r2 * 2, r2 * 2)
        p.setPen(QPen(QColor(0, 200, 255, 20), 1))
        p.drawLine(cx - R + 5, cy, cx + R - 5, cy)
        p.drawLine(cx, cy - R + 5, cx, cy + R - 5)
        tx, ty = int(self._tx), int(self._ty)
        grad = QRadialGradient(tx - 3, ty - 3, 15)
        grad.setColorAt(0, QColor(0, 200, 255, 165))
        grad.setColorAt(1, QColor(0, 200, 255, 51))
        p.setPen(QPen(QColor(C["a"]), 2))
        p.setBrush(QBrush(grad))
        p.drawEllipse(tx - 15, ty - 15, 30, 30)


# ─────────────────────────────────────────────────────────────
#  VERTICAL JOYSTICK
# ─────────────────────────────────────────────────────────────
class JoystickV(QWidget):
    moved    = pyqtSignal(float)
    released = pyqtSignal()

    def __init__(self, color=C["a2"], size=(48, 138), parent=None):
        super().__init__(parent)
        self.setFixedSize(*size)
        self._color = QColor(color)
        self._h  = size[1]
        self._tw = 36; self._th = 19
        self._ty = (self._h - self._th) / 2
        self._drag = False
        self._sy = 0; self._st = 0
        self.setCursor(Qt.SizeVerCursor)

    def _clamp(self, t):
        return max(0, min(self._h - self._th, t))

    def _to_val(self, t):
        return round(1 - t / (self._h - self._th) * 2, 3)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._drag = True; self._sy = e.y(); self._st = self._ty

    def mouseMoveEvent(self, e):
        if self._drag:
            self._ty = self._clamp(self._st + (e.y() - self._sy))
            self.update(); self.moved.emit(self._to_val(self._ty))

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._drag = False
            self._ty = (self._h - self._th) / 2
            self.update(); self.moved.emit(0.0); self.released.emit()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        c = self._color
        p.setPen(QPen(QColor(C["b"]), 2))
        p.setBrush(QBrush(QColor(c.red(), c.green(), c.blue(), 5)))
        p.drawRoundedRect(0, 0, w, self._h, w // 2, w // 2)
        p.setPen(QPen(QColor(c.red(), c.green(), c.blue(), 30), 1))
        p.drawLine(w // 2, int(self._h * 0.08), w // 2, int(self._h * 0.92))
        tx = (w - self._tw) // 2; ty = int(self._ty)
        grad = QRadialGradient(tx + 8, ty + 5, 18)
        grad.setColorAt(0, QColor(c.red(), c.green(), c.blue(), 133))
        grad.setColorAt(1, QColor(c.red(), c.green(), c.blue(), 36))
        p.setPen(QPen(self._color, 2))
        p.setBrush(QBrush(grad))
        p.drawRoundedRect(tx, ty, self._tw, self._th, self._th // 2, self._th // 2)


# ─────────────────────────────────────────────────────────────
#  COMMAND BUTTON
# ─────────────────────────────────────────────────────────────
class CmdButton(QPushButton):
    def __init__(self, icon, label, kind="normal", parent=None):
        super().__init__(parent)
        ly = QVBoxLayout(self)
        ly.setContentsMargins(4, 9, 4, 9)
        ly.setAlignment(Qt.AlignCenter)
        il = QLabel(icon); il.setAlignment(Qt.AlignCenter)
        il.setStyleSheet("font-size:14px; background:transparent;")
        ll = QLabel(label); ll.setAlignment(Qt.AlignCenter)
        ll.setStyleSheet(
            "font-size:7.5px; letter-spacing:0.15em; "
            "font-family:'Share Tech Mono'; background:transparent;"
        )
        ly.addWidget(il); ly.addWidget(ll)
        kmap = {
            "start": (C["a2"], "rgba(0,255,157,0.07)"),
            "stop":  (C["red"], "rgba(255,61,90,0.05)"),
            "normal":(C["t2"], C["card2"]),
        }
        fc, bg = kmap.get(kind, kmap["normal"])
        self.setStyleSheet(
            f"CmdButton {{ color:{fc}; background:{bg}; border:1px solid {C['b']};"
            f"  border-radius:5px; min-height:36px; }}"
            f"CmdButton:hover {{ border-color:{C['a']}; color:{C['a']}; background:rgba(0,200,255,0.09); }}"
        )


# ─────────────────────────────────────────────────────────────
#  LOG WIDGET
# ─────────────────────────────────────────────────────────────
class LogWidget(QTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setStyleSheet(
            f"QTextEdit {{ background:#030810; border:1px solid {C['b']}; "
            f"border-radius:4px; font-family:'Share Tech Mono'; font-size:8px; color:{C['t2']}; }}"
        )
        self._count = 0

    def add(self, msg, kind="info"):
        col = {"ok": C["ok"], "warn": C["warn"], "err": C["red"], "info": C["a"]}.get(kind, C["a"])
        ts  = datetime.now().strftime("%H:%M:%S")
        self._count += 1
        self.append(
            f'<span style="color:{C["t3"]}">{ts}</span> '
            f'<span style="color:{col}">{msg}</span>'
        )
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())
        return self._count


# ─────────────────────────────────────────────────────────────
#  RVIZ LIVE VIEWER
#  Uses mss (cross-platform screen capture) to grab the RViz
#  window region and display it as a live feed inside the HMI.
#  Works on WSL2, X11, Wayland — no xwd needed.
#
#  Install:  pip install mss pygetwindow
#  On WSL2:  pip install mss  (uses Windows screen capture)
# ─────────────────────────────────────────────────────────────
class RVizEmbed(QWidget):
    status_changed = pyqtSignal(str, str)

    POLL_MS    = 2000
    CAPTURE_MS = 300   # ~20 fps

    def __init__(self, parent=None):
        super().__init__(parent)
        self._live      = False
        self._rviz_geom = None   # {left, top, width, height}
        self._drag_btn  = None
        self._drag_rviz_start = None
        self._drag_widget_start = None

        self.setStyleSheet("background:#040c14;")
        self.setMinimumSize(200, 150)
        self.setMouseTracking(True)
        self.setCursor(Qt.CrossCursor)

        self._lbl = QLabel(self)
        self._lbl.setAlignment(Qt.AlignCenter)
        self._lbl.setStyleSheet("background:#040c14;")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._lbl)

        self._show_placeholder()

        self._search_tmr = QTimer(self)
        self._search_tmr.timeout.connect(self._find_and_start)
        self._search_tmr.start(self.POLL_MS)

        self._cap_tmr = QTimer(self)
        self._cap_tmr.timeout.connect(self._capture)

        self.status_changed.emit("WAITING FOR WEBOTS...", C["t3"])  # was RVIZ

    def _show_placeholder(self):
        self._lbl.setPixmap(QPixmap())
        self._lbl.setText(
            "<div style='color:#273d52;font-family:monospace;font-size:9px;"
            "text-align:center;line-height:2'>"
            "&#8635; WAITING FOR RVIZ2<br>"
            "<span style='font-size:7px'>"
            "Open RViz2 — it will appear here automatically</span></div>"
        )

    # ── Find RViz window position using xdotool + xwininfo
    def _find_rviz_geometry(self):
        """Returns {left, top, width, height} of the RViz window, or None."""
        import subprocess, re

        # Step 1: get window ID
        win_id = None
        for name in ["RViz2", "RViz", "rviz2", "rviz"]:
            try:
                r = subprocess.run(
                    ["xdotool", "search", "--name", name],
                    capture_output=True, text=True, timeout=2
                )
                ids = [x for x in r.stdout.strip().split() if x.isdigit()]
                if ids:
                    win_id = ids[-1]
                    break
            except Exception:
                pass

        if not win_id:
            return None

        # Step 2: get geometry via xwininfo
        try:
            r = subprocess.run(
                ["xwininfo", "-id", win_id],
                capture_output=True, text=True, timeout=2
            )
            out = r.stdout
            ax = re.search(r"Absolute upper-left X:\s+(-?\d+)", out)
            ay = re.search(r"Absolute upper-left Y:\s+(-?\d+)", out)
            w  = re.search(r"Width:\s+(\d+)", out)
            h  = re.search(r"Height:\s+(\d+)", out)
            if ax and ay and w and h:
                return {
                    "left":   int(ax.group(1)),
                    "top":    int(ay.group(1)),
                    "width":  int(w.group(1)),
                    "height": int(h.group(1)),
                    "win_id": win_id,
                }
        except Exception:
            pass
        return None

    def _find_and_start(self):
        if self._live:
            return
        geom = self._find_rviz_geometry()
        if geom is None:
            self.status_changed.emit("WAITING FOR RVIZ2...", C["t3"])
            return
        self._rviz_geom = geom
        self._live = True
        self._search_tmr.stop()
        self._cap_tmr.start(self.CAPTURE_MS)
        self.status_changed.emit("RVIZ LIVE", C["ok"])

    # ── Capture via Windows PowerShell (WSL2/WSLg)
    def _capture(self):
        if not self._live or not self._rviz_geom:
            return
        try:
            import subprocess, os
            g = self._rviz_geom
            win_tmp = "C:\\temp\\hmi_cap.png"
            wsl_tmp = "/mnt/c/temp/hmi_cap.png"
            os.makedirs("/mnt/c/temp", exist_ok=True)
            ps = (
                f"Add-Type -AssemblyName System.Windows.Forms,System.Drawing;"
                f"$b=New-Object System.Drawing.Bitmap({g['width']},{g['height']});"
                f"$gr=[System.Drawing.Graphics]::FromImage($b);"
                f"$gr.CopyFromScreen({g['left']},{g['top']},0,0,$b.Size);"
                f"$b.Save('{win_tmp}')"
            )
            result = subprocess.run(
                ["powershell.exe", "-Command", ps],
                capture_output=True, timeout=5
            )
            if result.returncode == 0 and os.path.exists(wsl_tmp):
                px = QPixmap(wsl_tmp)
                if not px.isNull():
                    scaled = px.scaled(
                        self._lbl.width(), self._lbl.height(),
                        Qt.KeepAspectRatio, Qt.SmoothTransformation
                    )
                    self._lbl.setPixmap(scaled)
                    self.status_changed.emit("WEBOTS LIVE", C["ok"])
            else:
                err = result.stderr.decode().strip()
                self.status_changed.emit(f"PS err: {err[:40]}", C["warn"])
        except FileNotFoundError:
            self._cap_tmr.stop()
            self.status_changed.emit("powershell.exe not found", C["red"])
        except Exception as ex:
            self.status_changed.emit(f"Capture err: {str(ex)[:30]}", C["warn"])

    def _handle_lost(self):
        self._live = False
        self._rviz_geom = None
        self._cap_tmr.stop()
        self._show_placeholder()
        self._search_tmr.start(self.POLL_MS)
        self.status_changed.emit("RVIZ CLOSED...", C["warn"])

    # ── Map widget coords → screen coords in RViz window
    def _to_screen(self, ex, ey):
        if not self._rviz_geom:
            return 0, 0
        g = self._rviz_geom
        sx = g["left"] + int(ex * g["width"]  / max(self.width(),  1))
        sy = g["top"]  + int(ey * g["height"] / max(self.height(), 1))
        return sx, sy

    def _xdo_move(self, sx, sy):
        import subprocess
        try:
            subprocess.Popen(
                ["xdotool", "mousemove", "--sync", str(sx), str(sy)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except Exception:
            pass

    def _xdo_btn(self, action, btn_num):
        import subprocess
        try:
            subprocess.Popen(
                ["xdotool", action, str(btn_num)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except Exception:
            pass

    def mousePressEvent(self, e):
        if not self._live: return
        sx, sy = self._to_screen(e.x(), e.y())
        btn = {Qt.LeftButton: 1, Qt.MiddleButton: 2, Qt.RightButton: 3}.get(e.button(), 1)
        self._drag_btn = btn
        self._xdo_move(sx, sy)
        self._xdo_btn("mousedown", btn)

    def mouseMoveEvent(self, e):
        if not self._live or not self._drag_btn: return
        sx, sy = self._to_screen(e.x(), e.y())
        self._xdo_move(sx, sy)

    def mouseReleaseEvent(self, e):
        if not self._live: return
        sx, sy = self._to_screen(e.x(), e.y())
        btn = {Qt.LeftButton: 1, Qt.MiddleButton: 2, Qt.RightButton: 3}.get(e.button(), 1)
        self._xdo_move(sx, sy)
        self._xdo_btn("mouseup", btn)
        self._drag_btn = None

    def wheelEvent(self, e):
        if not self._live: return
        sx, sy = self._to_screen(e.x(), e.y())
        self._xdo_move(sx, sy)
        btn = 4 if e.angleDelta().y() > 0 else 5
        self._xdo_btn("click", btn)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._lbl.resize(self.size())
class WebotsCaptureWidget(QWidget):
    status_changed = pyqtSignal(str, str)

    CAPTURE_MS = 150  # ~6fps — safe for scrot

    def __init__(self, parent=None):
        super().__init__(parent)
        self._region   = None   # {left, top, width, height}
        self._live     = False
        self._selecting = False
        self._sel_start = None
        self._sel_end   = None

        self.setStyleSheet("background:#040c14;")
        self.setMinimumSize(200, 150)

        self._lbl = QLabel(self)
        self._lbl.setAlignment(Qt.AlignCenter)
        self._lbl.setStyleSheet("background:#040c14;")

        # Set region button
        self._btn = QPushButton("📷  CLICK TO SET CAPTURE REGION", self)
        self._btn.setStyleSheet(
            f"QPushButton {{ background:rgba(0,200,255,0.08); border:2px dashed {C['a']};"
            f"  border-radius:6px; color:{C['a']}; font-family:'Share Tech Mono';"
            f"  font-size:9px; padding:8px 14px; }}"
            f"QPushButton:hover {{ background:rgba(0,200,255,0.18); }}"
        )
        self._btn.clicked.connect(self._start_region_select)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._lbl)

        self._show_placeholder()

        self._cap_tmr = QTimer(self)
        self._cap_tmr.timeout.connect(self._capture)

        self.status_changed.emit("SET CAPTURE REGION", C["t3"])

    def _show_placeholder(self):
        self._lbl.setPixmap(QPixmap())
        self._lbl.setText(
            "\n\n"
            "↻ NO REGION SET\n\n"
            "Click the button below to\n"
            "select the Webots window area\n"
        )
        self._position_btn()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._lbl.resize(self.size())
        self._position_btn()

    def _position_btn(self):
        bw, bh = 260, 36
        self._btn.setGeometry(
            (self.width() - bw) // 2,
            self.height() - bh - 10,
            bw, bh
        )
        self._btn.raise_()

    def _start_region_select(self):
        """Open a full-screen transparent overlay for region selection."""
        self._overlay = RegionSelector()
        self._overlay.region_selected.connect(self._on_region)
        self._overlay.showFullScreen()

    def _on_region(self, x, y, w, h):
        if w < 20 or h < 20:
            return
        self._region = {"left": x, "top": y, "width": w, "height": h}
        self._live   = True
        self._btn.setText("✎  CHANGE REGION")
        self._cap_tmr.start(self.CAPTURE_MS)
        self.status_changed.emit("WEBOTS LIVE", C["ok"])

    def _capture(self):
        if not self._region:    # ← 8 spaces indent
            return
        try:
            import subprocess, tempfile, os
            g = self._region
            tmp = tempfile.mktemp(suffix=".png")
            result = subprocess.run(
                [
                    "scrot",
                    "-a", f"{g['left']},{g['top']},{g['width']},{g['height']}",
                    tmp,
                ],
                capture_output=True, timeout=3
            )
            if result.returncode == 0 and os.path.exists(tmp):
                px = QPixmap(tmp)
                os.unlink(tmp)
                if not px.isNull():
                    scaled = px.scaled(
                        self._lbl.width(), self._lbl.height(),
                        Qt.KeepAspectRatio, Qt.SmoothTransformation
                    )
                    self._lbl.setPixmap(scaled)
                    self.status_changed.emit("WEBOTS LIVE", C["ok"])
            else:
                err = result.stderr.decode().strip()
                self.status_changed.emit(f"scrot error: {err[:30]}", C["warn"])

        except FileNotFoundError:
            self._cap_tmr.stop()
            self._lbl.setText("\n\nsudo apt install scrot")
            self.status_changed.emit("install scrot", C["red"])
        except Exception as ex:
            self.status_changed.emit(f"Capture error: {ex}", C["warn"])
# ── Full-screen drag-to-select overlay
class RegionSelector(QWidget):
    region_selected = pyqtSignal(int, int, int, int)

    def __init__(self):
        super().__init__(None, Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setCursor(Qt.CrossCursor)
        self._start = None
        self._end   = None

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._start = e.globalPos()
            self._end   = e.globalPos()

    def mouseMoveEvent(self, e):
        if self._start:
            self._end = e.globalPos()
            self.update()

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton and self._start:
            self._end = e.globalPos()
            x = min(self._start.x(), self._end.x())
            y = min(self._start.y(), self._end.y())
            w = abs(self._end.x() - self._start.x())
            h = abs(self._end.y() - self._start.y())
            self.close()
            self.region_selected.emit(x, y, w, h)

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.close()

    def paintEvent(self, e):
        p = QPainter(self)
        # dark overlay
        p.fillRect(self.rect(), QColor(0, 0, 0, 120))
        if self._start and self._end:
            x = min(self._start.x(), self._end.x())
            y = min(self._start.y(), self._end.y())
            w = abs(self._end.x() - self._start.x())
            h = abs(self._end.y() - self._start.y())
            # cut out the selected area
            p.setCompositionMode(QPainter.CompositionMode_Clear)
            p.fillRect(x, y, w, h, Qt.transparent)
            p.setCompositionMode(QPainter.CompositionMode_SourceOver)
            # border
            p.setPen(QPen(QColor(0, 200, 255), 2, Qt.DashLine))
            p.drawRect(x, y, w, h)
            # size label
            p.setPen(QColor(0, 200, 255))
            p.drawText(x + 4, y - 6, f"{w} × {h} px")
        # instructions
        p.setPen(QColor(255, 255, 255, 180))
        p.drawText(
            self.rect().adjusted(0, 20, 0, 0),
            Qt.AlignHCenter | Qt.AlignTop,
            "Drag to select the Webots window   •   ESC to cancel"
        )

class RobotCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._angles = [j["v"] for j in PANDA_JOINTS]
        self.setMinimumSize(348, 312)

        # ── orbit state (like OrbitControls)
        self._azimuth   =  35.0   # degrees, horizontal rotation
        self._elevation =  25.0   # degrees, vertical tilt
        self._zoom      = 1.0     # scale factor
        self._pan_x     = 0.0     # pixel offset
        self._pan_y     = 0.0

        # ── drag state
        self._drag_btn  = None
        self._drag_last = QPoint()

        self.setMouseTracking(False)
        self.setCursor(Qt.OpenHandCursor)

    # ── public
    def set_angles(self, a):
        self._angles = list(a)
        self.update()

    # ── input events
    def mousePressEvent(self, e):
        self._drag_btn  = e.button()
        self._drag_last = e.pos()
        self.setCursor(Qt.ClosedHandCursor if e.button() == Qt.LeftButton
                       else Qt.SizeAllCursor)

    def mouseMoveEvent(self, e):
        if self._drag_btn is None:
            return
        dx = e.x() - self._drag_last.x()
        dy = e.y() - self._drag_last.y()
        self._drag_last = e.pos()

        if self._drag_btn == Qt.LeftButton:
            # orbit — match Three.js sensitivity
            self._azimuth   += dx * 0.55
            self._elevation -= dy * 0.55
            # clamp elevation so arm never flips upside-down weirdly
            self._elevation  = max(-89.0, min(89.0, self._elevation))
        elif self._drag_btn == Qt.RightButton:
            # pan
            self._pan_x += dx
            self._pan_y += dy
        self.update()

    def mouseReleaseEvent(self, e):
        self._drag_btn = None
        self.setCursor(Qt.OpenHandCursor)

    def mouseDoubleClickEvent(self, e):
        # reset to default view
        self._azimuth   =  35.0
        self._elevation =  25.0
        self._zoom      =  1.0
        self._pan_x     =  0.0
        self._pan_y     =  0.0
        self.update()

    def wheelEvent(self, e):
        delta = e.angleDelta().y()
        factor = 1.12 if delta > 0 else 0.89
        self._zoom = max(0.25, min(5.0, self._zoom * factor))
        self.update()

    # ── 3D projection helpers
    def _project(self, x3, y3, z3, cx, cy, scale):
        """
        Project a 3D point (x3 right, y3 up, z3 toward viewer)
        to 2D canvas using azimuth/elevation rotation then
        perspective-like scale.
        """
        az  = math.radians(self._azimuth)
        el  = math.radians(self._elevation)

        # rotate around Y axis (azimuth)
        rx =  x3 * math.cos(az) + z3 * math.sin(az)
        ry =  y3
        rz = -x3 * math.sin(az) + z3 * math.cos(az)

        # rotate around X axis (elevation)
        fx =  rx
        fy =  ry * math.cos(el) - rz * math.sin(el)
        fz =  ry * math.sin(el) + rz * math.cos(el)

        # mild perspective (fz offset shifts depth)
        persp = 1.0 / (1.0 + fz * 0.0015)

        sx = cx + self._pan_x + fx * scale * self._zoom * persp
        sy = cy + self._pan_y - fy * scale * self._zoom * persp
        return int(sx), int(sy)

    # ── forward kinematics — returns list of 3D joint positions
    def _fk(self):
        """
        Simplified FK for Franka Panda — enough to look correct.
        Each joint rotates the accumulation matrix.
        Link lengths (mm, scaled to canvas units):
          base→J1: 60  (up)
          J1→J2:   55  (up)
          J2→J3:   50  (up+forward)
          J3→J4:   46  (up)
          J4→J5:   38  (up+forward)
          J5→J6:   30  (up)
          J6→J7:   20  (up)
          J7→TCP:  14  (up)
        """
        a = self._angles
        pts = []          # list of (x, y, z) in world space

        # We build the chain using a 3×3 rotation matrix approach
        # Start at origin, arm points upward initially

        # --- pure-Python fallback (no numpy needed) ---
        def rot_x(t):
            c, s = math.cos(t), math.sin(t)
            return [[1,0,0],[0,c,-s],[0,s,c]]
        def rot_y(t):
            c, s = math.cos(t), math.sin(t)
            return [[c,0,s],[0,1,0],[-s,0,c]]
        def rot_z(t):
            c, s = math.cos(t), math.sin(t)
            return [[c,-s,0],[s,c,0],[0,0,1]]
        def mul(R, v):
            return [
                R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
                R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
                R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2],
            ]
        def mat_mul(A, B):
            C = [[0,0,0],[0,0,0],[0,0,0]]
            for i in range(3):
                for j in range(3):
                    for k in range(3):
                        C[i][j] += A[i][k]*B[k][j]
            return C
        def identity():
            return [[1,0,0],[0,1,0],[0,0,1]]

        # DH-inspired simplified chain
        # Joint axes for Panda (simplified):
        #   J1: rot_z   J2: rot_y   J3: rot_z
        #   J4: rot_y   J5: rot_z   J6: rot_y   J7: rot_z
        joint_axes = ['z','y','z','y','z','y','z']

        # link vectors (local "up" before rotation, canvas units)
        links = [
            [0, 60, 0],   # base → J1
            [0, 55, 0],   # J1   → J2
            [0, 50, 5],   # J2   → J3  (slight forward offset)
            [0, 46, 0],   # J3   → J4
            [0, 38, 5],   # J4   → J5
            [0, 30, 0],   # J5   → J6
            [0, 20, 0],   # J6   → J7
            [0, 14, 0],   # J7   → TCP
        ]

        R = identity()
        pos = [0.0, 0.0, 0.0]
        pts.append(tuple(pos))

        for i, (ax, link) in enumerate(zip(joint_axes, links[:7])):
            ang = math.radians(a[i]) if i < len(a) else 0.0
            if   ax == 'z': Rj = rot_z(ang)
            elif ax == 'y': Rj = rot_y(ang)
            else:           Rj = rot_x(ang)
            R = mat_mul(R, Rj)
            world_link = mul(R, link)
            pos = [pos[0]+world_link[0], pos[1]+world_link[1], pos[2]+world_link[2]]
            pts.append(tuple(pos))

        # TCP
        tcp_link = mul(R, links[7])
        tcp = (pos[0]+tcp_link[0], pos[1]+tcp_link[1], pos[2]+tcp_link[2])
        pts.append(tcp)

        return pts

    # ── paint
    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#040c14"))

        w, h = self.width(), self.height()
        cx = w // 2
        cy = h // 2 + 40          # shift centre down slightly so arm has headroom
        scale = min(w, h) * 0.0042 * 10  # base scale

        # ── grid (projected ground plane)
        p.setPen(QPen(QColor(0, 200, 255, 14), 1))
        grid_range = range(-120, 140, 40)
        for gx in grid_range:
            x1, y1 = self._project(gx, 0, -120, cx, cy, scale)
            x2, y2 = self._project(gx, 0,  120, cx, cy, scale)
            p.drawLine(x1, y1, x2, y2)
        for gz in grid_range:
            x1, y1 = self._project(-120, 0, gz, cx, cy, scale)
            x2, y2 = self._project( 120, 0, gz, cx, cy, scale)
            p.drawLine(x1, y1, x2, y2)

        # ── world axes (X=red, Y=green, Z=blue)
        axis_len = 45
        origin = self._project(0, 0, 0, cx, cy, scale)
        for vec, col in [
            ([axis_len, 0, 0], "#ff4d6d"),
            ([0, axis_len, 0], "#00ff9d"),
            ([0, 0, axis_len], "#00c8ff"),
        ]:
            tip = self._project(*vec, cx, cy, scale)
            p.setPen(QPen(QColor(col), 1, Qt.DashLine))
            p.drawLine(*origin, *tip)

        # ── base platform
        bpts = [self._project(gx, 0, gz, cx, cy, scale)
                for gx, gz in [(-18,-18),(18,-18),(18,18),(-18,18)]]
        poly = QPolygon([QPoint(*pt) for pt in bpts])
        p.setPen(QPen(QColor(C["a"]), 2))
        p.setBrush(QBrush(QColor(0, 200, 255, 35)))
        p.drawPolygon(poly)

        # ── robot arm
        try:
            pts3d = self._fk()
        except Exception:
            pts3d = []

        seg_colors = [C["a"], C["a2"], C["a"], C["a2"], C["a3"], C["a"], C["red"], C["a3"]]
        joint_radius = [7, 6, 6, 6, 5, 5, 5, 4]

        if pts3d:
            proj = [self._project(*pt, cx, cy, scale) for pt in pts3d]

            # draw links (thick lines)
            for i in range(len(proj) - 1):
                col = QColor(seg_colors[i % len(seg_colors)])
                # shadow / depth line
                p.setPen(QPen(QColor(col.red()//3, col.green()//3, col.blue()//3, 120), 6,
                              Qt.SolidLine, Qt.RoundCap))
                p.drawLine(*proj[i], *proj[i+1])
                # main line
                p.setPen(QPen(col, 3, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(*proj[i], *proj[i+1])

            # draw joints (circles)
            for i, (px, py) in enumerate(proj[:-1]):
                col   = QColor(seg_colors[i % len(seg_colors)])
                r     = joint_radius[i % len(joint_radius)]
                # glow
                glow = QRadialGradient(px, py, r * 2.5)
                glow.setColorAt(0, QColor(col.red(), col.green(), col.blue(), 80))
                glow.setColorAt(1, QColor(0, 0, 0, 0))
                p.setPen(Qt.NoPen)
                p.setBrush(QBrush(glow))
                p.drawEllipse(px - r*2, py - r*2, r*4, r*4)
                # filled joint
                p.setPen(QPen(col, 2))
                p.setBrush(QBrush(QColor(C["bg"])))
                p.drawEllipse(px - r, py - r, r*2, r*2)
                # joint label
                p.setPen(QPen(QColor(C["t3"])))
                font = p.font(); font.setPointSize(6); p.setFont(font)
                p.drawText(px + r + 2, py + 4,
                           f"J{i+1}" if i < 7 else "TCP")

            # TCP marker (orange diamond)
            tx, ty = proj[-1]
            p.setPen(QPen(QColor(C["a3"]), 2))
            p.setBrush(QBrush(QColor(C["a3"])))
            diamond = QPolygon([
                QPoint(tx,    ty - 6),
                QPoint(tx + 5,ty),
                QPoint(tx,    ty + 6),
                QPoint(tx - 5,ty),
            ])
            p.drawPolygon(diamond)
            p.setPen(QPen(QColor(C["a3"])))
            font = p.font(); font.setPointSize(6); p.setFont(font)
            p.drawText(tx + 7, ty + 4, "TCP")

        # ── corner hint labels
        p.setPen(QPen(QColor(C["t3"])))
        font = p.font(); font.setPointSize(6); p.setFont(font)
        p.drawText(4, h - 16, "🖱 drag: orbit   scroll: zoom   dbl-click: reset")
        p.drawText(4, h - 4,  f"az:{self._azimuth:.0f}°  el:{self._elevation:.0f}°  zoom:{self._zoom:.2f}x")


# ─────────────────────────────────────────────────────────────
#  TOP BAR
# ─────────────────────────────────────────────────────────────
class TopBar(QFrame):
    nav_changed   = pyqtSignal(str)
    estop_pressed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(52)
        self.setStyleSheet(
            f"TopBar {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            f"stop:0 #0b1926,stop:1 {C['bg']}); border-bottom:1px solid {C['b']}; }}"
        )
        main = QHBoxLayout(self)
        main.setContentsMargins(18, 0, 18, 0)

        # Logo
        logo = QLabel("FR")
        logo.setFixedSize(28, 28)
        logo.setAlignment(Qt.AlignCenter)
        logo.setStyleSheet(
            f"color:{C['a']}; border:2px solid {C['a']}; border-radius:5px;"
            f"font-family:'Rajdhani'; font-size:12px; font-weight:700; background:transparent;"
        )
        name = QLabel(f"PANDA 7-DOF <span style='color:{C['a']}'> CONTROL</span>")
        name.setTextFormat(Qt.RichText)
        name.setStyleSheet(
            f"font-family:'Rajdhani'; font-size:18px; font-weight:700;"
            f"color:{C['t1']}; background:transparent;"
        )
        sub = QLabel("FRANKA EMIKA · 7-DOF COBOT · HMI v5.0  |  panda_arm_controller")
        sub.setStyleSheet(
            f"font-family:'Share Tech Mono'; font-size:8px; color:{C['t3']};"
            f"letter-spacing:0.18em; background:transparent;"
        )
        info_col = QVBoxLayout(); info_col.setSpacing(0)
        info_col.addWidget(name); info_col.addWidget(sub)
        main.addWidget(logo); main.addLayout(info_col)
        main.addStretch()

        # Nav
        self._btn_dash = QPushButton("DASHBOARD")
        self._btn_ctrl = QPushButton("CONTROL")
        for btn, pg in [(self._btn_dash, "dash"), (self._btn_ctrl, "ctrl")]:
            btn.setFixedHeight(28)
            btn.clicked.connect(lambda _, p=pg: self._nav(p))
            main.addWidget(btn)
        self._active = "dash"
        self._update_nav()
        main.addSpacing(12)

        # Status pill
        self.pill = QLabel("● CONNECTING")
        self.pill.setStyleSheet(
            f"background:rgba(255,61,90,0.07); border:1px solid rgba(255,61,90,0.22);"
            f"border-radius:12px; color:{C['red']}; font-family:'Share Tech Mono';"
            f"font-size:9px; padding:3px 10px;"
        )
        main.addWidget(self.pill)

        # Clock
        self.clock = QLabel("--:--:--")
        self.clock.setStyleSheet(
            f"font-family:'Share Tech Mono'; font-size:11px; color:{C['t2']};"
            f"min-width:66px; background:transparent;"
        )
        main.addWidget(self.clock)

        # E-STOP
        es = QPushButton("⬛ E-STOP")
        es.setStyleSheet(
            f"QPushButton {{ background:rgba(255,61,90,0.1); border:2px solid {C['red']};"
            f"  border-radius:4px; color:{C['red']}; font-family:'Rajdhani';"
            f"  font-size:11px; font-weight:700; letter-spacing:0.15em; padding:5px 13px; }}"
            f"QPushButton:hover {{ background:rgba(255,61,90,0.22); }}"
            f"QPushButton:pressed {{ background:{C['red']}; color:#fff; }}"
        )
        es.clicked.connect(self.estop_pressed)
        main.addWidget(es)

    def _nav_style(self, active=False):
        if active:
            return (
                f"QPushButton {{ padding:5px 16px; background:rgba(0,200,255,0.09);"
                f"border:1px solid {C['a']}; border-radius:4px; color:{C['a']};"
                f"font-family:'Rajdhani'; font-size:11px; font-weight:600; letter-spacing:0.1em; }}"
            )
        return (
            f"QPushButton {{ padding:5px 16px; background:transparent;"
            f"border:1px solid {C['b']}; border-radius:4px; color:{C['t2']};"
            f"font-family:'Rajdhani'; font-size:11px; font-weight:600; letter-spacing:0.1em; }}"
            f"QPushButton:hover {{ border-color:{C['a']}; color:{C['a']}; }}"
        )

    def _update_nav(self):
        self._btn_dash.setStyleSheet(self._nav_style(self._active == "dash"))
        self._btn_ctrl.setStyleSheet(self._nav_style(self._active == "ctrl"))

    def _nav(self, pg):
        self._active = pg
        self._update_nav()
        self.nav_changed.emit(pg)

    def set_ros_status(self, online: bool):
        if online:
            self.pill.setText("● ROS 2 ONLINE")
            self.pill.setStyleSheet(
                f"background:rgba(0,230,118,0.07); border:1px solid rgba(0,230,118,0.22);"
                f"border-radius:12px; color:{C['ok']}; font-family:'Share Tech Mono';"
                f"font-size:9px; padding:3px 10px;"
            )
        else:
            self.pill.setText("● OFFLINE")
            self.pill.setStyleSheet(
                f"background:rgba(255,61,90,0.07); border:1px solid rgba(255,61,90,0.22);"
                f"border-radius:12px; color:{C['red']}; font-family:'Share Tech Mono';"
                f"font-size:9px; padding:3px 10px;"
            )

    def tick(self):
        self.clock.setText(datetime.now().strftime("%H:%M:%S"))

    def paintEvent(self, e):
        super().paintEvent(e)
        p = QPainter(self)
        grad = QLinearGradient(0, self.height(), self.width(), self.height())
        grad.setColorAt(0,   QColor(0, 200, 255, 0))
        grad.setColorAt(0.5, QColor(0, 200, 255, 82))
        grad.setColorAt(1,   QColor(0, 200, 255, 0))
        p.setPen(QPen(QBrush(grad), 1))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)


# ─────────────────────────────────────────────────────────────
#  DASHBOARD PAGE
# ─────────────────────────────────────────────────────────────
class DashPage(QWidget):
    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent)
        self._ros    = ros
        self._joints = joints          # shared reference from HMI
        self._build_ui()

    def _build_ui(self):
        g = QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0)
        g.setSpacing(5)
        g.setColumnStretch(1, 1)
        g.setRowStretch(0, 0)   # stats  — fixed
        g.setRowStretch(1, 0)   # tcp    — fixed
        g.setRowStretch(2, 0)   # joints — fixed to content
        g.setColumnMinimumWidth(0, 650)
        g.setColumnMinimumWidth(2, 195)

        # col-0 spans both rows: 3D viewer top, TCP bottom
        g.addWidget(self._make_3d(),    0, 0, 3, 1)   # spans all 3 rows
        g.addWidget(self._make_stats(), 0, 1)
        g.addWidget(self._make_tcp(),   1, 1)
        g.addWidget(self._make_joints(),2, 1)
        g.addWidget(self._make_right(), 0, 2, 3, 1)

    # ── 3D viewer — embeds live RViz window
    def _make_3d(self):
        f = QFrame()
        f.setStyleSheet(
            f"background:#040c14; border:1px solid {C['b']}; border-radius:7px;"
        )
        ly = QVBoxLayout(f); ly.setContentsMargins(0, 0, 0, 0)

        # header
        hdr = QFrame(); hdr.setFixedHeight(24)
        hdr.setStyleSheet(
            "background:rgba(4,12,20,0.88); border-bottom:1px solid rgba(0,200,255,0.1);"
        )
        hl = QHBoxLayout(hdr); hl.setContentsMargins(9, 0, 9, 0)
        hl.addWidget(make_label("PANDA 7-DOF VIEW", C["a"], 9, True))
        hl.addStretch()
        self._rviz_status = make_label("WAITING FOR RVIZ...", C["t3"], 8, mono=True)
        hl.addWidget(self._rviz_status)
        ly.addWidget(hdr)

        # container where RViz window will be embedded
        self._rviz_container = WebotsCaptureWidget()
        self._rviz_container.status_changed.connect(
            lambda msg, col: (
                self._rviz_status.setText(msg),
                self._rviz_status.setStyleSheet(
                    f"color:{col}; font-size:8px; font-family:'Share Tech Mono';"
                    f" background:transparent;"
                )
            )
        )
        ly.addWidget(self._rviz_container, 1)
        return f

    def _ros3d_html(self):
        return """<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8"/>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  html, body { width:100%; height:100%; background:#040c14; overflow:hidden; }
  #viewer { width:100%; height:100%; }
  #status {
    position:absolute; top:6px; right:8px;
    font-family:'Share Tech Mono',monospace; font-size:9px;
    color:#273d52; letter-spacing:.1em; pointer-events:none;
  }
</style>
</head>
<body>
<div id="viewer"></div>
<div id="status" id="st">INITIALIZING...</div>

<script src="https://cdn.jsdelivr.net/npm/three@0.89.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.4/lib/eventemitter2.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
<script src="https://cdn.jsdelivr.net/gh/RobotWebTools/ros3djs@develop/build/ros3d.min.js"></script>
<script>
var ROS_URL  = 'ws://localhost:9090';
var URDF_URL = 'http://localhost:8080/';
var FRAME    = 'panda_link0';

function st(msg, col) {
  var el = document.getElementById('status');
  el.innerText = msg;
  el.style.color = col || '#273d52';
}

var ros = new ROSLIB.Ros({ url: ROS_URL });

ros.on('connection', function() {
  st('ROS 2 ONLINE', '#00e676');
  init3D();
});
ros.on('error',  function() { st('ROS ERROR', '#ff3d5a'); setTimeout(function(){ ros.connect(ROS_URL); }, 3000); });
ros.on('close',  function() { st('DISCONNECTED', '#ffb830'); setTimeout(function(){ ros.connect(ROS_URL); }, 3000); });

function init3D() {
  var el = document.getElementById('viewer');
  var viewer = new ROS3D.Viewer({
    divID: 'viewer',
    width:  el.offsetWidth  || window.innerWidth,
    height: el.offsetHeight || window.innerHeight,
    antialias: true,
    background: '#040c14'
  });

  viewer.addObject(new ROS3D.Grid({
    color: '#0a1f2e',
    cellSize: 0.25,
    num_cells: 20
  }));

  // Lighting
  var amb = new THREE.AmbientLight(0xffffff, 0.6);
  viewer.scene.add(amb);
  var dir = new THREE.DirectionalLight(0xffffff, 0.9);
  dir.position.set(2, 3, 2);
  viewer.scene.add(dir);

  // Camera
  viewer.camera.position.set(1.5, 1.5, 1.5);
  viewer.camera.lookAt(new THREE.Vector3(0, 0.5, 0));

  // TF client
  var tfClient = new ROSLIB.TFClient({
    ros: ros,
    fixedFrame: FRAME,
    angularThres: 0.01,
    transThres:   0.005,
    rate: 30
  });

  // URDF robot model
  new ROS3D.UrdfClient({
    ros: ros,
    tfClient: tfClient,
    path: URDF_URL,
    rootObject: viewer.scene,
    param: 'robot_description'
  });

  // Resize observer
  new ResizeObserver(function() {
    viewer.resize(el.offsetWidth, el.offsetHeight);
  }).observe(el);

  st('frame: ' + FRAME, '#273d52');
}
</script>
</body>
</html>"""

    # ── TCP panel
    def _make_tcp(self):
        pnl  = Panel("TCP POSITION", "CARTESIAN")
        pnl.setMaximumHeight(267)
        grid = QGridLayout(); grid.setSpacing(3)
        self._tcp = {
            "X":  TcpCell("X AXIS",  "#ff4d6d"),
            "Y":  TcpCell("Y AXIS",  C["a2"]),
            "Z":  TcpCell("Z AXIS",  C["a"]),
            "RX": TcpCell("RX", C["a3"], "°"),
            "RY": TcpCell("RY", C["a3"], "°"),
            "RZ": TcpCell("RZ", C["a3"], "°"),
        }
        for i, (_, w) in enumerate(self._tcp.items()):
            grid.addWidget(w, i // 3, i % 3)
        pnl.body.addLayout(grid)
        return pnl

    # ── Stat cards
    def _make_stats(self):
        w = QWidget(); w.setStyleSheet("background:transparent;")
        w.setMaximumHeight(267)
        g = QGridLayout(w); g.setSpacing(6); g.setContentsMargins(0, 0, 0, 0)
        self._sc_state = StatCard("ROBOT STATE",    "STANDBY", "READY FOR COMMAND", "g")
        self._sc_spd   = StatCard("TCP SPEED",      "0 mm/s",  "MAX 1500 mm/s",     "b")
        self._sc_prog  = StatCard("ACTIVE PROGRAM", "NONE",    "NO FILE LOADED",    "b")
        self._sc_reach = StatCard("REACH",          "855 mm",  "PANDA MAX",         "o")
        self._sc_up    = StatCard("UPTIME",         "00:00",   "SESSION TIME",      "g")
        self._sc_err   = StatCard("ERRORS",         "0",       "ACTIVE FAULTS",     "r")
        for i, c in enumerate([self._sc_state, self._sc_spd, self._sc_prog,
                                self._sc_reach, self._sc_up,  self._sc_err]):
            g.addWidget(c, i // 3, i % 3)
            g.setColumnStretch(i % 3, 1)
        return w

    # ── 7 joint cards  (4 + 3 layout)
    def _make_joints(self):
        w = QWidget(); w.setStyleSheet("background:transparent;")
        w.setSizePolicy(w.sizePolicy().horizontalPolicy(),
                        __import__("PyQt5.QtWidgets", fromlist=["QSizePolicy"]).QSizePolicy.Fixed)
        g = QGridLayout(w); g.setSpacing(3); g.setContentsMargins(0, 0, 0, 0)
        self._jcards = []
        cols = 4                        # row-0: J1..J4   row-1: J5..J7
        for i, j in enumerate(self._joints):
            card = JointCard(j["id"], j["ros"], j["nm"], j["mn"], j["mx"])
            g.addWidget(card, i // cols, i % cols)
            g.setColumnStretch(i % cols, 1)
            self._jcards.append(card)
        # no row stretch — cards pack to their natural height
        return w

    # ── Right column
    def _make_right(self):
        w = QWidget(); w.setStyleSheet("background:transparent;")
        w.setFixedWidth(200)
        ly = QVBoxLayout(w); ly.setContentsMargins(0, 0, 0, 0); ly.setSpacing(5)

        # Program loader
        prog = Panel("PROGRAM LOADER", "NO FILE")
        self._file_lbl  = make_label("Drop or click to load", C["t2"], 8, mono=True)
        self._file_lbl.setAlignment(Qt.AlignCenter)
        self._file_lbl.setWordWrap(True)
        load_btn = QPushButton("📁  BROWSE FILE")
        load_btn.setStyleSheet(
            f"QPushButton {{ background:transparent; border:2px dashed {C['bb']};"
            f"  border-radius:5px; color:{C['t2']}; font-family:'Share Tech Mono';"
            f"  font-size:8px; padding:5px; letter-spacing:0.12em; }}"
            f"QPushButton:hover {{ border-color:{C['a']}; color:{C['a']}; background:rgba(0,200,255,0.04); }}"
        )
        load_btn.clicked.connect(self._browse)
        self._pbar = QProgressBar()
        self._pbar.setRange(0, 100); self._pbar.setValue(0)
        self._pbar.setTextVisible(False); self._pbar.setFixedHeight(3)
        self._pbar.setStyleSheet(
            f"QProgressBar {{ background:rgba(255,255,255,0.04); border-radius:2px; border:none; }}"
            f"QProgressBar::chunk {{ background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            f"stop:0 {C['a2']},stop:1 {C['a']}); border-radius:2px; }}"
        )
        self._pbar_lbl = make_label("READY", C["t3"], 7, mono=True)
        run_btn = QPushButton("▶  START")
        run_btn.setStyleSheet(
            f"QPushButton {{ background:rgba(0,255,157,0.1); border:2px solid {C['a2']};"
            f"  border-radius:6px; color:{C['a2']}; font-family:'Rajdhani';"
            f"  font-size:11px; font-weight:700; letter-spacing:0.12em; padding:5px; }}"
            f"QPushButton:hover {{ background:rgba(0,255,157,0.2); }}"
        )
        stop_btn = QPushButton("⏹  STOP")
        stop_btn.setStyleSheet(
            f"QPushButton {{ background:rgba(255,61,90,0.1); border:2px solid {C['red']};"
            f"  border-radius:6px; color:{C['red']}; font-family:'Rajdhani';"
            f"  font-size:11px; font-weight:700; letter-spacing:0.12em; padding:5px; }}"
            f"QPushButton:hover {{ background:rgba(255,61,90,0.2); }}"
        )
        run_btn.clicked.connect(self._run_prog)
        stop_btn.clicked.connect(self._stop_prog)
        btns = QHBoxLayout()
        btns.addWidget(run_btn); btns.addWidget(stop_btn)
        prog.body.addWidget(load_btn)
        prog.body.addWidget(self._file_lbl)
        prog.body.addWidget(self._pbar_lbl)
        prog.body.addWidget(self._pbar)
        prog.body.addLayout(btns)
        ly.addWidget(prog, 0)

        # Diagnostics
        diag = Panel("DIAGNOSTICS")
        self._diag = {}
        for key, label, color in [
            ("temp", "JOINT TEMP AVG", C["a"]),
            ("pwr",  "POWER DRAW",     C["a2"]),
            ("lat",  "ROS LATENCY",    C["a"]),
        ]:
            row = QHBoxLayout()
            lbl = make_label(label, C["t3"], 6, mono=True); lbl.setFixedWidth(64)
            bar = QProgressBar()
            bar.setRange(0, 100); bar.setValue(40)
            bar.setTextVisible(False); bar.setFixedHeight(3)
            bar.setStyleSheet(
                f"QProgressBar {{ background:rgba(255,255,255,0.04); border-radius:2px; border:none; }}"
                f"QProgressBar::chunk {{ background:{color}; border-radius:2px; }}"
            )
            val = make_label("--", C["t1"],  9, True)
            val.setFixedWidth(30); val.setAlignment(Qt.AlignRight)
            row.addWidget(lbl); row.addWidget(bar); row.addWidget(val)
            self._diag[key] = (bar, val)
            diag.body.addLayout(row)
        ly.addWidget(diag, 0)

        # Log
        log_pnl = Panel("SYSTEM LOG")
        self._log = LogWidget()
        log_pnl.body.addWidget(self._log)
        ly.addWidget(log_pnl, 1)
        return w

    # ── Program helpers
    _prog_file  = None
    _prog_pct   = 0.0
    _prog_timer = None

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Program", "",
            "Robot Programs (*.script *.txt *.json *.py *.xml);;All Files (*)"
        )
        if path:
            self._prog_file = path
            self._file_lbl.setText(f"📄 {path.split('/')[-1]}")
            self._pbar_lbl.setText("LOADED")
            self.log(f"Loaded: {path.split('/')[-1]}", "ok")

    def _run_prog(self):
        if not self._prog_file:
            self.log("No program loaded", "err"); return
        self._prog_pct = 0.0
        self._pbar_lbl.setText("RUNNING")
        self._sc_state.set_value("RUNNING", C["a2"])
        self._prog_timer = QTimer(self)
        self._prog_timer.timeout.connect(self._tick_prog)
        self._prog_timer.start(200)
        self._ros.publish_cmd(f"RUN:{self._prog_file}")

    def _tick_prog(self):
        self._prog_pct = min(100.0, self._prog_pct + random.uniform(0.5, 2.5))
        self._pbar.setValue(int(self._prog_pct))
        if self._prog_pct >= 100.0:
            self._prog_timer.stop()
            self._pbar_lbl.setText("COMPLETE")
            self._sc_state.set_value("DONE", C["a"])
            self.log("Program complete", "ok")

    def _stop_prog(self):
        if self._prog_timer: self._prog_timer.stop()
        self._pbar.setValue(0); self._prog_pct = 0.0
        self._pbar_lbl.setText("STOPPED")
        self._sc_state.set_value("STOPPED", C["red"])
        self.log("Program stopped", "warn")
        self._ros.publish_cmd("STOP")

    # ── Public API called by HMI
    def update_from_ros(self, pos, vel, eff):
        """Called when /joint_states arrives (radians → degrees)."""
        for i, card in enumerate(self._jcards):
            if i >= len(pos): break
            deg = math.degrees(pos[i])
            v   = math.degrees(vel[i]) if i < len(vel) else 0.0
            e   = eff[i] if i < len(eff) else 0.0
            self._joints[i]["v"] = deg
            card.update_data(deg, v, e)

    def update_demo(self, t):
        """Demo animation — runs when ROS is not connected."""
        for i, j in enumerate(self._joints):
            j["v"] = max(j["mn"], min(j["mx"], j["v"] + (random.random() - 0.5) * 0.3))
            vel = (random.random() - 0.5) * 2.0
            eff = (random.random() - 0.5) * 5.0
            self._jcards[i].update_data(j["v"], vel, eff)

        x  = math.sin(t / 3) * 200
        y  = math.cos(t / 4) * 150
        z  = 600 + math.sin(t / 5) * 50
        self._tcp["X"].set_value(x); self._tcp["Y"].set_value(y)
        self._tcp["Z"].set_value(z, 0, 1200)
        self._tcp["RX"].set_value(math.sin(t / 6) * 30, -180, 180)
        self._tcp["RY"].set_value(math.cos(t / 7) * 20, -180, 180)
        self._tcp["RZ"].set_value(math.sin(t / 8) * 45, -180, 180)
        self._sc_spd.set_value(f"{abs(math.sin(t/2)*80):.0f} mm/s")

        tmp = 38 + random.random() * 8
        pwr = 100 + random.random() * 80
        lat = 5   + random.random() * 15
        self._diag["temp"][0].setValue(int(tmp))
        self._diag["temp"][1].setText(f"{tmp:.0f}°C")
        self._diag["pwr"][0].setValue(int(pwr / 480 * 100))
        self._diag["pwr"][1].setText(f"{pwr:.0f}W")
        self._diag["lat"][0].setValue(int(lat))
        self._diag["lat"][1].setText(f"{lat:.0f}ms")

        # RViz viewer updates live from /joint_states — no manual push needed

    def log(self, msg, kind="info"):
        return self._log.add(msg, kind)


# ─────────────────────────────────────────────────────────────
#  CONTROL PAGE
# ─────────────────────────────────────────────────────────────
class CtrlPage(QWidget):
    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent)
        self._ros    = ros
        self._joints = joints          # shared reference
        self._jx = self._jy = self._jz = self._jwz = 0.0
        self._joy_timer = QTimer(self)
        self._joy_timer.timeout.connect(self._pub_twist)
        self._build_ui()

    def _build_ui(self):
        g = QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0); g.setSpacing(7)
        g.setColumnStretch(1, 1)
        g.setColumnMinimumWidth(0, 650)
        g.setColumnMinimumWidth(2, 200)

        # ══════════════════════════════════════════════════════
        # LEFT COLUMN — 3D viewer only (full height)
        # ══════════════════════════════════════════════════════
        v3d_frame = QFrame()
        v3d_frame.setStyleSheet(
            f"background:#040c14; border:1px solid {C['b']}; border-radius:7px;"
        )
        v3d_ly = QVBoxLayout(v3d_frame); v3d_ly.setContentsMargins(0, 0, 0, 0)

        v3d_hdr = QFrame(); v3d_hdr.setFixedHeight(24)
        v3d_hdr.setStyleSheet(
            "background:rgba(4,12,20,0.88); border-bottom:1px solid rgba(0,200,255,0.1);"
        )
        v3d_hl = QHBoxLayout(v3d_hdr); v3d_hl.setContentsMargins(9, 0, 9, 0)
        v3d_hl.addWidget(make_label("PANDA 7-DOF VIEW", C["a"], 9, True))
        v3d_hl.addStretch()
        self._ctrl_angle_lbl = make_label("J1: 0.0°", C["t3"], 8, mono=True)
        v3d_hl.addWidget(self._ctrl_angle_lbl)
        v3d_ly.addWidget(v3d_hdr)

        self._ctrl_canvas = WebotsCaptureWidget()
        self._ctrl_canvas.status_changed.connect(
            lambda msg, col: (
                self._ctrl_angle_lbl.setText(msg),
                self._ctrl_angle_lbl.setStyleSheet(
                    f"color:{col}; font-size:8px; font-family:'Share Tech Mono';"
                    f" background:transparent;"
                )
            )
        )
        v3d_ly.addWidget(self._ctrl_canvas, 1)

        scan_lbl = QLabel(); scan_lbl.setFixedHeight(1)
        scan_lbl.setStyleSheet(
            f"background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            f"stop:0 transparent, stop:0.5 rgba(0,200,255,55), stop:1 transparent);"
        )
        v3d_ly.addWidget(scan_lbl)
        g.addWidget(v3d_frame, 0, 0, 2, 1)

        # ══════════════════════════════════════════════════════
        # CENTER COLUMN — mode + joint sliders + commands
        # ══════════════════════════════════════════════════════
        center = QScrollArea(); center.setWidgetResizable(True)
        center.setStyleSheet("background:transparent; border:none;")
        cw = QWidget(); cw.setStyleSheet("background:transparent;")
        cl = QVBoxLayout(cw); cl.setSpacing(7)

        # Operation mode
        mode_pnl = Panel("OPERATION MODE")
        mg = QGridLayout(); mg.setSpacing(4)
        self._mode_btns = []
        for i, m in enumerate(["MANUAL", "AUTO", "TEACH", "SIMULATION"]):
            b = QPushButton(m); b.setCheckable(True); b.setChecked(m == "MANUAL")
            b.setStyleSheet(self._mode_style())
            b.clicked.connect(lambda _, btn=b, md=m: self._set_mode(btn, md))
            mg.addWidget(b, i // 2, i % 2)
            self._mode_btns.append(b)
        mode_pnl.body.addLayout(mg)
        cl.addWidget(mode_pnl)

        # ── Joint sliders (moved from left column into center)
        s_pnl = Panel("JOINT CONTROL", "7-DOF · DEG")
        sa = QScrollArea(); sa.setWidgetResizable(True)
        sa.setStyleSheet("background:transparent; border:none;")
        sw = QWidget(); sw.setStyleSheet("background:transparent;")
        sl = QVBoxLayout(sw); sl.setSpacing(9)
        self._sliders = []
        for i, j in enumerate(self._joints):
            box = QVBoxLayout(); box.setSpacing(3)
            hdr = QHBoxLayout()
            hdr.addWidget(make_label(f"{j['id']} · {j['ros']}", C["t2"], 10, True))
            hdr.addStretch()
            sv = make_label(f"{j['v']:.1f}°", C["a"], 10, mono=True)
            hdr.addWidget(sv)
            s = QSlider(Qt.Horizontal)
            s.setRange(int(j["mn"] * 10), int(j["mx"] * 10))
            s.setValue(int(j["v"] * 10))
            s.setStyleSheet(slider_style(C["a"]))
            lims = QHBoxLayout()
            lims.addWidget(make_label(f"{j['mn']}°", C["t3"], 7, mono=True))
            lims.addStretch()
            lims.addWidget(make_label(f"{j['mx']}°", C["t3"], 7, mono=True))
            s.valueChanged.connect(
                lambda val, idx=i, lbl=sv: self._on_slider(idx, val / 10, lbl)
            )
            box.addLayout(hdr); box.addWidget(s); box.addLayout(lims)
            sl.addLayout(box)
            self._sliders.append((s, sv))
        sa.setWidget(sw)
        s_pnl.body.addWidget(sa)
        cl.addWidget(s_pnl, 1)   # stretch=1 so sliders fill available space

        # Robot commands
        cmd_pnl = Panel("ROBOT COMMANDS")
        cg = QGridLayout(); cg.setSpacing(5)
        for i, (ic, lb, kd) in enumerate([
            ("▶", "START", "start"), ("⏹", "STOP",  "stop"),   ("⏸", "PAUSE", "normal"),
            ("⌂", "HOME",  "normal"),("○", "ZERO",  "normal"), ("↺", "RESET", "normal"),
        ]):
            b = CmdButton(ic, lb, kd)
            b.clicked.connect(lambda _, c=lb: self._cmd(c))
            cg.addWidget(b, i // 3, i % 3)
        cmd_pnl.body.addLayout(cg)
        cl.addWidget(cmd_pnl)
        center.setWidget(cw)
        g.addWidget(center, 0, 1, 2, 1)

        # initialise stub joystick attrs so _joy_stop etc. don't crash
        self._jxy = None
        self._joy_spd = None

        # ── Right: limits + tasks
        rs = QScrollArea(); rs.setWidgetResizable(True)
        rs.setStyleSheet("background:transparent; border:none;")
        rw = QWidget(); rw.setStyleSheet("background:transparent;")
        rl = QVBoxLayout(rw); rl.setSpacing(7)
        lim_pnl = Panel("SPEED & TORQUE", "LIMITS")
        for lbl, mn, mx, val, unit, color in [
            ("JOINT SPEED",     0,  100, 100, "%",    C["a2"]),
            ("TCP SPEED LIMIT", 0, 1500,1500, "mm/s", C["a2"]),
            ("TORQUE LIMIT",    0,  100, 100, "%",    C["a3"]),
            ("COLLISION SENSE", 0,  100,  50, "%",    C["a3"]),
        ]:
            lh = QHBoxLayout()
            lh.addWidget(make_label(lbl, C["t2"], 8, mono=True)); lh.addStretch()
            vl = make_label(f"{val}{unit}", C["t1"], 13, True); lh.addWidget(vl)
            s = QSlider(Qt.Horizontal)
            s.setRange(mn, mx); s.setValue(val)
            s.setStyleSheet(slider_style(color))
            s.valueChanged.connect(lambda v, u=unit, l=vl: l.setText(f"{v}{u}"))
            lims = QHBoxLayout()
            lims.addWidget(make_label(str(mn), C["t3"], 7, mono=True))
            lims.addStretch()
            lims.addWidget(make_label(str(mx), C["t3"], 7, mono=True))
            lim_pnl.body.addLayout(lh); lim_pnl.body.addWidget(s); lim_pnl.body.addLayout(lims)
        rl.addWidget(lim_pnl)

        task_pnl = Panel("TASK PROGRAMS", "panda_ws")
        for no, nm, st in [
            ("01", "Home Position",      "rdy"),
            ("02", "Pick & Place A→B",   "idle"),
            ("03", "Inspection Scan",    "idle"),
            ("04", "Calibration Routine","done"),
            ("05", "Custom Trajectory",  "idle"),
        ]:
            f = QFrame()
            f.setStyleSheet(
                f"QFrame {{ background:rgba(0,0,0,0.25); border:1px solid {C['b']}; border-radius:5px; }}"
                f"QFrame:hover {{ border-color:{C['bb']}; }}"
            )
            f.setCursor(Qt.PointingHandCursor)
            ly2 = QHBoxLayout(f); ly2.setContentsMargins(9, 6, 9, 6)
            ly2.addWidget(make_label(no, C["t3"], 8, mono=True))
            ly2.addWidget(make_label(nm, C["t1"], 10))
            ly2.addStretch()
            sc, bg = {"rdy": (C["a2"],"rgba(0,255,157,0.07)"),
                      "done":(C["a"], "rgba(0,200,255,0.07)"),
                      "idle":(C["t3"],"rgba(255,255,255,0.02)")}.get(st,(C["t3"],"transparent"))
            sl2 = QLabel(st.upper())
            sl2.setStyleSheet(
                f"color:{sc}; background:{bg}; border:1px solid {sc}40;"
                f"border-radius:10px; font-size:7px; padding:2px 5px; font-family:'Share Tech Mono';"
            )
            ly2.addWidget(sl2)
            task_pnl.body.addWidget(f)
        rl.addWidget(task_pnl); rl.addStretch()
        rs.setWidget(rw)
        g.addWidget(rs, 0, 2, 2, 1)

    def _ros3d_html_ctrl(self):
        """Same ros3djs viewer HTML as dashboard."""
        return """<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8"/>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  html, body { width:100%; height:100%; background:#040c14; overflow:hidden; }
  #viewer { width:100%; height:100%; }
  #status {
    position:absolute; top:6px; right:8px;
    font-family:'Share Tech Mono',monospace; font-size:9px;
    color:#273d52; letter-spacing:.1em; pointer-events:none;
  }
</style>
</head>
<body>
<div id="viewer"></div>
<div id="status" id="st">INITIALIZING...</div>

<script src="https://cdn.jsdelivr.net/npm/three@0.89.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.4/lib/eventemitter2.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
<script src="https://cdn.jsdelivr.net/gh/RobotWebTools/ros3djs@develop/build/ros3d.min.js"></script>
<script>
var ROS_URL  = 'ws://localhost:9090';
var URDF_URL = 'http://localhost:8080/';
var FRAME    = 'panda_link0';

function st(msg, col) {
  var el = document.getElementById('status');
  el.innerText = msg;
  el.style.color = col || '#273d52';
}

var ros = new ROSLIB.Ros({ url: ROS_URL });

ros.on('connection', function() {
  st('ROS 2 ONLINE', '#00e676');
  init3D();
});
ros.on('error',  function() { st('ROS ERROR', '#ff3d5a'); setTimeout(function(){ ros.connect(ROS_URL); }, 3000); });
ros.on('close',  function() { st('DISCONNECTED', '#ffb830'); setTimeout(function(){ ros.connect(ROS_URL); }, 3000); });

function init3D() {
  var el = document.getElementById('viewer');
  var viewer = new ROS3D.Viewer({
    divID: 'viewer',
    width:  el.offsetWidth  || window.innerWidth,
    height: el.offsetHeight || window.innerHeight,
    antialias: true,
    background: '#040c14'
  });

  viewer.addObject(new ROS3D.Grid({
    color: '#0a1f2e',
    cellSize: 0.25,
    num_cells: 20
  }));

  // Lighting
  var amb = new THREE.AmbientLight(0xffffff, 0.6);
  viewer.scene.add(amb);
  var dir = new THREE.DirectionalLight(0xffffff, 0.9);
  dir.position.set(2, 3, 2);
  viewer.scene.add(dir);

  // Camera
  viewer.camera.position.set(1.5, 1.5, 1.5);
  viewer.camera.lookAt(new THREE.Vector3(0, 0.5, 0));

  // TF client
  var tfClient = new ROSLIB.TFClient({
    ros: ros,
    fixedFrame: FRAME,
    angularThres: 0.01,
    transThres:   0.005,
    rate: 30
  });

  // URDF robot model
  new ROS3D.UrdfClient({
    ros: ros,
    tfClient: tfClient,
    path: URDF_URL,
    rootObject: viewer.scene,
    param: 'robot_description'
  });

  // Resize observer
  new ResizeObserver(function() {
    viewer.resize(el.offsetWidth, el.offsetHeight);
  }).observe(el);

  st('frame: ' + FRAME, '#273d52');
}
</script>
</body>
</html>"""

    def _mode_style(self):
        return (
            f"QPushButton {{ background:transparent; border:1px solid {C['b']}; border-radius:4px;"
            f"  color:{C['t2']}; font-family:'Share Tech Mono'; font-size:8.5px;"
            f"  letter-spacing:0.12em; padding:3px 6px; }}"
            f"QPushButton:checked {{ background:rgba(0,200,255,0.09); border-color:{C['a']}; color:{C['a']}; }}"
            f"QPushButton:hover {{ border-color:{C['a']}; color:{C['a']}; }}"
        )

    def _on_slider(self, i, val, lbl):
        self._joints[i]["v"] = val
        lbl.setText(f"{val:.1f}°")
        self._ros.publish_joints([j["v"] for j in self._joints])
        self._refresh_canvas()

    def _refresh_canvas(self):
        angles = [j["v"] for j in self._joints]
        if isinstance(self._ctrl_canvas, RobotCanvas): self._ctrl_canvas.set_angles(angles)
        peak = max(self._joints, key=lambda j: abs(j["v"]))
        self._ctrl_angle_lbl.setText(f"{peak['id']}: {peak['v']:.1f}°")

    def update_canvas_from_ros(self, angles_deg):
        if isinstance(self._ctrl_canvas, RobotCanvas): self._ctrl_canvas.set_angles(angles_deg)
        if angles_deg:
            peak_i = max(range(len(angles_deg)), key=lambda i: abs(angles_deg[i]))
            j = self._joints[peak_i]
            self._ctrl_angle_lbl.setText(f"{j['id']}: {angles_deg[peak_i]:.1f}°")

    def _on_jxy(self, nx, ny):
        self._jx = nx; self._jy = ny
        if not self._joy_timer.isActive():
            self._joy_timer.start(50)

    def _on_vj(self, key, val):
        if key == "z":
            self._jz  = val
        else:
            self._jwz = val
        if not self._joy_timer.isActive():
            self._joy_timer.start(50)

    def _joy_stop(self):
        self._jx = self._jy = self._jz = self._jwz = 0.0
        self._joy_timer.stop()
        self._ros.publish_twist(0, 0, 0, 0)

    def _pub_twist(self):
        s = (self._joy_spd.value() / 100 * 0.5) if self._joy_spd else 0.25
        self._ros.publish_twist(self._jx*s, self._jy*s, self._jz*s, self._jwz*s)

    def _set_mode(self, active_btn, mode):
        for b in self._mode_btns:
            b.setChecked(b is active_btn)
        self._ros.publish_cmd(f"MODE:{mode}")

    def _cmd(self, c):
        self._ros.publish_cmd(c)


# ─────────────────────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────────────────────
class HMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Franka Panda | Robot HMI v5.0")
        self.setMinimumSize(1200, 700)
        self.resize(1440, 860)
        self.setStyleSheet(STYLE_BASE)

        # ── Shared joint state (single source of truth)
        import copy
        self._joints = copy.deepcopy(PANDA_JOINTS)

        # ── ROS worker
        self._ros = RosWorker()
        self._ros.connection_changed.connect(self._on_ros_status)
        self._ros.joint_state_received.connect(self._on_joint_state)
        self._ros.log_message.connect(self._on_ros_log)
        self._ros.start()

        # ── UI
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(0)

        self._tb = TopBar()
        self._tb.nav_changed.connect(self._nav)
        self._tb.estop_pressed.connect(self._estop)
        root.addWidget(self._tb)

        self._stack = QStackedWidget()
        self._stack.setStyleSheet("background:transparent;")
        root.addWidget(self._stack, 1)

        self._dash = DashPage(self._ros, self._joints)
        self._ctrl = CtrlPage(self._ros, self._joints)
        self._stack.addWidget(_padded(self._dash))
        self._stack.addWidget(_padded(self._ctrl))
        self._pages = {"dash": 0, "ctrl": 1}

        # ── Timers
        self._clk_tmr = QTimer(self); self._clk_tmr.timeout.connect(self._tb.tick)
        self._clk_tmr.start(1000)

        self._up = 0
        self._up_tmr = QTimer(self); self._up_tmr.timeout.connect(self._tick_up)
        self._up_tmr.start(1000)

        self._demo_t  = 0.0
        self._demo_on = True           # flips to False once ROS connects
        self._demo_tmr = QTimer(self); self._demo_tmr.timeout.connect(self._tick_demo)
        self._demo_tmr.start(400)

        # ── Boot log
        self._dash.log("HMI v5.0 initialized — Franka Panda 7-DOF", "ok")
        self._dash.log("Connecting to ROS 2 node: panda_hmi …", "warn")
        self._dash.log(
            "Joint names: " + ", ".join(j["ros"] for j in PANDA_JOINTS), "info"
        )
        self._dash.log(
            "Publisher → /panda_arm_controller/joint_trajectory", "info"
        )

    def _nav(self, pg):
        self._stack.setCurrentIndex(self._pages.get(pg, 0))

    def _estop(self):
        self._dash.log("⚠ EMERGENCY STOP TRIGGERED", "err")
        self._ros.publish_twist(0, 0, 0, 0)
        self._ros.publish_cmd("ESTOP")
        self._dash._sc_state.set_value("E-STOP", C["red"])

    @pyqtSlot(bool)
    def _on_ros_status(self, online):
        self._tb.set_ros_status(online)
        self._demo_on = not online     # stop demo when real data flows in

    @pyqtSlot(list, list, list)
    def _on_joint_state(self, pos, vel, eff):
        self._dash.update_from_ros(pos, vel, eff)
        # keep control-page canvas in sync with live joint data
        angles_deg = [math.degrees(p) for p in pos]
        self._ctrl.update_canvas_from_ros(angles_deg)

    @pyqtSlot(str, str)
    def _on_ros_log(self, msg, kind):
        self._dash.log(msg, kind)

    def _tick_up(self):
        self._up += 1
        m, s = divmod(self._up, 60)
        self._dash._sc_up.set_value(f"{m:02d}:{s:02d}")

    def _tick_demo(self):
        if not self._demo_on:
            return
        self._demo_t += 0.4
        self._dash.update_demo(self._demo_t)
        # mirror demo animation into control page canvas
        angles = [j["v"] for j in self._joints]
        if isinstance(self._ctrl._ctrl_canvas, RobotCanvas): self._ctrl._ctrl_canvas.set_angles(angles)
        if angles:
            peak_i = max(range(len(angles)), key=lambda i: abs(angles[i]))
            j = self._joints[peak_i]
            self._ctrl._ctrl_angle_lbl.setText(f"{j['id']}: {j['v']:.1f}°")

    def closeEvent(self, e):
        self._ros.stop()
        self._ros.wait(2000)
        super().closeEvent(e)


# ─────────────────────────────────────────────────────────────
#  UTILITY
# ─────────────────────────────────────────────────────────────
def _padded(widget):
    w = QWidget(); w.setStyleSheet("background:transparent;")
    ly = QVBoxLayout(w); ly.setContentsMargins(7, 7, 7, 7); ly.setSpacing(0)
    ly.addWidget(widget)
    return w


# ─────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("Franka Panda HMI")
    app.setStyleSheet(STYLE_BASE)
    QFontDatabase.addApplicationFont("Rajdhani-Regular.ttf")
    QFontDatabase.addApplicationFont("Exo2-Regular.ttf")
    win = HMI()
    win.show()
    sys.exit(app.exec_())
