# ─────────────────────────────────────────────────────────────────────────────
#  control.py  —  Kontrol sayfası (2 sütunlu düzen)
#  Sol: Operasyon modu + Manuel eklem slider'ları
#  Sağ: Hızlı komutlar + Hız/Tork limitleri + Hazır görevler
# ─────────────────────────────────────────────────────────────────────────────

import math

from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QPushButton, QSlider, QFrame, QLabel,
)
from PyQt5.QtCore import Qt, QTimer

from config     import C
from widgets    import Panel, CmdButton, make_label, slider_style
from ros_worker import RosWorker


class CtrlPage(QWidget):
    """
    Kontrol sayfası.
    - Sol: Mod seçimi + 6-DOF slider kontrolü
    - Sağ: Hızlı komut butonları, güvenlik limitleri, hazır görevler
    """

    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent)
        self._ros    = ros
        self._joints = joints
        # Joystick durumu (şu an kullanılmıyor — ileride genişletilebilir)
        self._jx = self._jy = self._jz = self._jwz = 0.0
        self._joy_timer = QTimer(self)
        self._joy_timer.timeout.connect(self._pub_twist)
        self._build_ui()

    # ── Ana düzen ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        ml = QHBoxLayout(self)
        ml.setContentsMargins(15, 15, 15, 15)
        ml.setSpacing(15)

        ml.addLayout(self._make_left_col(), 1)
        ml.addLayout(self._make_right_col(), 1)

    # ── Sol sütun: Mod + Slider'lar ───────────────────────────────────────────
    def _make_left_col(self):
        col = QVBoxLayout(); col.setSpacing(15)
        col.addWidget(self._make_mode_panel())
        col.addWidget(self._make_slider_panel(), 1)
        return col

    def _make_mode_panel(self):
        pnl = Panel("OPERATION MODE")
        mg = QGridLayout(); mg.setSpacing(6)
        self._mode_btns = []
        for i, m in enumerate(["MANUAL", "AUTO", "TEACH", "SIMULATION"]):
            b = QPushButton(m)
            b.setCheckable(True)
            b.setChecked(m == "MANUAL")
            b.setStyleSheet(self._mode_style())
            b.clicked.connect(lambda _, btn=b, md=m: self._set_mode(btn, md))
            b.setMinimumHeight(35)
            mg.addWidget(b, i // 2, i % 2)
            self._mode_btns.append(b)
        pnl.body.addLayout(mg)
        return pnl

    def _make_slider_panel(self):
        pnl = Panel("MANUAL JOINT CONTROL", "6-DOF · DEG")
        sw = QWidget(); sw.setStyleSheet("background:transparent;")
        sl_ly = QVBoxLayout(sw); sl_ly.setSpacing(15)
        self._sliders = []
        
        self._slider_active = [False] * len(self._joints)

        for i, j in enumerate(self._joints):
            box = QVBoxLayout(); box.setSpacing(5)

            # Başlık satırı: eklem adı + anlık değer
            hdr = QHBoxLayout()
            hdr.addWidget(make_label(f"{j['id']} · {j['ros']}", C["t2"], 12, bold=True))
            hdr.addStretch()
            sv = make_label(f"{j['v']:.1f}°", C["a"], 12, mono=True)
            hdr.addWidget(sv)

            # Slider
            s = QSlider(Qt.Horizontal)
            s.setRange(int(j["mn"] * 10), int(j["mx"] * 10))
            s.setValue(int(j["v"] * 10))
            s.setStyleSheet(slider_style(C["a"]))
            s.valueChanged.connect(lambda val, idx=i, lbl=sv: self._on_slider(idx, val / 10, lbl))
            
            # Kullanıcı slider'ı tuttuğunda
            s.sliderPressed.connect(lambda idx=i: self._on_slider_pressed(idx))
            # Kullanıcı slider'ı bıraktığında
            s.sliderReleased.connect(lambda idx=i: self._on_slider_released(idx))

            # Min/Max etiketleri
            lims = QHBoxLayout()
            lims.addWidget(make_label(f"{j['mn']}°", C["t3"], 9, mono=True))
            lims.addStretch()
            lims.addWidget(make_label(f"{j['mx']}°", C["t3"], 9, mono=True))

            box.addLayout(hdr); box.addWidget(s); box.addLayout(lims)
            sl_ly.addLayout(box)
            self._sliders.append((s, sv))

        pnl.body.addWidget(sw)
        return pnl

    # ── Sağ sütun: Komutlar + Limitler + Görevler ─────────────────────────────
    def _make_right_col(self):
        col = QVBoxLayout(); col.setSpacing(15)
        col.addWidget(self._make_cmd_panel())
        col.addWidget(self._make_limits_panel())
        col.addWidget(self._make_tasks_panel())
        col.addStretch()
        return col

    def _make_cmd_panel(self):
        pnl = Panel("QUICK COMMANDS")
        cg = QGridLayout(); cg.setSpacing(8)
        for i, (ic, lb, kd) in enumerate([
            ("▶", "START",  "start"),
            ("⏹", "STOP",   "stop"),
            ("⏸", "PAUSE",  "normal"),
            ("⌂", "HOME",   "normal"),
            ("○", "ZERO",   "normal"),
            ("↺", "RESET",  "normal"),
        ]):
            b = CmdButton(ic, lb, kd)
            b.clicked.connect(lambda _, c=lb: self._cmd(c))
            cg.addWidget(b, i // 3, i % 3)
        pnl.body.addLayout(cg)
        return pnl

    def _make_limits_panel(self):
        pnl = Panel("SPEED & TORQUE LIMITS", "SAFETY")
        for lbl, mn, mx, val, unit, color in [
            ("JOINT SPEED",     0, 100,   100,  "%",    C["a2"]),
            ("TCP SPEED LIMIT", 0, 2500,  2500, "mm/s", C["a2"]),
            ("TORQUE LIMIT",    0, 100,   100,  "%",    C["a3"]),
            ("COLLISION SENSE", 0, 100,    50,  "%",    C["a3"]),
        ]:
            lh = QHBoxLayout()
            lh.addWidget(make_label(lbl, C["t2"], 10, mono=True))
            lh.addStretch()
            vl = make_label(f"{val}{unit}", C["t1"], 14, bold=True)
            lh.addWidget(vl)

            s = QSlider(Qt.Horizontal)
            s.setRange(mn, mx); s.setValue(val)
            s.setStyleSheet(slider_style(color))
            s.valueChanged.connect(lambda v, u=unit, l=vl: l.setText(f"{v}{u}"))

            lims2 = QHBoxLayout()
            lims2.addWidget(make_label(str(mn), C["t3"], 8, mono=True))
            lims2.addStretch()
            lims2.addWidget(make_label(str(mx), C["t3"], 8, mono=True))

            pnl.body.addLayout(lh)
            pnl.body.addWidget(s)
            pnl.body.addLayout(lims2)
        return pnl

    def _make_tasks_panel(self):
        pnl = Panel("PRESET TASKS", "WEBOTS")
        for no, nm, st_ in [
            ("01", "Home Position",      "rdy"),
            ("02", "Pick & Place A→B",   "idle"),
            ("05", "Custom Trajectory",  "idle"),
        ]:
            f = QFrame()
            f.setStyleSheet(
                f"QFrame{{background:rgba(0,0,0,0.25);"
                f"border:1px solid {C['b']};border-radius:5px;}}"
                f"QFrame:hover{{border-color:{C['bb']};}}"
            )
            f.setCursor(Qt.PointingHandCursor)
            ly2 = QHBoxLayout(f); ly2.setContentsMargins(12, 10, 12, 10)
            ly2.addWidget(make_label(no, C["t3"], 10, mono=True))
            ly2.addWidget(make_label(nm, C["t1"], 12))
            ly2.addStretch()

            sc, bg = {
                "rdy":  (C["a2"], "rgba(0,255,157,0.07)"),
                "done": (C["a"],  "rgba(0,212,170,0.07)"),
                "idle": (C["t3"], "rgba(255,255,255,0.02)"),
            }.get(st_, (C["t3"], "transparent"))

            sl2 = QLabel(st_.upper())
            sl2.setStyleSheet(
                f"color:{sc};background:{bg};border:1px solid {sc}40;"
                f"border-radius:10px;font-size:9px;padding:3px 8px;"
                f"font-family:'Share Tech Mono';"
            )
            ly2.addWidget(sl2)
            pnl.body.addWidget(f)
        return pnl

    # ── Slider callback ───────────────────────────────────────────────────────
    def _on_slider(self, i: int, val: float, lbl):
        self._joints[i]["v"] = val
        lbl.setText(f"{val:.1f}°")
        self._ros.publish_joints([j["v"] for j in self._joints])
        
    def _on_slider_pressed(self, idx: int):
        """Kullanıcı slider'a tıkladığında ROS güncellemelerini durdurur."""
        self._slider_active[idx] = True

    def _on_slider_released(self, idx: int):
        """Kullanıcı slider'ı bıraktığında ROS güncellemelerini tekrar açar."""
        self._slider_active[idx] = False
    

    # ── ROS güncellemesi (slider'ları güncelle) ───────────────────────────────
    def update_canvas_from_ros(self, angles_rad: list):
        for i, (s, lbl) in enumerate(self._sliders):
            if i < len(angles_rad):
                if hasattr(self, '_slider_active') and self._slider_active[i]:
                    continue
                deg = math.degrees(angles_rad[i])
                s.blockSignals(True); s.setValue(int(deg * 10)); s.blockSignals(False)
                lbl.setText(f"{deg:.1f}°")

    # ── Mod seçimi ────────────────────────────────────────────────────────────
    def _mode_style(self) -> str:
        return (
            f"QPushButton{{background:transparent;border:1px solid {C['b']};"
            f"border-radius:4px;color:{C['t2']};font-family:'Share Tech Mono';"
            f"font-size:11px;letter-spacing:.12em;padding:5px 10px;}}"
            f"QPushButton:checked{{background:rgba(0,212,170,0.09);"
            f"border-color:{C['a']};color:{C['a']};}}"
            f"QPushButton:hover{{border-color:{C['a']};color:{C['a']};}}"
        )

    def _set_mode(self, active_btn, mode: str):
        for b in self._mode_btns:
            b.setChecked(b is active_btn)
        self._ros.publish_cmd(f"MODE:{mode}")

    # ── Komut gönderme ────────────────────────────────────────────────────────
    def _cmd(self, c: str):
        self._ros.publish_cmd(c)

    # ── Joystick (ileride kullanılabilir) ─────────────────────────────────────
    def _joy_stop(self):
        self._jx = self._jy = self._jz = self._jwz = 0.0
        self._joy_timer.stop()
        self._ros.publish_twist(0, 0, 0, 0)

    def _pub_twist(self):
        self._ros.publish_twist(
            self._jx * 0.25, self._jy * 0.25,
            self._jz * 0.25, self._jwz * 0.25,
        )