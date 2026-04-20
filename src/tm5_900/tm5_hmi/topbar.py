# ─────────────────────────────────────────────────────────────────────────────
#  topbar.py  —  Üst navigasyon barı
#  DASHBOARD / CONTROL  |  ROS durum pill'i  |  Saat  |  E-STOP
# ─────────────────────────────────────────────────────────────────────────────

from datetime import datetime

from PyQt5.QtWidgets import QFrame, QHBoxLayout, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtCore    import Qt, pyqtSignal

from config  import C
from widgets import make_label


class TopBar(QFrame):
    nav_changed   = pyqtSignal(str)   # "dash" | "ctrl"
    estop_pressed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(56)
        self.setStyleSheet(
            f"TopBar{{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            f"stop:0 #0b1926,stop:1 {C['bg']});border-bottom:1px solid {C['b']};}}"
        )
        main = QHBoxLayout(self)
        main.setContentsMargins(18, 0, 18, 0)

        # ── Logo + proje adı ──────────────────────────────────────────────
        logo = QLabel("TM")
        logo.setFixedSize(32, 32)
        logo.setAlignment(Qt.AlignCenter)
        logo.setStyleSheet(
            f"color:{C['a']};border:2px solid {C['a']};border-radius:5px;"
            f"font-family:'Rajdhani';font-size:14px;font-weight:700;background:transparent;"
        )

        name = QLabel(f"TM5-900 <span style='color:{C['a']}'> CONTROL</span>")
        name.setTextFormat(Qt.RichText)
        name.setStyleSheet(
            f"font-family:'Rajdhani';font-size:20px;font-weight:700;"
            f"color:{C['t1']};background:transparent;"
        )
        sub = QLabel(
            "6-DOF COBOT (3D PRINTED CHASSIS) · HMI v2.0  |  DEV: HAMZA & AHMET"
        )
        sub.setStyleSheet(
            f"font-family:'Share Tech Mono';font-size:9px;color:{C['t3']};"
            f"letter-spacing:.18em;background:transparent;"
        )
        info_col = QVBoxLayout(); info_col.setSpacing(0)
        info_col.addWidget(name); info_col.addWidget(sub)
        main.addWidget(logo); main.addLayout(info_col); main.addStretch()

        # ── Navigasyon butonları ──────────────────────────────────────────
        self._btn_dash = QPushButton("DASHBOARD")
        self._btn_ctrl = QPushButton("CONTROL")
        for btn, pg in [(self._btn_dash, "dash"), (self._btn_ctrl, "ctrl")]:
            btn.setFixedHeight(30)
            btn.clicked.connect(lambda _, p=pg: self._nav(p))
            main.addWidget(btn)
        self._active = "dash"
        self._update_nav()
        main.addSpacing(12)

        # ── ROS durum pill ────────────────────────────────────────────────
        self.pill = QLabel("● CONNECTING")
        self.pill.setStyleSheet(
            f"background:rgba(255,61,90,0.07);border:1px solid rgba(255,61,90,0.22);"
            f"border-radius:12px;color:{C['red']};font-family:'Share Tech Mono';"
            f"font-size:10px;padding:4px 12px;"
        )
        main.addWidget(self.pill)

        # ── Saat ──────────────────────────────────────────────────────────
        self.clock = QLabel("--:--:--")
        self.clock.setStyleSheet(
            f"font-family:'Share Tech Mono';font-size:12px;color:{C['t2']};"
            f"min-width:70px;background:transparent;"
        )
        main.addWidget(self.clock)

        # ── E-STOP ────────────────────────────────────────────────────────
        es = QPushButton("⬛ E-STOP")
        es.setStyleSheet(
            f"QPushButton{{background:rgba(255,61,90,0.1);border:2px solid {C['red']};"
            f"border-radius:4px;color:{C['red']};font-family:'Rajdhani';font-size:12px;"
            f"font-weight:700;letter-spacing:.15em;padding:6px 15px;}}"
            f"QPushButton:hover{{background:rgba(255,61,90,0.22);}}"
            f"QPushButton:pressed{{background:{C['red']};color:#fff;}}"
        )
        es.clicked.connect(self.estop_pressed)
        main.addWidget(es)

    # ── Nav buton stili ────────────────────────────────────────────────────────
    def _nav_style(self, active: bool = False) -> str:
        if active:
            return (
                f"QPushButton{{padding:6px 18px;"
                f"background:rgba(0,212,170,0.09);"
                f"border:1px solid {C['a']};border-radius:4px;color:{C['a']};"
                f"font-family:'Rajdhani';font-size:12px;font-weight:600;"
                f"letter-spacing:.1em;}}"
            )
        return (
            f"QPushButton{{padding:6px 18px;background:transparent;"
            f"border:1px solid {C['b']};border-radius:4px;color:{C['t2']};"
            f"font-family:'Rajdhani';font-size:12px;font-weight:600;"
            f"letter-spacing:.1em;}}"
            f"QPushButton:hover{{border-color:{C['a']};color:{C['a']};}}"
        )

    def _update_nav(self):
        self._btn_dash.setStyleSheet(self._nav_style(self._active == "dash"))
        self._btn_ctrl.setStyleSheet(self._nav_style(self._active == "ctrl"))

    def _nav(self, pg: str):
        self._active = pg
        self._update_nav()
        self.nav_changed.emit(pg)

    # ── Public metodlar ───────────────────────────────────────────────────────
    def set_ros_status(self, online: bool):
        if online:
            self.pill.setText("● ROS 2 ONLINE")
            self.pill.setStyleSheet(
                f"background:rgba(0,230,118,0.07);"
                f"border:1px solid rgba(0,230,118,0.22);"
                f"border-radius:12px;color:{C['ok']};"
                f"font-family:'Share Tech Mono';font-size:10px;padding:4px 12px;"
            )
        else:
            self.pill.setText("● OFFLINE")
            self.pill.setStyleSheet(
                f"background:rgba(255,61,90,0.07);"
                f"border:1px solid rgba(255,61,90,0.22);"
                f"border-radius:12px;color:{C['red']};"
                f"font-family:'Share Tech Mono';font-size:10px;padding:4px 12px;"
            )

    def tick(self):
        self.clock.setText(datetime.now().strftime("%H:%M:%S"))