# ─────────────────────────────────────────────────────────────────────────────
#  dashboard.py  —  Dashboard sayfası (3 sütunlu düzen)
#  Sol: Webots 3D Viewer  |  Orta: İstatistik + TCP + Eklemler
#  Sağ: Program Loader + Diagnostics + Log
# ─────────────────────────────────────────────────────────────────────────────

import math
import random

from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QPushButton, QProgressBar, QFileDialog,
)
from PyQt5.QtCore import Qt, QTimer

from config     import C
from widgets    import (Panel, StatCard, JointCard, TcpCell, LogWidget, make_label)
from viewer     import ViewerFrame
from ros_worker import RosWorker


class DashPage(QWidget):
    """
    Dashboard sayfası.
    ROS bağlıysa gerçek eklem verilerini gösterir;
    bağlı değilse demo animasyonu oynatır.
    """

    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent)
        self._ros    = ros
        self._joints = joints
        self._build_ui()

    # ── Ana düzen ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        ml = QHBoxLayout(self)
        ml.setContentsMargins(15, 15, 15, 15)
        ml.setSpacing(15)

        # Sol: 3D Viewer (2 birim)
        self._viewer_frame = ViewerFrame()
        ml.addWidget(self._viewer_frame, 2)

        # Orta: İstatistikler + TCP + Eklemler (1 birim)
        mid = QVBoxLayout(); mid.setSpacing(15)
        mid.addWidget(self._make_stats())
        mid.addWidget(self._make_tcp())
        mid.addWidget(self._make_joints())
        mid.addStretch()
        ml.addLayout(mid, 1)

        # Sağ: Araçlar (1 birim)
        right = QVBoxLayout(); right.setSpacing(15)
        right.addWidget(self._make_right_tools())
        ml.addLayout(right, 1)

    # ── Sistem genel bakış kartları ───────────────────────────────────────────
    def _make_stats(self):
        pnl = Panel("SYSTEM OVERVIEW", "REAL-TIME")
        g = QGridLayout(); g.setSpacing(8)

        self._sc_state = StatCard("ROBOT STATE",    "STANDBY","READY FOR COMMAND","g")
        self._sc_spd   = StatCard("TCP SPEED",      "0 mm/s", "MAX 2500 mm/s",   "b")
        self._sc_prog  = StatCard("ACTIVE PROGRAM", "NONE",   "NO FILE LOADED",  "b")
        self._sc_reach = StatCard("REACH",          "900 mm", "TM5-900 MAX",     "o")
        self._sc_up    = StatCard("UPTIME",         "00:00",  "SESSION TIME",    "g")
        self._sc_err   = StatCard("ERRORS",         "0",      "ACTIVE FAULTS",   "r")

        for i, card in enumerate([
            self._sc_state, self._sc_spd, self._sc_prog,
            self._sc_reach, self._sc_up, self._sc_err,
        ]):
            g.addWidget(card, i // 3, i % 3)

        pnl.body.addLayout(g)
        return pnl

    # ── TCP Kartezyen pozisyon hücreleri ──────────────────────────────────────
    def _make_tcp(self):
        pnl = Panel("TCP CARTESIAN POSITION", "WORLD FRAME")
        grid = QGridLayout(); grid.setSpacing(8)
        self._tcp = {
            "X":  TcpCell("X AXIS", "#ff4d6d"),
            "Y":  TcpCell("Y AXIS", C["a2"]),
            "Z":  TcpCell("Z AXIS", C["a"]),
            "RX": TcpCell("RX",     C["a3"], "°"),
            "RY": TcpCell("RY",     C["a3"], "°"),
            "RZ": TcpCell("RZ",     C["a3"], "°"),
        }
        for i, (_, w) in enumerate(self._tcp.items()):
            grid.addWidget(w, i // 3, i % 3)
        pnl.body.addLayout(grid)
        return pnl

    # ── Eklem durum kartları ──────────────────────────────────────────────────
    def _make_joints(self):
        pnl = Panel("JOINT STATES", "6-DOF KINEMATICS")
        g = QGridLayout(); g.setSpacing(8)
        self._jcards = []
        for i, j in enumerate(self._joints):
            card = JointCard(j["id"], j["ros"], j["nm"], j["mn"], j["mx"])
            g.addWidget(card, i // 3, i % 3)
            self._jcards.append(card)
        pnl.body.addLayout(g)
        return pnl

    # ── Sağ sütun: Program Loader + Diagnostics + Log ─────────────────────────
    def _make_right_tools(self):
        w = QWidget(); w.setStyleSheet("background:transparent;")
        ly = QVBoxLayout(w); ly.setContentsMargins(0, 0, 0, 0); ly.setSpacing(15)

        # Program Loader
        prog = Panel("PROGRAM LOADER", "NO FILE")
        self._file_lbl = make_label("Drop or click to load", C["t2"], 10, mono=True)
        self._file_lbl.setAlignment(Qt.AlignCenter)
        self._file_lbl.setWordWrap(True)

        load_btn = QPushButton("📁  BROWSE FILE")
        load_btn.setStyleSheet(
            f"QPushButton{{background:transparent;border:2px dashed {C['bb']};"
            f"border-radius:5px;color:{C['t2']};font-family:'Share Tech Mono';"
            f"font-size:10px;padding:8px;letter-spacing:.12em;}}"
            f"QPushButton:hover{{border-color:{C['a']};color:{C['a']};"
            f"background:rgba(0,212,170,0.04);}}"
        )
        load_btn.clicked.connect(self._browse)

        self._pbar = QProgressBar()
        self._pbar.setRange(0, 100); self._pbar.setValue(0)
        self._pbar.setTextVisible(False); self._pbar.setFixedHeight(6)
        self._pbar.setStyleSheet(
            f"QProgressBar{{background:rgba(255,255,255,0.04);border-radius:3px;border:none;}}"
            f"QProgressBar::chunk{{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            f"stop:0 {C['a2']},stop:1 {C['a']});border-radius:3px;}}"
        )
        self._pbar_lbl = make_label("READY", C["t3"], 8, mono=True)

        run_btn = QPushButton("▶  START")
        run_btn.setStyleSheet(
            f"QPushButton{{background:rgba(0,255,157,0.1);border:2px solid {C['a2']};"
            f"border-radius:6px;color:{C['a2']};font-family:'Rajdhani';font-size:12px;"
            f"font-weight:700;letter-spacing:.12em;padding:8px;}}"
            f"QPushButton:hover{{background:rgba(0,255,157,0.2);}}"
        )
        stop_btn = QPushButton("⏹  STOP")
        stop_btn.setStyleSheet(
            f"QPushButton{{background:rgba(255,61,90,0.1);border:2px solid {C['red']};"
            f"border-radius:6px;color:{C['red']};font-family:'Rajdhani';font-size:12px;"
            f"font-weight:700;letter-spacing:.12em;padding:8px;}}"
            f"QPushButton:hover{{background:rgba(255,61,90,0.2);}}"
        )
        run_btn.clicked.connect(self._run_prog)
        stop_btn.clicked.connect(self._stop_prog)

        from PyQt5.QtWidgets import QHBoxLayout as _HL
        btns = _HL(); btns.addWidget(run_btn); btns.addWidget(stop_btn)

        prog.body.addWidget(load_btn); prog.body.addWidget(self._file_lbl)
        prog.body.addWidget(self._pbar_lbl); prog.body.addWidget(self._pbar)
        prog.body.addLayout(btns)
        ly.addWidget(prog)

        # Diagnostics
        diag = Panel("DIAGNOSTICS", "HARDWARE")
        self._diag = {}
        for key, label, color in [
            ("temp", "JOINT TEMP AVG", C["a"]),
            ("pwr",  "POWER DRAW",     C["a2"]),
            ("lat",  "ROS LATENCY",    C["a"]),
        ]:
            from PyQt5.QtWidgets import QHBoxLayout as _HL2
            row = _HL2()
            lbl = make_label(label, C["t3"], 8, mono=True); lbl.setFixedWidth(90)
            bar = QProgressBar(); bar.setRange(0, 100); bar.setValue(40)
            bar.setTextVisible(False); bar.setFixedHeight(4)
            bar.setStyleSheet(
                f"QProgressBar{{background:rgba(255,255,255,0.04);"
                f"border-radius:2px;border:none;}}"
                f"QProgressBar::chunk{{background:{color};border-radius:2px;}}"
            )
            val = make_label("--", C["t1"], 11, bold=True)
            val.setFixedWidth(40)
            from PyQt5.QtCore import Qt as _Qt
            val.setAlignment(_Qt.AlignRight)
            row.addWidget(lbl); row.addWidget(bar); row.addWidget(val)
            self._diag[key] = (bar, val)
            diag.body.addLayout(row)
        ly.addWidget(diag)

        # Log
        log_pnl = Panel("SYSTEM LOG", "RUNTIME")
        self._log = LogWidget()
        log_pnl.body.addWidget(self._log)
        ly.addWidget(log_pnl, 1)   # kalan yüksekliği kapla

        return w

    # ── Program Loader işlemleri ──────────────────────────────────────────────
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
        self._prog_pct = 0.0; self._pbar_lbl.setText("RUNNING")
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
        if self._prog_timer:
            self._prog_timer.stop()
        self._pbar.setValue(0); self._prog_pct = 0.0
        self._pbar_lbl.setText("STOPPED")
        self._sc_state.set_value("STOPPED", C["red"])
        self.log("Program stopped", "warn")
        self._ros.publish_cmd("STOP")

    # ── Veri güncelleme ───────────────────────────────────────────────────────
    def update_from_ros(self, pos: list, vel: list, eff: list):
        """Gerçek ROS verileriyle eklem kartlarını güncelle."""
        for i, card in enumerate(self._jcards):
            if i >= len(pos): break
            deg = math.degrees(pos[i])
            self._joints[i]["v"] = deg
            card.update_data(
                deg,
                math.degrees(vel[i]) if i < len(vel) else 0.0,
                eff[i] if i < len(eff) else 0.0,
            )

    def update_demo(self, t: float):
        """Demo animasyonu (ROS yokken)."""
        for i, j in enumerate(self._joints):
            j["v"] = max(j["mn"], min(j["mx"], j["v"] + (random.random() - 0.5) * 0.3))
            self._jcards[i].update_data(
                j["v"],
                (random.random() - 0.5) * 2,
                (random.random() - 0.5) * 5,
            )
        x = math.sin(t / 3) * 300; y = math.cos(t / 4) * 200; z = 650 + math.sin(t / 5) * 80
        self._tcp["X"].set_value(x); self._tcp["Y"].set_value(y)
        self._tcp["Z"].set_value(z, 0, 1400)
        self._tcp["RX"].set_value(math.sin(t / 6) * 30, -180, 180)
        self._tcp["RY"].set_value(math.cos(t / 7) * 20, -180, 180)
        self._tcp["RZ"].set_value(math.sin(t / 8) * 45, -180, 180)
        self._sc_spd.set_value(f"{abs(math.sin(t / 2) * 100):.0f} mm/s")

        tmp = 38 + random.random() * 8
        pwr = 80 + random.random() * 60
        lat = 5  + random.random() * 15
        self._diag["temp"][0].setValue(int(tmp))
        self._diag["temp"][1].setText(f"{tmp:.0f}°C")
        self._diag["pwr"][0].setValue(int(pwr / 300 * 100))
        self._diag["pwr"][1].setText(f"{pwr:.0f}W")
        self._diag["lat"][0].setValue(int(lat))
        self._diag["lat"][1].setText(f"{lat:.0f}ms")

    def log(self, msg: str, kind: str = "info") -> int:
        return self._log.add(msg, kind)