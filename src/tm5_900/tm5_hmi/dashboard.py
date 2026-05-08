# ─────────────────────────────────────────────────────────────────────────────
#  dashboard.py  —  Dashboard sayfası (3 sütunlu düzen)
#  Sol: Webots 3D Viewer  |  Orta: İstatistik + TCP + Eklemler
#  Sağ: Program Loader + Diagnostics + Log
# ─────────────────────────────────────────────────────────────────────────────

import math
import random
import json
import time

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

        # Only keep the 3 requested cards
        self._sc_state = StatCard("ROBOT STATE",    "STANDBY","READY FOR COMMAND","g")
        self._sc_prog  = StatCard("ACTIVE PROGRAM", "NONE",   "NO FILE LOADED",  "b")
        self._sc_up    = StatCard("UPTIME",         "00:00",  "SESSION TIME",    "g")

        # Put them all in a single row
        for i, card in enumerate([self._sc_state, self._sc_prog, self._sc_up]):
            g.addWidget(card, 0, i)

        pnl.body.addLayout(g)
        return pnl
    # ── TCP Kartezyen pozisyon hücreleri ──────────────────────────────────────
    def _make_tcp(self):
        pnl = Panel("TCP CARTESIAN POSITION", "WORLD FRAME")
        
        # --- NEW CODE: Make the panel taller and space the cells out ---
        pnl.setMinimumHeight(170) 
        grid = QGridLayout(); grid.setSpacing(12) 
        # ---------------------------------------------------------------
        
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

        load_btn = QPushButton("BROWSE FILE")
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

        # Log
        log_pnl = Panel("SYSTEM LOG", "RUNTIME")
        self._log = LogWidget()
        log_pnl.body.addWidget(self._log)
        ly.addWidget(log_pnl, 1)   # kalan yüksekliği kapla

        return w

    # ── Program Loader işlemleri ──────────────────────────────────────────────
    _prog_file  = None
    _prog_timer = None
    _loaded_trajectory = []
    _run_idx = 0

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Program", "",
            "Robot Programs (*.json *.script *.txt *.py *.xml);;All Files (*)"
        )
        if path:
            filename = path.split('/')[-1]
            self._prog_file = path
            self._file_lbl.setText(f"📄 {filename}")
            self._pbar_lbl.setText("LOADED")
            
            # --- YENİ EKLENEN: State ve Active Program Kartını Güncelle ---
            self._sc_state.set_value("PROGRAM READY", "#00d2d3") # Cyan (Turkuaz) Renk
            self._sc_prog.set_value(filename[:15], "#00d2d3")
            # --------------------------------------------------------------
            
            self.log(f"Loaded: {filename}", "ok")

    def _run_prog(self):
        if not self._prog_file:
            self.log("No program loaded", "err")
            return
            
        # JSON Dosyasını Oku ve Ayrıştır
        try:
            with open(self._prog_file, "r") as f:
                data = json.load(f)
        except Exception as e:
            self.log(f"File Error (Not a valid JSON): {e}", "err")
            return
            
        # Format Kontrolü: Dosyadaki yörünge noktalarını bul
        self._loaded_trajectory = []
        
        if isinstance(data, list):
            # Dosya sadece düz bir liste ise
            self._loaded_trajectory = data
        elif isinstance(data, dict):
            # Dosya Teach Mode gibi sözlük (dict) formatındaysa, ilk dolu programı al
            for key, points in data.items():
                if isinstance(points, list) and len(points) > 0:
                    self._loaded_trajectory = points
                    self.log(f"Found '{key}' in file.", "info")
                    break

        # Dosya boşsa durdur
        if not self._loaded_trajectory:
            self.log("No valid trajectory points found in file!", "err")
            return

        # Başlangıç Ayarları
        self._run_idx = 0
        self._pbar_lbl.setText("RUNNING")
        self._pbar.setValue(0)
        self._sc_state.set_value("RUNNING", C["a2"])
        
        count = len(self._loaded_trajectory)
        self.log(f"PROGRAM STARTED: Executing {count} points...", "ok")

        # Timer'ı oluştur ve ilk noktayı MoveIt'e gönder
        self._prog_timer = QTimer(self)
        self._prog_timer.timeout.connect(self._tick_prog)
        
        self._tick_prog() # İlk hareketi beklemeden hemen başlat
        self._prog_timer.start(4000) # Diğer hedefler için 4 saniye bekle

    def _tick_prog(self):
        # Program bitti mi?
        if self._run_idx >= len(self._loaded_trajectory):
            self._prog_timer.stop()
            self._pbar.setValue(100)
            self._pbar_lbl.setText("COMPLETE")
            self._sc_state.set_value("DONE", C["a"])
            self.log("PROGRAM COMPLETE!", "ok")
            return

        # Sıradaki noktayı al ve radyana çevirip MoveIt'e gönder
        pt = self._loaded_trajectory[self._run_idx]
        
        if "joints" in pt:
            angles_deg = pt["joints"]
            angles_rad = [math.radians(a) for a in angles_deg]
            self.log(f"Executing Point {self._run_idx + 1} of {len(self._loaded_trajectory)}...", "warn")
            self._ros.send_moveit_goal(angles_rad)
            
        # --- YENİ EKLENEN: Gripper varsa çalıştır ---
        if "gripper" in pt:
            action = pt["gripper"]
            self.log(f"Gripper '{action}' command found in point {self._run_idx + 1}.", "info")
            if "joints" in pt:
                # Kolun hedefe gitmesi için 3.5 saniye bekle
                QTimer.singleShot(3500, lambda a=action: self._ros.set_gripper(a))
            else:
                # Sadece gripper komutuysa hemen çalıştır
                self._ros.set_gripper(action)
        # ---------------------------------------------
        elif "joints" not in pt and "gripper" not in pt:
            self.log(f"Point {self._run_idx + 1} is corrupted, skipping.", "err")

        # Progress bar'ı gerçek yüzdelik dilime göre güncelle
        pct = int((self._run_idx / len(self._loaded_trajectory)) * 100)
        self._pbar.setValue(pct)

        self._run_idx += 1

    def _stop_prog(self):
        if self._prog_timer and self._prog_timer.isActive():
            self._prog_timer.stop()
            
        self._pbar.setValue(0)
        self._run_idx = 0
        self._loaded_trajectory = []
        
        self._pbar_lbl.setText("STOPPED")
        self._sc_state.set_value("STOPPED", C["red"])
        self.log("Program stopped by user", "err")
        self._ros.publish_cmd("STOP")

    # ── Veri güncelleme ───────────────────────────────────────────────────────
#================================================================    
#this added
    def update_tcp(self, x, y, z, rx, ry, rz):
        self._tcp["X"].set_value(x)
        self._tcp["Y"].set_value(y)
        self._tcp["Z"].set_value(z, 0, 1400)

        self._tcp["RX"].set_value(rx, -180, 180)
        self._tcp["RY"].set_value(ry, -180, 180)
        self._tcp["RZ"].set_value(rz, -180, 180)
#================================================================
    def update_from_ros(self, pos: list, vel: list):
        """Gerçek ROS verileriyle eklem kartlarını güncelle."""
        for i, card in enumerate(self._jcards):
            if i >= len(pos): break
            deg = math.degrees(pos[i])
            self._joints[i]["v"] = deg
            card.update_data(
                deg,
                math.degrees(vel[i]) if i < len(vel) else 0.0,
                  # eff is not used in the updated method
            )

    # def update_demo(self, t: float):
    #     """Demo animasyonu (ROS yokken)."""
    #     for i, j in enumerate(self._joints):
    #         j["v"] = max(j["mn"], min(j["mx"], j["v"] + (random.random() - 0.5) * 0.3))
    #         self._jcards[i].update_data(
    #             j["v"],
    #             (random.random() - 0.5) * 2,
    #         )
        #x = math.sin(t / 3) * 300; y = math.cos(t / 4) * 200; z = 650 + math.sin(t / 5) * 80
        # self._tcp["X"].set_value(x); self._tcp["Y"].set_value(y)
        # self._tcp["Z"].set_value(z, 0, 1400)
        # self._tcp["RX"].set_value(math.sin(t / 6) * 30, -180, 180)
        # self._tcp["RY"].set_value(math.cos(t / 7) * 20, -180, 180)
        # self._tcp["RZ"].set_value(math.sin(t / 8) * 45, -180, 180)

    def log(self, msg: str, kind: str = "info") -> int:
        return self._log.add(msg, kind)