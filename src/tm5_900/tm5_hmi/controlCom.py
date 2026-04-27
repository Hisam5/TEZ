# ─────────────────────────────────────────────────────────────────────────────
#  control.py  —  Kontrol sayfası (2 sütunlu düzen)
#  Sol: Operasyon modu + Manuel eklem slider'ları
#  Sağ: Hızlı komutlar + Hız/Tork limitleri + Hazır görevler
# ─────────────────────────────────────────────────────────────────────────────
import math
import subprocess
import json  # <--- YENİ
import os    # <--- YENİ
from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QPushButton, QSlider, QFrame, QLabel, QTabWidget,
    QDoubleSpinBox, QSpinBox
)
from PyQt5.QtGui import QWindow
from PyQt5.QtCore import Qt, QTimer
from config     import C
from widgets    import Panel, CmdButton, make_label, slider_style
from ros_worker import RosWorker

class CtrlPage(QWidget):
    def __init__(self, ros: RosWorker, joints: list, parent=None):
        super().__init__(parent)
        self._ros    = ros
        self._joints = joints
        self._jx = self._jy = self._jz = self._jwz = 0.0
        self._joy_timer = QTimer(self)
        self._joy_timer.timeout.connect(self._pub_twist)
        # --- Teach Mode Hafızası ---
        # Arayüz, oluşturduğunuz "programs" klasörünü doğrudan hedef alacak
        self._save_dir = "programs"  
        self._teach_file = os.path.join(self._save_dir, "teach_programs.json")
        self._teach_data = self._load_teach_data()
        
        # --- Program Oynatma Zamanlayıcısı ---
        self._run_idx = 0
        self._run_prog_name = ""
        self._run_timer = QTimer(self)
        self._run_timer.timeout.connect(self._tick_run_prog)
        self._build_ui()

    def _build_ui(self):
        ml = QHBoxLayout(self)
        ml.setContentsMargins(15, 15, 15, 15) 
        ml.setSpacing(15)
        ml.addLayout(self._make_left_col(), 1)
        ml.addLayout(self._make_right_col(), 1)

    def _make_left_col(self):
        col = QVBoxLayout(); col.setSpacing(15)
        col.addWidget(self._make_cmd_panel())
        col.addWidget(self._make_rviz_panel(), 1)
        return col

    def _make_mode_panel(self):
        pnl = Panel("TEACH MODE")
        
        # Üst Kısım: Aksiyon Butonları (2x2 Grid)
        mg = QGridLayout(); mg.setSpacing(6)
        
        self._btn_activate = QPushButton("INACTIVE")
        self._btn_activate.setCheckable(True) 
        self._btn_activate.setStyleSheet(self._mode_style())
        self._btn_activate.setMinimumHeight(35)
        self._btn_activate.toggled.connect(self._on_activate_toggled)
        mg.addWidget(self._btn_activate, 0, 0) 
        
        btn_point = QPushButton("📍 RECORD POINT")
        btn_point.setStyleSheet(self._mode_style())
        btn_point.setMinimumHeight(35)
        btn_point.clicked.connect(self._record_point)
        mg.addWidget(btn_point, 0, 1) 

        btn_save = QPushButton("💾 SAVE PROG")
        btn_save.setStyleSheet(self._mode_style())
        btn_save.setMinimumHeight(35)
        btn_save.clicked.connect(self._save_active_prog)
        mg.addWidget(btn_save, 1, 0)
        
        btn_delete = QPushButton("🗑 DELETE PROG")
        btn_delete.setStyleSheet(self._mode_style())
        btn_delete.setMinimumHeight(35)
        btn_delete.clicked.connect(self._delete_active_prog)
        mg.addWidget(btn_delete, 1, 1) 

        # Alt Kısım: Program Seçiciler (Yatay)
        prog_ly = QHBoxLayout(); prog_ly.setSpacing(6)
        self._prog_btns = []
        for i, prog_name in enumerate(["PROG1", "PROG2", "PROG3"]):
            b = QPushButton(prog_name)
            b.setCheckable(True)
            b.setChecked(i == 0)  
            b.setStyleSheet(self._mode_style())
            b.setMinimumHeight(35)
            b.clicked.connect(lambda _, btn=b: self._set_prog(btn))
            self._prog_btns.append(b)
            prog_ly.addWidget(b)
            
        pnl.body.addLayout(mg)
        pnl.body.addLayout(prog_ly)
        return pnl

    # ── TEACH MODE Hafıza & Dosya İşlemleri ───────────────────────────────────
    def _load_teach_data(self):
        """Kayıtlı programları JSON dosyasından okur."""
        if os.path.exists(self._teach_file):
            try:
                with open(self._teach_file, "r") as f:
                    return json.load(f)
            except Exception as e:
                print(f"[HATA] Teach dosyası okunamadı: {e}")
        # Dosya yoksa veya bozuksa boş şablon döndür
        return {"PROG1": [], "PROG2": [], "PROG3": []}

    def _save_teach_data(self):
        """Programları JSON dosyasına kaydeder."""
        with open(self._teach_file, "w") as f:
            json.dump(self._teach_data, f, indent=4)

    # ── TEACH MODE Aksiyonları ────────────────────────────────────────────────
    def _on_activate_toggled(self, checked):
        if checked:
            self._btn_activate.setText("ACTIVE")
            self._ros.log_message.emit("Teach Mode: ACTIVE. Ready to record.", "warn")
        else:
            self._btn_activate.setText("INACTIVE")
            self._ros.log_message.emit("Teach Mode: INACTIVE.", "info")

    def _set_prog(self, active_btn):
        for b in self._prog_btns:
            b.setChecked(b is active_btn)
        
        # Hangi programa geçtiysek loga yazdır ve kaç nokta olduğunu göster
        prog_name = active_btn.text()
        count = len(self._teach_data[prog_name])
        self._ros.log_message.emit(f"Switched to {prog_name} ({count} points loaded).", "info")

    def _get_active_prog(self) -> str:
        """Şu an hangi program sekmesinin seçili olduğunu bulur."""
        for b in self._prog_btns:
            if b.isChecked():
                return b.text()
        return "PROG1"

    def _record_point(self):
        """Mevcut Joint ve TCP değerlerini okur ve seçili programa ekler."""
        if not self._btn_activate.isChecked():
            self._ros.log_message.emit("Cannot record! Teach Mode is INACTIVE.", "err")
            return
            
        prog = self._get_active_prog()
        
        # Joint Slider Değerlerini Oku
        j_vals = [s.value() / 10.0 for s, _sb in self._sliders]
        
        # TCP Slider Değerlerini Oku
        tcp_vals = {
            "X": self._tcp_sliders["X"][0].value() / 10.0,
            "Y": self._tcp_sliders["Y"][0].value() / 10.0,
            "Z": self._tcp_sliders["Z"][0].value() / 10.0,
            "RX": self._tcp_sliders["RX"][0].value() / 10.0,
            "RY": self._tcp_sliders["RY"][0].value() / 10.0,
            "RZ": self._tcp_sliders["RZ"][0].value() / 10.0
        }
        
        # Hafızaya ekle
        point_data = {"joints": j_vals, "tcp": tcp_vals}
        self._teach_data[prog].append(point_data)
        
        count = len(self._teach_data[prog])
        self._ros.log_message.emit(f"Point {count} added to {prog}.", "ok")

    def _save_active_prog(self):
        """Değişiklikleri diske yazar."""
        self._save_teach_data()
        prog = self._get_active_prog()
        count = len(self._teach_data[prog])
        self._ros.log_message.emit(f"SUCCESS: {prog} saved with {count} points!", "ok")

    def _delete_active_prog(self):
        """Seçili programın içini tamamen boşaltır ve siler."""
        prog = self._get_active_prog()
        self._teach_data[prog] = []
        self._save_teach_data()
        self._ros.log_message.emit(f"DELETED: All points in {prog} cleared.", "err")
    def _start_teach_program(self):
        """Seçili Teach Mode programını başlatır."""
        # YENİ EKLENEN: Başlamadan hemen önce programs klasöründeki dosyayı oku
        self._teach_data = self._load_teach_data()
        
        prog = self._get_active_prog()
        
        # Program boş mu kontrol et
        if not self._teach_data[prog]:
            self._ros.log_message.emit(f"Cannot start: {prog} is empty!", "err")
            return
            
        self._run_prog_name = prog
        self._run_idx = 0
        count = len(self._teach_data[prog])
        self._ros.log_message.emit(f"PLAYBACK: {prog} started ({count} points)...", "info")
        
        # İlk noktaya hemen git
        self._tick_run_prog()
        
        # Diğer noktalar için zamanlayıcıyı başlat (Her nokta arası 4 saniye)
        self._run_timer.start(4000)

    def _tick_run_prog(self):
        """Timer ile sıradaki kayıtlı noktaya MoveIt komutu gönderir."""
        pts = self._teach_data[self._run_prog_name]
        
        # Tüm noktalar bittiyse durdur
        if self._run_idx >= len(pts):
            self._run_timer.stop()
            self._ros.log_message.emit(f"PLAYBACK: {self._run_prog_name} COMPLETE.", "ok")
            return
            
        # Sıradaki noktayı al ve radyana çevirip MoveIt'e yolla
        pt = pts[self._run_idx]
        angles_deg = pt["joints"]
        angles_rad = [math.radians(a) for a in angles_deg]
        
        self._ros.log_message.emit(f"Executing Point {self._run_idx + 1} of {len(pts)}...", "warn")
        self._ros.send_moveit_goal(angles_rad)
        
        self._run_idx += 1
    # ── YENİ: Manuel Input Kutusu Stili ──────────────────────────────────────
    def _spin_style(self, color: str) -> str:
        return (
            f"QDoubleSpinBox, QSpinBox {{"
            f"background: rgba(0,0,0,0.3); border: 1px solid {C['b']}; "
            f"border-radius: 4px; color: {color}; "
            f"font-family: 'Share Tech Mono'; font-size: 13px; font-weight: bold; padding: 2px 4px;}}"
            f"QDoubleSpinBox::up-button, QSpinBox::up-button {{ width: 0px; }}"
            f"QDoubleSpinBox::down-button, QSpinBox::down-button {{ width: 0px; }}"
        )

    def _make_slider_panel(self):
        pnl = Panel("MANUAL JOINT CONTROL", "6-DOF · DEG")
        sw = QWidget(); sw.setStyleSheet("background:transparent;")
        sl_ly = QVBoxLayout(sw); sl_ly.setSpacing(15)
        self._sliders = []
        for i, j in enumerate(self._joints):
            box = QVBoxLayout(); box.setSpacing(5)
            hdr = QHBoxLayout()
            hdr.addWidget(make_label(f"{j['id']} · {j['ros']}", C["t2"], 12, bold=True))
            hdr.addStretch()
            
            # Manuel Değer Girme Kutusu (SpinBox)
            sb = QDoubleSpinBox()
            sb.setRange(j["mn"], j["mx"])
            sb.setDecimals(1)
            sb.setSingleStep(0.1)
            sb.setValue(j["v"])
            sb.setButtonSymbols(QDoubleSpinBox.NoButtons)
            sb.setStyleSheet(self._spin_style(C["a"]))
            sb.setFixedWidth(65)
            sb.setAlignment(Qt.AlignRight)
            
            hdr.addWidget(sb)
            hdr.addWidget(make_label("°", C["a"], 12, mono=True))
            
            s = QSlider(Qt.Horizontal)
            s.setRange(int(j["mn"] * 10), int(j["mx"] * 10))
            s.setValue(int(j["v"] * 10))
            s.setStyleSheet(slider_style(C["a"]))
            
            # Daha yumuşak hareket için
            s.setSingleStep(1)
            s.setPageStep(10)
            
            # Slider <-> Kutu Senkronizasyonu (Sonsuz döngüyü engeller)
            s.valueChanged.connect(lambda val, spin=sb, idx=i: (
                spin.blockSignals(True), spin.setValue(val / 10.0), spin.blockSignals(False),
                self._on_slider(idx, val / 10.0)
            ))
            sb.valueChanged.connect(lambda val, slider=s, idx=i: (
                slider.blockSignals(True), slider.setValue(int(val * 10)), slider.blockSignals(False),
                self._on_slider(idx, val)
            ))
            
            s.sliderPressed.connect(lambda: setattr(self, '_is_planning', True))
            sb.editingFinished.connect(lambda: setattr(self, '_is_planning', True))
            
            lims = QHBoxLayout()
            lims.addWidget(make_label(f"{j['mn']}°", C["t3"], 9, mono=True))
            lims.addStretch()
            lims.addWidget(make_label(f"{j['mx']}°", C["t3"], 9, mono=True))
            
            box.addLayout(hdr); box.addWidget(s); box.addLayout(lims)
            sl_ly.addLayout(box)
            self._sliders.append((s, sb)) 
        pnl.body.addWidget(sw)
        return pnl
    
    def _make_right_col(self):
        col = QVBoxLayout(); col.setSpacing(15)
        col.addWidget(self._make_mode_panel())   
        
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(self._tab_style())
        self.tabs.addTab(self._make_slider_panel(), "JOINTS")
        self.tabs.addTab(self._make_tcp_panel(), "TCP")
        self.tabs.addTab(self._make_limits_panel(), "LIMITS")
        
        col.addWidget(self.tabs, 1) 
        col.addWidget(self._make_gripper_panel())
        col.addStretch()
        return col

    def _make_cmd_panel(self):
        pnl = Panel("QUICK COMMANDS")
        cg = QHBoxLayout(); cg.setSpacing(8)
        for i, (ic, lb, kd) in enumerate([
            ("▶", "START",  "start"),
            ("⌂", "HOME",   "normal"),
            ("○", "ZERO",   "normal"),
            ]):
            b = CmdButton(ic, lb, kd)
            b.clicked.connect(lambda _, c=lb: self._cmd(c))
            cg.addWidget(b)
        pnl.body.addLayout(cg)
        return pnl

    def _make_limits_panel(self):
        pnl = Panel("SPEED & TORQUE LIMITS", "SAFETY")
        self._limit_sliders = {} 
        
        for lbl, mn, mx, val, unit, color in [
            ("JOINT SPEED",     0, 100,   100,  "%",    C["a2"]),
            ("TCP SPEED LIMIT", 0, 2500,  2500, "mm/s", C["a2"]),
            ("TORQUE LIMIT",    0, 100,   100,  "%",    C["a3"]),
            ("COLLISION SENSE", 0, 100,    50,  "%",    C["a3"]),
        ]:
            lh = QHBoxLayout()
            lh.addWidget(make_label(lbl, C["t2"], 10, mono=True))
            lh.addStretch()
            
            sb = QSpinBox() 
            sb.setRange(mn, mx)
            sb.setSingleStep(5 if mx > 100 else 1)
            sb.setValue(val)
            sb.setButtonSymbols(QSpinBox.NoButtons)
            sb.setStyleSheet(self._spin_style(C["t1"]))
            sb.setFixedWidth(60)
            sb.setAlignment(Qt.AlignRight)
            lh.addWidget(sb)
            lh.addWidget(make_label(unit, C["t1"], 12, bold=True))
            
            s = QSlider(Qt.Horizontal)
            s.setRange(mn, mx); s.setValue(val)
            s.setStyleSheet(slider_style(color))
            s.setSingleStep(5 if mx > 100 else 1)
            s.setPageStep(25 if mx > 100 else 10)
            
            s.valueChanged.connect(lambda v, spin=sb: (
                spin.blockSignals(True), spin.setValue(v), spin.blockSignals(False), self._on_limits_changed()
            ))
            sb.valueChanged.connect(lambda v, slider=s: (
                slider.blockSignals(True), slider.setValue(v), slider.blockSignals(False), self._on_limits_changed()
            ))
            
            lims2 = QHBoxLayout()
            lims2.addWidget(make_label(str(mn), C["t3"], 8, mono=True))
            lims2.addStretch()
            lims2.addWidget(make_label(str(mx), C["t3"], 8, mono=True))
            
            pnl.body.addLayout(lh)
            pnl.body.addWidget(s)
            pnl.body.addLayout(lims2)
            self._limit_sliders[lbl] = (s, sb)
            
        return pnl
    
    def _make_tcp_panel(self):
        pnl = Panel("CARTESIAN CONTROL", "TCP · MM/DEG")
        sw = QWidget(); sw.setStyleSheet("background:transparent;")
        sl_ly = QVBoxLayout(sw); sl_ly.setSpacing(15)
        
        self._tcp_sliders = {} 
        tcp_axes = [
            ("X", -1000, 1000, 0, "mm"), ("Y", -1000, 1000, 0, "mm"), ("Z", -500, 1500, 0, "mm"),
            ("RX", -180, 180, 0, "°"), ("RY", -180, 180, 0, "°"), ("RZ", -180, 180, 0, "°")
        ]
        for name, mn, mx, val, unit in tcp_axes:
            box = QVBoxLayout(); box.setSpacing(5)
            hdr = QHBoxLayout()
            hdr.addWidget(make_label(name, C["t2"], 11, bold=True))
            hdr.addStretch()
            
            sb = QDoubleSpinBox()
            sb.setRange(mn, mx)
            sb.setDecimals(1)
            sb.setSingleStep(1.0 if "mm" in unit else 0.5)
            sb.setValue(val)
            sb.setButtonSymbols(QDoubleSpinBox.NoButtons)
            sb.setStyleSheet(self._spin_style(C["a"]))
            sb.setFixedWidth(75)
            sb.setAlignment(Qt.AlignRight)
            hdr.addWidget(sb)
            hdr.addWidget(make_label(unit, C["a"], 11, mono=True))
            
            s = QSlider(Qt.Horizontal)
            s.setRange(mn * 10, mx * 10); s.setValue(val * 10)
            s.setStyleSheet(slider_style(C["a"]))
            s.setSingleStep(5)
            s.setPageStep(50)
            
            s.valueChanged.connect(lambda v, spin=sb: (
                spin.blockSignals(True), spin.setValue(v / 10.0), spin.blockSignals(False), self._on_tcp_slider_moved()
            ))
            sb.valueChanged.connect(lambda v, slider=s: (
                slider.blockSignals(True), slider.setValue(int(v * 10)), slider.blockSignals(False), self._on_tcp_slider_moved()
            ))
            
            s.sliderPressed.connect(lambda: setattr(self, '_is_planning', True))
            sb.editingFinished.connect(lambda: setattr(self, '_is_planning', True))
            
            box.addLayout(hdr); box.addWidget(s)
            sl_ly.addLayout(box)
            self._tcp_sliders[name] = (s, sb) 
        pnl.body.addWidget(sw)
        return pnl

    def _on_tcp_slider_moved(self):
        if not hasattr(self, '_tcp_sliders') or len(self._tcp_sliders) < 6:
            return
        x = (self._tcp_sliders["X"][0].value() / 10.0) / 1000.0
        y = (self._tcp_sliders["Y"][0].value() / 10.0) / 1000.0
        z = (self._tcp_sliders["Z"][0].value() / 10.0) / 1000.0
        rx = math.radians(self._tcp_sliders["RX"][0].value() / 10.0)
        ry = math.radians(self._tcp_sliders["RY"][0].value() / 10.0)
        rz = math.radians(self._tcp_sliders["RZ"][0].value() / 10.0)
        self._ros.publish_cartesian_ghost(x, y, z, rx, ry, rz)
    
    def _tab_style(self) -> str:
        return (
            f"QTabWidget::pane {{ border: 1px solid {C['b']}; border-radius: 4px; top: -1px; }}"
            f"QTabBar::tab {{ background: {C['card2']}; border: 1px solid {C['b']}; "
            f"min-width: 90px; padding: 8px; color: {C['t3']}; "
            f"font-family: 'Share Tech Mono'; font-size: 11px; margin-right: 2px; "
            f"border-top-left-radius: 4px; border-top-right-radius: 4px; }}"
            f"QTabBar::tab:selected {{ background: {C['card']}; color: {C['a']}; border-bottom-color: {C['card']}; }}"
            f"QTabBar::tab:hover {{ color: {C['a']}; }}"
        )

    def _make_gripper_panel(self):
        """Tutucu (Gripper) Kontrol Paneli."""
        pnl = Panel("GRIPPER CONTROL")
        ly = QHBoxLayout()
        ly.setSpacing(10)
        
        # OPEN ve CLOSE butonlarını oluşturuyoruz
        for txt in ["OPEN", "CLOSE"]:
            btn = QPushButton(txt)
            btn.setStyleSheet(self._mode_style())
            btn.setMinimumHeight(45) 
            
            # YENİ: Butona tıklandığında _send_gripper_cmd fonksiyonunu çağırır
            btn.clicked.connect(lambda _, t=txt: self._send_gripper_cmd(t))
            
            ly.addWidget(btn)
            
        pnl.body.addLayout(ly)
        return pnl
    def _send_gripper_cmd(self, action: str):
        """Gripper komutunu loglar ve doğrudan ROS motorlarına iletir."""
        color = "ok" if action == "OPEN" else "warn"
        self._ros.log_message.emit(f"GRIPPER: {action}", color)
        
        # Sadece text göndermek yerine doğrudan motoru tetikle!
        self._ros.set_gripper(action)
    
    def _make_rviz_panel(self):
        pnl = Panel("3D GHOST PREVIEW", "RVIZ2")
        self.rviz_container = QWidget()
        self.rviz_container.setMinimumHeight(220)  
        self.rviz_container.setStyleSheet(f"background:rgba(0,0,0,0.4); border:1px solid {C['b']}; border-radius:5px;")
        
        ly = QVBoxLayout(self.rviz_container)
        info_lbl = make_label("RViz2 Ekranı Harici Olarak Çalışıyor\n\nArayüze Gömmek İçin Butona Basın", C["t3"], 10, mono=True)
        info_lbl.setAlignment(Qt.AlignCenter)
        ly.addWidget(info_lbl)
        
        btn_embed = QPushButton("⚲ EMBED RVIZ WINDOW")
        btn_embed.setStyleSheet(self._mode_style()) 
        btn_embed.clicked.connect(self._try_embed_rviz)
        
        pnl.body.addWidget(self.rviz_container)
        pnl.body.addWidget(btn_embed)
        return pnl

    def _try_embed_rviz(self):
        try:
            out = subprocess.check_output(["xdotool", "search", "--class", "rviz2"]).decode('utf-8').strip()
            win_id_str = out.split('\n')[0]
            win_id = int(win_id_str)
            window = QWindow.fromWinId(win_id)
            rviz_widget = QWidget.createWindowContainer(window)
            
            for i in reversed(range(self.rviz_container.layout().count())): 
                self.rviz_container.layout().itemAt(i).widget().setParent(None)
                
            self.rviz_container.layout().addWidget(rviz_widget)
            print("\n[HMI] BAŞARILI: RViz2 penceresi arayüze gömüldü!")
            
        except Exception as e:
            print(f"\n[HATA] RViz2 penceresi bulunamadı veya gömülemedi: {e}")

    def update_tcp_from_ros(self, x, y, z, rx, ry, rz):
        if getattr(self, '_is_planning', False):
            return
        if not hasattr(self, '_tcp_sliders'):
            return
        new_values = {
            "X": x * 10, "Y": y * 10, "Z": z * 10,
            "RX": rx * 10, "RY": ry * 10, "RZ": rz * 10
        }
        for key, val in new_values.items():
            if key in self._tcp_sliders:
                s, sb = self._tcp_sliders[key]
                s.blockSignals(True); s.setValue(int(val)); s.blockSignals(False)
                sb.blockSignals(True); sb.setValue(val / 10.0); sb.blockSignals(False)

    def update_ghost_tcp(self, x, y, z, rx, ry, rz):
        if self.tabs.tabText(self.tabs.currentIndex()) != "JOINTS":
            return
        if not hasattr(self, '_tcp_sliders'):
            return
        new_values = {
            "X": x * 10, "Y": y * 10, "Z": z * 10,
            "RX": rx * 10, "RY": ry * 10, "RZ": rz * 10
        }
        for key, val in new_values.items():
            if key in self._tcp_sliders:
                s, sb = self._tcp_sliders[key]
                s.blockSignals(True); s.setValue(int(val)); s.blockSignals(False)
                sb.blockSignals(True); sb.setValue(val / 10.0); sb.blockSignals(False)

    def _on_slider(self, i: int, val: float):
        self._joints[i]["v"] = val
        self._is_planning = True
        if hasattr(self, '_sliders') and len(self._sliders) == len(self._joints):
            angles_deg = [s.value() / 10.0 for s, _sb in self._sliders]
            angles_rad = [math.radians(deg) for deg in angles_deg]
            self._ros.publish_ghost_robot(angles_rad)
        
    def _on_limits_changed(self):
        if not hasattr(self, '_limit_sliders') or len(self._limit_sliders) < 4:
            return
        j_spd = self._limit_sliders["JOINT SPEED"][0].value()
        t_spd = self._limit_sliders["TCP SPEED LIMIT"][0].value()
        trq   = self._limit_sliders["TORQUE LIMIT"][0].value()
        col   = self._limit_sliders["COLLISION SENSE"][0].value()
        self._ros.set_limits(j_spd, t_spd, trq, col)

    def update_canvas_from_ros(self, angles_rad: list):
        if getattr(self, '_is_planning', False):
            return
        for i, (s, sb) in enumerate(self._sliders):
            if i < len(angles_rad):
                deg = math.degrees(angles_rad[i]) 
                self._joints[i]["v"] = deg
                s.blockSignals(True); s.setValue(int(deg * 10)); s.blockSignals(False)
                sb.blockSignals(True); sb.setValue(deg); sb.blockSignals(False)

    def update_ghost_joints(self, angles_rad: list):
        if self.tabs.tabText(self.tabs.currentIndex()) != "TCP":
            return
        if not hasattr(self, '_sliders') or len(self._sliders) < len(angles_rad):
            return
        for i, (s, sb) in enumerate(self._sliders):
            deg = math.degrees(angles_rad[i])
            self._joints[i]["v"] = deg
            s.blockSignals(True); s.setValue(int(deg * 10)); s.blockSignals(False)
            sb.blockSignals(True); sb.setValue(deg); sb.blockSignals(False)

    def _mode_style(self) -> str:
        return (
            f"QPushButton{{background:transparent;border:1px solid {C['b']};"
            f"border-radius:4px;color:White;font-family:'Share Tech Mono';"
            f"font-size:12px;letter-spacing:.12em;padding:5px 10px;}}"
            f"QPushButton:checked{{background:rgba(0,212,170,0.15);"
            f"border-color:{C['a']};color:{C['a']};}}"
            f"QPushButton:hover{{border-color:{C['a']};color:{C['a']};}}"
            f"QPushButton:pressed{{background:rgba(255,255,255,0.2);}}"
        )

    def _set_mode(self, active_btn, mode: str):
        for b in self._mode_btns:
            b.setChecked(b is active_btn)
        self._ros.publish_cmd(f"MODE:{mode}")

    def _cmd(self, c: str):
        self._is_planning = False
        
        if c == "START":
            # --- YENİ: Teach Mode aktifse kayıtlı programı oynat ---
            if hasattr(self, '_btn_activate') and self._btn_activate.isChecked():
                self._start_teach_program()
                return
            # -------------------------------------------------------
            
            active_tab = self.tabs.tabText(self.tabs.currentIndex())
            
            if active_tab == "JOINTS":
                print("[HMI] START: Joint hedefleri MoveIt'e iletiliyor...")
                self._ros.log_message.emit("START: Joint hedefleri MoveIt'e iletiliyor...", "info")
                angles_deg = [s.value() / 10.0 for s, _sb in self._sliders]
                angles_rad = [math.radians(deg) for deg in angles_deg]
                self._ros.send_moveit_goal(angles_rad)
                
            elif active_tab == "TCP":
                print("[HMI] START: Kartezyen (TCP) hedefleri MoveIt'e iletiliyor...")
                self._ros.log_message.emit("START: Kartezyen hedefler MoveIt'e iletiliyor...", "info")
                x = (self._tcp_sliders["X"][0].value() / 10.0) / 1000.0
                y = (self._tcp_sliders["Y"][0].value() / 10.0) / 1000.0
                z = (self._tcp_sliders["Z"][0].value() / 10.0) / 1000.0
                rx = math.radians(self._tcp_sliders["RX"][0].value() / 10.0)
                ry = math.radians(self._tcp_sliders["RY"][0].value() / 10.0)
                rz = math.radians(self._tcp_sliders["RZ"][0].value() / 10.0)
                self._ros.send_moveit_cartesian_goal(x, y, z, rx, ry, rz)
                
        elif c == "HOME":
            print("[HMI] KOMUT: MoveIt 'HOME' pozisyonuna gidiyor...")
            self._ros.log_message.emit("KOMUT: MoveIt 'HOME' pozisyonuna gidiyor...", "ok")
            home_angles_deg = [0.0, 50.7, 86.5, -18.9, 80.5, 0.0]
            home_angles_rad = [math.radians(deg) for deg in home_angles_deg]
            self._ros.send_moveit_goal(home_angles_rad)
            
        elif c == "ZERO":
            print("[HMI] KOMUT: MoveIt 'ZERO' pozisyonuna gidiyor...")
            self._ros.log_message.emit("KOMUT: MoveIt 'ZERO' pozisyonuna gidiyor...", "warn")
            zero_angles_rad = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self._ros.send_moveit_goal(zero_angles_rad)
            
        elif c == "STOP":
            # YENİ: Eğer Teach Mode oynatılıyorsa durdur
            if hasattr(self, '_run_timer') and self._run_timer.isActive():
                self._run_timer.stop()
                self._ros.log_message.emit("PLAYBACK STOPPED by user.", "err")
            self._ros.publish_cmd(c)
            
        else:
            self._ros.publish_cmd(c)

    def _joy_stop(self):
        self._jx = self._jy = self._jz = self._jwz = 0.0
        self._joy_timer.stop()
        self._ros.publish_twist(0, 0, 0, 0)

    def _pub_twist(self):
        self._ros.publish_twist(
            self._jx * 0.25, self._jy * 0.25,
            self._jz * 0.25, self._jwz * 0.25,
        )