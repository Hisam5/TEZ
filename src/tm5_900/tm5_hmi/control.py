# ─────────────────────────────────────────────────────────────────────────────
#  control.py  —  Kontrol sayfası (2 sütunlu düzen)
#  Sol: Operasyon modu + Manuel eklem slider'ları
#  Sağ: Hızlı komutlar + Hız/Tork limitleri + Hazır görevler
# ─────────────────────────────────────────────────────────────────────────────

import math
import subprocess

from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QPushButton, QSlider, QFrame, QLabel,QTabWidget
)
from PyQt5.QtGui import QWindow
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
    def _build_ui(self): #
        ml = QHBoxLayout(self)
        ml.setContentsMargins(15, 15, 15, 15) 
        ml.setSpacing(15)

        ml.addLayout(self._make_left_col(), 1)
        ml.addLayout(self._make_right_col(), 1)

    # ── Sol sütun: Mod + Slider'lar ───────────────────────────────────────────
    def _make_left_col(self):
        col = QVBoxLayout(); col.setSpacing(15)
        col.addWidget(self._make_cmd_panel())
        col.addWidget(self._make_rviz_panel(),1)
        #col.addWidget(self._make_slider_panel(), 1)
        return col

    def _make_mode_panel(self):
        pnl = Panel("TEACH MODE")
        mg = QGridLayout(); mg.setSpacing(6)

        # --- 1. Satır: Activate/Inactive, Point, Delete p. ---
        
        # Activate Butonu
        self._btn_activate = QPushButton("INACTIVE")
        self._btn_activate.setCheckable(True) # Basılı kalabilme özelliği
        self._btn_activate.setStyleSheet(self._mode_style())
        self._btn_activate.setMinimumHeight(35)
        self._btn_activate.toggled.connect(self._on_activate_toggled)
        mg.addWidget(self._btn_activate, 0, 0) # 0. satır, 0. sütun

        # Point Butonu (Sadece tıklanabilir, basılı kalmaz)
        btn_point = QPushButton("POINT")
        btn_point.setStyleSheet(self._mode_style())
        btn_point.setMinimumHeight(35)
        # İleride bir fonksiyona bağlamak için: btn_point.clicked.connect(self._point_action)
        mg.addWidget(btn_point, 0, 1) # 0. satır, 1. sütun

        # Delete p. Butonu (Sadece tıklanabilir, basılı kalmaz)
        btn_delete = QPushButton("DELETE PROGRAM")
        btn_delete.setStyleSheet(self._mode_style())
        btn_delete.setMinimumHeight(35)
        # İleride bir fonksiyona bağlamak için: btn_delete.clicked.connect(self._delete_action)
        mg.addWidget(btn_delete, 0, 2) # 0. satır, 2. sütun

        # --- 2. Satır: Prog1, Prog2, Prog3 ---
        self._prog_btns = []
        for i, prog_name in enumerate(["PROG1", "PROG2", "PROG3"]):
            b = QPushButton(prog_name)
            b.setCheckable(True)
            b.setChecked(i == 0)  # Başlangıçta Prog1 seçili (highlighted) olsun
            b.setStyleSheet(self._mode_style())
            b.setMinimumHeight(35)
            # Tıklandığında sadece seçilen butonu aktif yapacak fonksiyona bağla
            b.clicked.connect(lambda _, btn=b: self._set_prog(btn))
            self._prog_btns.append(b)
            mg.addWidget(b, 1, i) # 1. satır, i. sütun (0, 1, 2)

        pnl.body.addLayout(mg)
        return pnl

    # ── TEACH MODE Yardımcı Fonksiyonları ─────────────────────────────────────

    def _on_activate_toggled(self, checked):
        """Activate butonu basılıysa metni Active, değilse Inactive yapar."""
        if checked:
            self._btn_activate.setText("ACTIVE")
        else:
            self._btn_activate.setText("INACTIVE")

    def _set_prog(self, active_btn):
        """Prog butonlarından sadece tıklananı aktif (highlighted) yapar."""
        for b in self._prog_btns:
            b.setChecked(b is active_btn)

    def _make_slider_panel(self):
        pnl = Panel("MANUAL JOINT CONTROL", "6-DOF · DEG")
        sw = QWidget(); sw.setStyleSheet("background:transparent;")
        sl_ly = QVBoxLayout(sw); sl_ly.setSpacing(15)
        self._sliders = []

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
        col.addWidget(self._make_mode_panel())   
        
        # Altta 3 Sekmeli Yapı (JOINTS, TCP, LIMITS)
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(self._tab_style())
        
        # Mevcut fonksiyonları sekme olarak ekliyoruz
        self.tabs.addTab(self._make_slider_panel(), "JOINTS")
        self.tabs.addTab(self._make_tcp_panel(), "TCP")
        self.tabs.addTab(self._make_limits_panel(), "LIMITS")
        
        col.addWidget(self.tabs, 1) 
                
        # EN ALTA: Gripper Kontrol Paneli Eklendi
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
    
    def _make_tcp_panel(self):
        """TCP (Kartezyen) Kontrol Sekmesi."""
        pnl = Panel("CARTESIAN CONTROL", "TCP · MM/DEG")
        sw = QWidget(); sw.setStyleSheet("background:transparent;")
        sl_ly = QVBoxLayout(sw); sl_ly.setSpacing(15)
        
        self._tcp_sliders = {} #new added
        # Eksenler ve Limitler
        tcp_axes = [
            ("X", -1000, 1000, 0, "mm"), ("Y", -1000, 1000, 0, "mm"), ("Z", -500, 1500, 0, "mm"),
            ("RX", -180, 180, 0, "°"), ("RY", -180, 180, 0, "°"), ("RZ", -180, 180, 0, "°")
        ]

        for name, mn, mx, val, unit in tcp_axes:
            box = QVBoxLayout(); box.setSpacing(5)
            hdr = QHBoxLayout()
            hdr.addWidget(make_label(name, C["t2"], 11, bold=True))
            hdr.addStretch()
            sv = make_label(f"{val:.1f}{unit}", C["a"], 11, mono=True)
            hdr.addWidget(sv)

            s = QSlider(Qt.Horizontal)
            s.setRange(mn * 10, mx * 10); s.setValue(val * 10)
            s.setStyleSheet(slider_style(C["a"]))
            s.valueChanged.connect(lambda v, lbl=sv, u=unit: lbl.setText(f"{v/10:.1f}{u}"))

            s.sliderPressed.connect(lambda: setattr(self, '_is_planning', True))#new added
            s.sliderMoved.connect(self._on_tcp_slider_moved)
            
            box.addLayout(hdr); box.addWidget(s)
            sl_ly.addLayout(box)
            self._tcp_sliders[name] = s #new added
        pnl.body.addWidget(sw)
        return pnl

    # ── TCP Slider'ı hareket ettirildiğinde çağrılan fonksiyon: IK çözümü yapıp Ghost Robotu günceller
    def _on_tcp_slider_moved(self, _):
        """Kullanıcı TCP slider'ını kaydırdığında arka planda IK çözüp Ghost Robotu günceller."""
        if not hasattr(self, '_tcp_sliders') or len(self._tcp_sliders) < 6:
            return

        # mm -> Metre
        x = (self._tcp_sliders["X"].value() / 10.0) / 1000.0
        y = (self._tcp_sliders["Y"].value() / 10.0) / 1000.0
        z = (self._tcp_sliders["Z"].value() / 10.0) / 1000.0

        # Derece -> Radyan
        rx = math.radians(self._tcp_sliders["RX"].value() / 10.0)
        ry = math.radians(self._tcp_sliders["RY"].value() / 10.0)
        rz = math.radians(self._tcp_sliders["RZ"].value() / 10.0)

        # Worker'a gönder
        self._ros.publish_cartesian_ghost(x, y, z, rx, ry, rz)
    
    def _tab_style(self) -> str:
        """Sekmelerin koyu tema tasarımı."""
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
            # Daha önce yaptığımız beyaz yazılı ve tıklama efektli stil
            btn.setStyleSheet(self._mode_style())
            btn.setMinimumHeight(45) # Biraz daha belirgin olması için yükseklik verdik
            
            # Tıklandığında ROS üzerinden komut göndermek istersen:
            # btn.clicked.connect(lambda _, t=txt: self._ros.publish_cmd(f"GRIPPER:{t}"))
            
            ly.addWidget(btn)
            
        pnl.body.addLayout(ly)
        return pnl
    
    # def _make_tasks_panel(self):
    #     pnl = Panel("PRESET TASKS", "WEBOTS")
    #     for no, nm, st_ in [
    #         ("01", "Home Position",      "rdy"),
    #         ("02", "Pick & Place A→B",   "idle"),
    #         ("05", "Custom Trajectory",  "idle"),
    #     ]:
    #         f = QFrame()
    #         f.setStyleSheet(
    #             f"QFrame{{background:rgba(0,0,0,0.25);"
    #             f"border:1px solid {C['b']};border-radius:5px;}}"
    #             f"QFrame:hover{{border-color:{C['bb']};}}"
    #         )
    #         f.setCursor(Qt.PointingHandCursor)
    #         ly2 = QHBoxLayout(f); ly2.setContentsMargins(12, 10, 12, 10)
    #         ly2.addWidget(make_label(no, C["t3"], 10, mono=True))
    #         ly2.addWidget(make_label(nm, C["t1"], 12))
    #         ly2.addStretch()

    #         sc, bg = {
    #             "rdy":  (C["a2"], "rgba(0,255,157,0.07)"),
    #             "done": (C["a"],  "rgba(0,212,170,0.07)"),
    #             "idle": (C["t3"], "rgba(255,255,255,0.02)"),
    #         }.get(st_, (C["t3"], "transparent"))

    #         sl2 = QLabel(st_.upper())
    #         sl2.setStyleSheet(
    #             f"color:{sc};background:{bg};border:1px solid {sc}40;"
    #             f"border-radius:10px;font-size:9px;padding:3px 8px;"
    #             f"font-family:'Share Tech Mono';"
    #         )
    #         ly2.addWidget(sl2)
    #         pnl.body.addWidget(f)
    #     return pnl

    
    def _make_rviz_panel(self):
        pnl = Panel("3D GHOST PREVIEW", "RVIZ2")
        
        # RViz'in gömüleceği siyah ekran kutusu
        self.rviz_container = QWidget()
        self.rviz_container.setMinimumHeight(220)  # Yüksekliği ayarlayabilirsin
        self.rviz_container.setStyleSheet(f"background:rgba(0,0,0,0.4); border:1px solid {C['b']}; border-radius:5px;")
        
        # İlk açıldığında görünecek bilgi metni
        ly = QVBoxLayout(self.rviz_container)
        info_lbl = make_label("RViz2 Ekranı Harici Olarak Çalışıyor\n\nArayüze Gömmek İçin Butona Basın", C["t3"], 10, mono=True)
        info_lbl.setAlignment(Qt.AlignCenter)
        ly.addWidget(info_lbl)
        
        # Gömme Hilesini Tetikleyecek Buton
        btn_embed = QPushButton("⚲ EMBED RVIZ WINDOW")
        btn_embed.setStyleSheet(self._mode_style()) # Sol taraftaki butonların stilini kullandık
        btn_embed.clicked.connect(self._try_embed_rviz)
        
        pnl.body.addWidget(self.rviz_container)
        pnl.body.addWidget(btn_embed)
        return pnl

    def _try_embed_rviz(self):
        """X11 Window Reparenting Hilesi: RViz2 penceresini zorla PyQt5 içine alır."""
        try:
            # xdotool komutu ile "rviz2" sınıfına sahip pencerenin ID'sini bul
            out = subprocess.check_output(["xdotool", "search", "--class", "rviz2"]).decode('utf-8').strip()
            win_id_str = out.split('\n')[0]
            win_id = int(win_id_str)
            
            # Pencereyi PyQt5 nesnesi haline getir
            window = QWindow.fromWinId(win_id)
            rviz_widget = QWidget.createWindowContainer(window)
            
            # Container'ın içindeki eski bilgi etiketini sil
            for i in reversed(range(self.rviz_container.layout().count())): 
                self.rviz_container.layout().itemAt(i).widget().setParent(None)
                
            # RViz2 penceresini kutunun içine sıkıştır
            self.rviz_container.layout().addWidget(rviz_widget)
            print("\n[HMI] BAŞARILI: RViz2 penceresi arayüze gömüldü!")
            
        except Exception as e:
            print(f"\n[HATA] RViz2 penceresi bulunamadı veya gömülemedi: {e}")
            print("[ÇÖZÜM] Terminalde 'xdotool' aracının kurulu olduğundan emin olun:")
            print("Komut: sudo apt install xdotool\n")
    
    # ── ROS güncellemesi (TCP pozisyonunu güncelle) ───────────────────────────────
    def update_tcp_from_ros(self, x, y, z, rx, ry, rz):
        """Updates the TCP sliders with real-time TF data from ROS."""
        # If the user is currently dragging a slider, do NOT overwrite their target
        if getattr(self, '_is_planning', False):
            return
            
        if not hasattr(self, '_tcp_sliders'):
            return
        # Block signals briefly so setting the value doesn't trigger endless loops
        for s in self._tcp_sliders.values():
            s.blockSignals(True)
        self._tcp_sliders["X"].setValue(int(x * 10))
        self._tcp_sliders["Y"].setValue(int(y * 10))
        self._tcp_sliders["Z"].setValue(int(z * 10))
        self._tcp_sliders["RX"].setValue(int(rx * 10))
        self._tcp_sliders["RY"].setValue(int(ry * 10))
        self._tcp_sliders["RZ"].setValue(int(rz * 10))
        for s in self._tcp_sliders.values():
            s.blockSignals(False)
            
        # Manually trigger the label update for the new values
        self._tcp_sliders["X"].valueChanged.emit(int(x * 10))
        self._tcp_sliders["Y"].valueChanged.emit(int(y * 10))
        self._tcp_sliders["Z"].valueChanged.emit(int(z * 10))
        self._tcp_sliders["RX"].valueChanged.emit(int(rx * 10))
        self._tcp_sliders["RY"].valueChanged.emit(int(ry * 10))
        self._tcp_sliders["RZ"].valueChanged.emit(int(rz * 10))

            
    # ── Slider callback ───────────────────────────────────────────────────────
    def _on_slider(self, i: int, val: float, lbl):
        self._joints[i]["v"] = val
        lbl.setText(f"{val:.1f}°")
        
        self._is_planning = True
        # EĞER PLANLAMA MODUNDAYSAK (Kullanıcı slider'ı tutuyorsa):
        # Gerçek robotu değil, RViz'deki hayalet robotu hareket ettir!
        if hasattr(self, '_sliders') and len(self._sliders) == len(self._joints): #d
            angles_deg = [s.value() / 10.0 for s, _lbl in self._sliders]
            angles_rad = [math.radians(deg) for deg in angles_deg]
            self._ros.publish_ghost_robot(angles_rad)
        #self._ros.publish_joints([j["v"] for j in self._joints])
        #ROS'a eklem açılarını yayınla dierk siderden değil tüm değişikliği yaptıktan
        #sonra START buttona basildığında ÇARPIŞMA KONTROLÜYLE birlikte hareket etmesi sağlanacak 
        
    def _on_slider_pressed(self, idx: int):
        """Kullanıcı slider'a tıkladığında ROS güncellemelerini durdurur."""
        self._is_planning = True    
        

    def _on_slider_released(self, idx: int):
        """Kullanıcı slider'ı bıraktığında ROS güncellemelerini tekrar açar."""
        pass
    

    # ── ROS güncellemesi (slider'ları güncelle) ───────────────────────────────
    def update_canvas_from_ros(self, angles_rad: list):
        
        if getattr(self, '_is_planning', False):
            return
        
        for i, (s, lbl) in enumerate(self._sliders):
            if i < len(angles_rad):
                deg = math.degrees(angles_rad[i]) 
                self._joints[i]["v"] = deg
                s.blockSignals(True); s.setValue(int(deg * 10)); s.blockSignals(False)
                lbl.setText(f"{deg:.1f}°")

    # ── Mod seçimi ────────────────────────────────────────────────────────────
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

    # ── Komut gönderme ────────────────────────────────────────────────────────
    def _cmd(self, c: str):
        self._is_planning = False
        
        if c == "START":
            active_tab = self.tabs.tabText(self.tabs.currentIndex())
            
            if active_tab == "JOINTS":
                print("[HMI] START: Joint hedefleri MoveIt'e iletiliyor...")
                angles_deg = [s.value() / 10.0 for s, _lbl in self._sliders]
                angles_rad = [math.radians(deg) for deg in angles_deg]
                self._ros.send_moveit_goal(angles_rad)
                
            elif active_tab == "TCP":
                print("[HMI] START: Kartezyen (TCP) hedefleri MoveIt'e iletiliyor...")
                
                # Sliders output integers (value * 10). 
                # Convert mm to meters (for MoveIt XYZ) and degrees to radians (for MoveIt RPY)
                x = (self._tcp_sliders["X"].value() / 10.0) / 1000.0
                y = (self._tcp_sliders["Y"].value() / 10.0) / 1000.0
                z = (self._tcp_sliders["Z"].value() / 10.0) / 1000.0
                
                rx = math.radians(self._tcp_sliders["RX"].value() / 10.0)
                ry = math.radians(self._tcp_sliders["RY"].value() / 10.0)
                rz = math.radians(self._tcp_sliders["RZ"].value() / 10.0)
                
                self._ros.send_moveit_cartesian_goal(x, y, z, rx, ry, rz)
        else:
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
