#!/usr/bin/env python3
"""
Techman TM5-900 Robot HMI — PyQt5
==================================================================
Başlatma sırası:
  1. ros2 launch tm5_900 webots.launch.py
  2. python3 main.py

Bağımlılıklar:
  sudo apt install python3-pyqt5 python3-pyqt5.qtwebengine
"""

# ─────────────────────────────────────────────────────────────────────────────
#  KRİTİK: Tüm Qt import'larından ÖNCE ortam değişkenleri ayarlanmalı
# ─────────────────────────────────────────────────────────────────────────────
import os
import sys

# WSL2'de GPU yoktur — SwiftShader yazılımsal renderer zorunlu
os.environ["QTWEBENGINE_DISABLE_GPU"]     = "1"
os.environ["QT_QPA_PLATFORM"]            = "xcb"
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = (
    "--use-gl=swiftshader "
    "--disable-gpu "
    "--disable-gpu-compositing "
    "--enable-webgl "
    "--ignore-gpu-blacklist "
    "--enable-accelerated-2d-canvas "
    "--no-sandbox "
    "--disable-software-rasterizer"
)

# sys.argv üzerinden de geçirilmesi ZORUNLU (env değişkeni tek başına yetmez)
for _flag in [
    "--use-gl=swiftshader",
    "--disable-gpu",
    "--disable-gpu-compositing",
    "--enable-webgl",
    "--ignore-gpu-blacklist",
    "--no-sandbox",
]:
    if _flag not in sys.argv:
        sys.argv.append(_flag)

# ─────────────────────────────────────────────────────────────────────────────
#  Qt ve proje modülleri (env ayarlandıktan SONRA import edilmeli)
# ─────────────────────────────────────────────────────────────────────────────
import copy

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QStackedWidget
from PyQt5.QtCore    import Qt, QTimer, pyqtSlot
from PyQt5.QtGui     import QFontDatabase

from config     import C, STYLE_BASE, TM5_JOINTS
from ros_worker import RosWorker
from topbar     import TopBar
from dashboard  import DashPage
from control    import CtrlPage


# ─────────────────────────────────────────────────────────────────────────────
#  Ana pencere
# ─────────────────────────────────────────────────────────────────────────────

class HMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Techman TM5-900 | HMI v2.0")
        self.setMinimumSize(900, 700)
        self.resize(1400, 900)
        self.setStyleSheet(STYLE_BASE)

        # Eklem durumu (DashPage ve CtrlPage paylaşır)
        self._joints = copy.deepcopy(TM5_JOINTS)

        # ROS 2 worker
        self._ros = RosWorker()
        self._ros.connection_changed.connect(self._on_ros_status)
        self._ros.joint_state_received.connect(self._on_joint_state)
        self._ros.log_message.connect(self._on_ros_log)
        self._ros.start()

        # Merkez widget + dikey düzen
        central = QWidget(); self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # TopBar
        self._tb = TopBar()
        self._tb.nav_changed.connect(self._nav)
        self._tb.estop_pressed.connect(self._estop)
        root.addWidget(self._tb)

        # Sayfa yığını
        self._stack = QStackedWidget()
        self._stack.setStyleSheet("background:transparent;")
        root.addWidget(self._stack, 1)

        self._dash = DashPage(self._ros, self._joints)
        self._ctrl = CtrlPage(self._ros, self._joints)
        self._stack.addWidget(_padded(self._dash))   # index 0
        self._stack.addWidget(_padded(self._ctrl))   # index 1
        self._pages = {"dash": 0, "ctrl": 1}

        # Saat timer'ı (1 sn)
        self._clk_tmr = QTimer(self)
        self._clk_tmr.timeout.connect(self._tb.tick)
        self._clk_tmr.start(1000)

        # Uptime sayacı (1 sn)
        self._up = 0
        self._up_tmr = QTimer(self)
        self._up_tmr.timeout.connect(self._tick_up)
        self._up_tmr.start(1000)

        # Demo animasyonu (ROS yokken, 400ms)
        self._demo_t  = 0.0
        self._demo_on = True
        self._demo_tmr = QTimer(self)
        self._demo_tmr.timeout.connect(self._tick_demo) 
        self._demo_tmr.start(400)

        # Başlangıç log mesajları
        self._dash.log("HMI v2.0 başlatıldı", "ok")
        self._dash.log("SwiftShader WebGL renderer aktif (WSL2 modu)", "info")
        self._dash.log(
            "Eklem adları: " + " · ".join(j["ros"] for j in TM5_JOINTS), "info"
        )
#================================================================
        #this added
        self._ros.tcp_pose_received.connect(self._on_tcp_pose)
        # --- GHOST SYNC SİNYALLERİ ---
        self._ros.ghost_joints_received.connect(self._on_ghost_joints)
        self._ros.ghost_tcp_received.connect(self._on_ghost_tcp)
#================================================================

    # ── Navigasyon ────────────────────────────────────────────────────────────
    def _nav(self, pg: str):
        self._stack.setCurrentIndex(self._pages.get(pg, 0))
#================================================================
#this added
    @pyqtSlot(float, float, float, float, float, float)
    def _on_tcp_pose(self, x, y, z, rx, ry, rz):
        self._dash.update_tcp(x, y, z, rx, ry, rz)
        self._ctrl.update_tcp_from_ros(x, y, z, rx, ry, rz)
    @pyqtSlot(list)
    def _on_ghost_joints(self, angles_rad):
        self._ctrl.update_ghost_joints(angles_rad)

    @pyqtSlot(float, float, float, float, float, float)
    def _on_ghost_tcp(self, x, y, z, rx, ry, rz):
        self._ctrl.update_ghost_tcp(x, y, z, rx, ry, rz)
#================================================================
    # ── E-STOP ────────────────────────────────────────────────────────────────
    def _estop(self):
        self._dash.log("⚠ ACİL DURUM BUTONU BASILDI", "err")
        self._ros.publish_twist(0, 0, 0, 0)
        self._ros.publish_cmd("ESTOP")
        self._dash._sc_state.set_value("E-STOP", C["red"])

    # ── ROS sinyal işleyicileri ───────────────────────────────────────────────
    @pyqtSlot(bool)
    def _on_ros_status(self, online: bool):
        self._tb.set_ros_status(online)
        self._demo_on = not online   # ROS aktifken demo durur

    @pyqtSlot(list, list, list)
    def _on_joint_state(self, pos: list, vel: list, eff: list):
        self._dash.update_from_ros(pos, vel, eff)
        self._ctrl.update_canvas_from_ros(pos)

    @pyqtSlot(str, str)
    def _on_ros_log(self, msg: str, kind: str):
        self._dash.log(msg, kind)

    # ── Timer callback'leri ───────────────────────────────────────────────────
    def _tick_up(self):
        self._up += 1
        m, s = divmod(self._up, 60)
        self._dash._sc_up.set_value(f"{m:02d}:{s:02d}")

    def _tick_demo(self):  #
        if not self._demo_on:
            return
        self._demo_t += 0.4
        self._dash.update_demo(self._demo_t)

    # ── Pencere kapatma ───────────────────────────────────────────────────────
    def closeEvent(self, e):
        self._ros.stop()
        self._ros.wait(2000)
        super().closeEvent(e)


# ─────────────────────────────────────────────────────────────────────────────
#  Yardımcı: widget'ı içine alan saydam kaplama
# ─────────────────────────────────────────────────────────────────────────────

def _padded(w: QWidget) -> QWidget:
    c = QWidget(); c.setStyleSheet("background:transparent;")
    ly = QVBoxLayout(c); ly.setContentsMargins(0, 0, 0, 0); ly.setSpacing(0)
    ly.addWidget(w)
    return c


# ─────────────────────────────────────────────────────────────────────────────
#  Giriş noktası
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("Techman TM5-900 HMI")
    app.setStyleSheet(STYLE_BASE)

    # Özel fontlar (TTF dosyaları proje klasöründe olmalı)
    QFontDatabase.addApplicationFont("Rajdhani-Regular.ttf")
    QFontDatabase.addApplicationFont("Exo2-Regular.ttf")

    win = HMI()
    win.show()
    sys.exit(app.exec_())   