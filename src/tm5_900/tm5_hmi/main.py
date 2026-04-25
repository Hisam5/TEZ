#!/usr/bin/env python3
"""
Techman TM5-900 Robot HMI — PyQt5  (v2.4)

SIM YÖNETİCİSİ:
  BAŞLAT : subprocess.Popen ile ros2 launch → os.setsid ile yeni oturum
  DURDUR : os.killpg(SIGINT) → ros2 launch kendi alt düğümlerini kapatır

VIEWER BAĞLANTISI:
  Simülasyon RUNNING'e geçince → viewer.connect_to_webots()
  Simülasyon durursa          → viewer.disconnect_webots()

DÜZELTME:
  dashboard.py içinde viewer self._viewer_frame adıyla tanımlı.
  Önceki kodda getattr(self._dash, "_viewer", None) yazıyordu → None geliyordu.
  Şimdi getattr(self._dash, "_viewer_frame", None) olarak düzeltildi.
"""

import os
import sys
import signal
import subprocess

os.environ["QTWEBENGINE_DISABLE_GPU"]     = "1"
os.environ["QT_QPA_PLATFORM"]            = "xcb"
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = (
    "--use-gl=swiftshader --disable-gpu --disable-gpu-compositing "
    "--enable-webgl --ignore-gpu-blacklist --enable-accelerated-2d-canvas "
    "--no-sandbox --disable-software-rasterizer"
)
for _flag in ["--use-gl=swiftshader","--disable-gpu","--disable-gpu-compositing",
              "--enable-webgl","--ignore-gpu-blacklist","--no-sandbox"]:
    if _flag not in sys.argv:
        sys.argv.append(_flag)

import copy
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QStackedWidget
from PyQt5.QtCore    import Qt, QTimer, pyqtSlot, QProcess
from PyQt5.QtGui     import QFontDatabase

from config     import C, STYLE_BASE, TM5_JOINTS
from ros_worker import RosWorker
from topbar     import TopBar
from dashboard  import DashPage
from control    import CtrlPage

_LOADING_TIMEOUT_SEC = 180

_SIM_READY_TRIGGERS = [
    "extern controller: connected",
    "extern_controller: connected",
    "You can start planning now",
    "All is well! Everyone is happy!",
    "Successfully loaded controller",
    "Configured and activated",
]


class HMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Techman TM5-900 | HMI v2.4")
        self.setMinimumSize(900, 700)
        self.resize(1400, 900)
        self.setStyleSheet(STYLE_BASE)

        self._joints = copy.deepcopy(TM5_JOINTS)

        self._ros = RosWorker()
        self._ros.connection_changed.connect(self._on_ros_status)
        self._ros.joint_state_received.connect(self._on_joint_state)
        self._ros.log_message.connect(self._on_ros_log)
        self._ros.tcp_pose_received.connect(self._on_tcp_pose)
        self._ros.start()

        central = QWidget(); self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(0)

        self._tb = TopBar()
        self._tb.nav_changed.connect(self._nav)
        self._tb.sim_toggle_pressed.connect(self._toggle_sim)
        root.addWidget(self._tb)

        # ── Sim yöneticisi ────────────────────────────────────────────────
        self._sim_state  = "IDLE"
        self._sim_proc   = None
        self._output_buf = ""

        self._output_timer = QTimer(self)
        self._output_timer.setInterval(200)
        self._output_timer.timeout.connect(self._poll_output)

        self._loading_timer = QTimer(self)
        self._loading_timer.setSingleShot(True)
        self._loading_timer.timeout.connect(self._handle_loading_timeout)

        self._stop_check_timer = QTimer(self)
        self._stop_check_timer.setInterval(500)
        self._stop_check_timer.timeout.connect(self._check_stopped)
        self._stop_attempts = 0

        # Sayfa yığını
        self._stack = QStackedWidget()
        self._stack.setStyleSheet("background:transparent;")
        root.addWidget(self._stack, 1)

        self._dash = DashPage(self._ros, self._joints)
        self._ctrl = CtrlPage(self._ros, self._joints)
        self._stack.addWidget(_padded(self._dash))
        self._stack.addWidget(_padded(self._ctrl))
        self._pages = {"dash": 0, "ctrl": 1}

        # ── Viewer referansı — dashboard.py'deki gerçek isim: _viewer_frame ──
        self._viewer = getattr(self._dash, "_viewer_frame", None)
        if self._viewer is None:
            print("[UYARI] dashboard._viewer_frame bulunamadı — viewer bağlantısı devre dışı")

        self._clk_tmr = QTimer(self); self._clk_tmr.timeout.connect(self._tb.tick)
        self._clk_tmr.start(1000)

        self._up = 0
        self._up_tmr = QTimer(self); self._up_tmr.timeout.connect(self._tick_up)
        self._up_tmr.start(1000)

        self._demo_t = 0.0; self._demo_on = True
        self._demo_tmr = QTimer(self); self._demo_tmr.timeout.connect(self._tick_demo)
        self._demo_tmr.start(400)

        self._dash.log("HMI v2.4 başlatıldı", "ok")
        self._dash.log("WSL2 + Windows Webots modu", "info")
        self._dash.log("Eklem: " + " · ".join(j["ros"] for j in TM5_JOINTS), "info")
        if self._viewer:
            self._dash.log("Viewer hazır — simülasyon bekleniyor", "info")

    # ── Navigasyon ────────────────────────────────────────────────────────────
    def _nav(self, pg: str):
        self._stack.setCurrentIndex(self._pages.get(pg, 0))

    @pyqtSlot(float, float, float, float, float, float)
    def _on_tcp_pose(self, x, y, z, rx, ry, rz):
        self._dash.update_tcp(x, y, z, rx, ry, rz)
        self._ctrl.update_tcp_from_ros(x, y, z, rx, ry, rz)

    # ── Sim toggle ────────────────────────────────────────────────────────────
    def _toggle_sim(self):
        if self._sim_state in ("LOADING", "STOPPING"):
            return
        if self._sim_state == "IDLE":
            self._start_sim()
        else:
            self._stop_sim()

    # ── BAŞLAT ────────────────────────────────────────────────────────────────
    def _start_sim(self):
        self._sim_state  = "LOADING"
        self._output_buf = ""
        self._tb.set_sim_state("LOADING")
        self._dash.log("ros2 launch tm5_900 bringup.launch.py başlatılıyor...", "warn")

        launch_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "cd ~/ros2_ws && "
            "source install/setup.bash && "
            "ros2 launch tm5_900 bringup.launch.py"
        )

        try:
            self._sim_proc = subprocess.Popen(
                ["bash", "--login", "-c", launch_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid
            )
            self._dash.log(f"Süreç başlatıldı PID={self._sim_proc.pid}", "info")
            self._output_timer.start()
            self._loading_timer.start(_LOADING_TIMEOUT_SEC * 1000)
        except Exception as e:
            self._dash.log(f"Başlatma hatası: {e}", "err")
            self._sim_state = "IDLE"
            self._tb.set_sim_state("IDLE")

    # ── DURDUR ────────────────────────────────────────────────────────────────
    def _stop_sim(self):
        if self._sim_proc is None:
            self._reset_to_idle()
            return

        self._sim_state     = "STOPPING"
        self._stop_attempts = 0
        self._tb.set_sim_state("STOPPING")
        self._loading_timer.stop()
        self._dash.log("SIGINT gönderiliyor (Ctrl+C)...", "warn")

        if self._viewer:
            self._viewer.disconnect_webots()

        try:
            pgid = os.getpgid(self._sim_proc.pid)
            os.killpg(pgid, signal.SIGINT)
            self._dash.log(f"SIGINT → PGID {pgid}", "info")
        except ProcessLookupError:
            self._reset_to_idle()
            return
        except Exception as e:
            self._dash.log(f"SIGINT hatası: {e}", "warn")

        self._stop_check_timer.start()

    def _check_stopped(self):
        self._stop_attempts += 1

        if self._sim_proc is None or self._sim_proc.poll() is not None:
            self._stop_check_timer.stop()
            self._output_timer.stop()
            self._flush_output()
            self._reset_to_idle()
            self._dash.log("Simülasyon durduruldu — IDLE", "ok")
            return

        if self._stop_attempts >= 20:   # 10 saniye
            self._stop_check_timer.stop()
            self._dash.log("10s içinde kapanmadı — SIGKILL", "warn")
            try:
                pgid = os.getpgid(self._sim_proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass
            self._output_timer.stop()
            self._reset_to_idle()
            self._dash.log("Zorla durduruldu — IDLE", "ok")

    def _reset_to_idle(self):
        self._sim_state = "IDLE"
        self._sim_proc  = None
        self._tb.set_sim_state("IDLE")

    # ── Output okuma ──────────────────────────────────────────────────────────
    def _poll_output(self):
        if self._sim_proc is None:
            self._output_timer.stop()
            return

        if self._sim_proc.poll() is not None:
            self._flush_output()
            self._output_timer.stop()
            if self._sim_state not in ("STOPPING", "IDLE"):
                self._dash.log(
                    f"Süreç beklenmedik kapandı (exit={self._sim_proc.returncode})", "err"
                )
                if self._viewer:
                    self._viewer.disconnect_webots()
                self._reset_to_idle()
            return

        try:
            import select
            ready, _, _ = select.select([self._sim_proc.stdout], [], [], 0)
            if ready:
                chunk = os.read(
                    self._sim_proc.stdout.fileno(), 4096
                ).decode("utf-8", errors="replace")
                self._output_buf += chunk
                while "\n" in self._output_buf:
                    line, self._output_buf = self._output_buf.split("\n", 1)
                    self._process_line(line.rstrip())
        except Exception:
            pass

    def _flush_output(self):
        if self._output_buf.strip():
            self._process_line(self._output_buf.strip())
            self._output_buf = ""

    def _process_line(self, line: str):
        if not line:
            return

        if any(kw in line for kw in ("ERROR", "Exception", "Traceback")):
            self._dash.log(f"[SIM] {line[:80]}", "err")
            return

        if self._sim_state == "LOADING":
            for trigger in _SIM_READY_TRIGGERS:
                if trigger in line:
                    self._loading_timer.stop()
                    self._sim_state = "RUNNING"
                    self._tb.set_sim_state("RUNNING")
                    self._dash.log(f"✓ Simülasyon HAZIR — '{trigger}'", "ok")

                    if self._viewer:
                        self._dash.log("Webots 3D görünümüne bağlanılıyor...", "info")
                        self._viewer.connect_to_webots()
                    break

    # ── Zaman aşımı ───────────────────────────────────────────────────────────
    def _handle_loading_timeout(self):
        if self._sim_state == "LOADING":
            self._dash.log(
                f"⚠ {_LOADING_TIMEOUT_SEC}s içinde hazır sinyali gelmedi — "
                "RUNNING'e zorla alınıyor.", "warn"
            )
            self._sim_state = "RUNNING"
            self._tb.set_sim_state("RUNNING")
            if self._viewer:
                self._viewer.connect_to_webots()

    # ── ROS sinyal işleyicileri ───────────────────────────────────────────────
    @pyqtSlot(bool)
    def _on_ros_status(self, online: bool):
        self._tb.set_ros_status(online)
        self._demo_on = not online

    @pyqtSlot(list, list, list)
    def _on_joint_state(self, pos, vel, eff):
        self._dash.update_from_ros(pos, vel, eff)
        self._ctrl.update_canvas_from_ros(pos)

    @pyqtSlot(str, str)
    def _on_ros_log(self, msg, kind):
        self._dash.log(msg, kind)

    # ── Timer'lar ─────────────────────────────────────────────────────────────
    def _tick_up(self):
        self._up += 1
        m, s = divmod(self._up, 60)
        self._dash._sc_up.set_value(f"{m:02d}:{s:02d}")

    def _tick_demo(self):
        if self._demo_on:
            self._demo_t += 0.4
            self._dash.update_demo(self._demo_t)

    # ── Kapat ─────────────────────────────────────────────────────────────────
    def closeEvent(self, e):
        if self._sim_state != "IDLE":
            self._stop_sim()
        self._ros.stop()
        self._ros.wait(2000)
        super().closeEvent(e)


def _padded(w):
    c = QWidget(); c.setStyleSheet("background:transparent;")
    ly = QVBoxLayout(c); ly.setContentsMargins(0,0,0,0); ly.setSpacing(0)
    ly.addWidget(w)
    return c


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("Techman TM5-900 HMI")
    app.setStyleSheet(STYLE_BASE)
    QFontDatabase.addApplicationFont("Rajdhani-Regular.ttf")
    QFontDatabase.addApplicationFont("Exo2-Regular.ttf")
    win = HMI()
    win.show()
    sys.exit(app.exec_())