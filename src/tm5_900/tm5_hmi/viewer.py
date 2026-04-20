# ─────────────────────────────────────────────────────────────────────────────
#  viewer.py  —  Webots 3D akış görüntüleyicisi (QWebEngineView)
#  WSL2 → Windows gateway IP'sini otomatik bulur
#  Sayfa yüklendikten sonra JS ile WebSocket bağlantısını otomatik kurar
# ─────────────────────────────────────────────────────────────────────────────

import subprocess

from PyQt5.QtCore    import QUrl
from PyQt5.QtWidgets import QFrame, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings, QWebEnginePage

from config  import C
from widgets import make_label


# ── JS console mesajlarını terminale yansıtan özel sayfa ─────────────────────

class _WebPage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, msg, line, src):
        tag = ["DBG", "INF", "WRN", "ERR"][level] if level < 4 else "???"
        print(f"  [JS:{tag} L{line}] {msg}")


# ── Ana ViewerFrame bileşeni ──────────────────────────────────────────────────

class ViewerFrame(QFrame):
    """
    Webots R2025a streaming viewer'ını gömülü olarak gösteren panel.
    - Windows IP'yi WSL2 varsayılan ağ geçidinden otomatik bulur.
    - Sayfa yüklendikten sonra JS ile URL alanını doldurur ve
      Connect butonuna otomatik tıklar (retry döngüsüyle).
    """

    # Bağlantı ayarları
    WEBOTS_PORT    = 1234
    INITIAL_WAIT   = 3000   # ms  — Parsing tamamlanması için ilk bekleme
    RETRY_INTERVAL = 1000   # ms  — Her deneme arasındaki bekleme
    MAX_RETRIES    = 10     # kez — Maksimum deneme sayısı

    def __init__(self, title: str = "TM5-900 · LIVE WEBOTS SCENE", parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"ViewerFrame{{background:#040c14;"
            f"border:1px solid {C['b']};border-radius:7px;}}"
        )
        ly = QVBoxLayout(self)
        ly.setContentsMargins(0, 0, 0, 0)
        ly.setSpacing(0)

        # ── Başlık barı ────────────────────────────────────────────────────
        hdr = QFrame(); hdr.setFixedHeight(32)
        hdr.setStyleSheet(
            "background:rgba(4,12,20,0.92);"
            "border-bottom:1px solid rgba(0,212,170,0.12);"
        )
        hl = QHBoxLayout(hdr); hl.setContentsMargins(10, 0, 10, 0)
        hl.addWidget(make_label(title, C["a"], 10, bold=True))
        hl.addStretch()

        self._status_lbl = make_label("● BAĞLANIYOR...", C["warn"], 9, mono=True)
        hl.addWidget(self._status_lbl)
        hl.addSpacing(10)

        btn_reload = QPushButton("↻ RELOAD 3D")
        btn_reload.setStyleSheet(
            f"background:transparent;color:{C['t2']};"
            f"border:1px solid {C['t2']};border-radius:3px;"
            f"padding:2px 8px;font-size:9px;"
        )
        btn_reload.clicked.connect(self._manual_reload)
        hl.addWidget(btn_reload)
        ly.addWidget(hdr)

        # ── Windows IP tespiti ─────────────────────────────────────────────
        self._win_ip = self._detect_windows_ip()
        print(f"\n[BİLGİ] WSL2 → Windows IP: {self._win_ip}")
        print(f"[BİLGİ] Webots URL: http://{self._win_ip}:{self.WEBOTS_PORT}/index.html\n")

        # ── WebEngine kurulumu ─────────────────────────────────────────────
        self.browser = QWebEngineView()
        self.browser.setPage(_WebPage(self.browser))
        self._apply_settings()

        self.browser.loadStarted.connect(
            lambda: self._set_status("● YÜKLENİYOR...", C["warn"])
        )
        self.browser.loadFinished.connect(self._on_load_finished)

        self.browser.setUrl(
            QUrl(f"http://{self._win_ip}:{self.WEBOTS_PORT}/index.html")
        )
        ly.addWidget(self.browser, 1)

    # ── Ayarlar ───────────────────────────────────────────────────────────────
    def _apply_settings(self):
        s = self.browser.settings()
        s.setAttribute(QWebEngineSettings.WebGLEnabled,                     True)
        s.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls,  True)
        s.setAttribute(QWebEngineSettings.PluginsEnabled,                   True)
        s.setAttribute(QWebEngineSettings.JavascriptEnabled,                True)
        s.setAttribute(QWebEngineSettings.AllowRunningInsecureContent,      True)
        s.setAttribute(QWebEngineSettings.Accelerated2dCanvasEnabled,       True)
        s.setAttribute(QWebEngineSettings.ScrollAnimatorEnabled,            False)

    # ── Windows IP tespiti ────────────────────────────────────────────────────
    @staticmethod
    def _detect_windows_ip() -> str:
        # Yöntem 1: varsayılan ağ geçidi
        try:
            out = subprocess.check_output(
                "ip route show default | awk '{print $3}'",
                shell=True, stderr=subprocess.DEVNULL
            ).decode().strip()
            if out:
                return out
        except Exception:
            pass
        # Yöntem 2: /etc/resolv.conf nameserver
        try:
            with open("/etc/resolv.conf") as f:
                for line in f:
                    if line.startswith("nameserver"):
                        return line.split()[1].strip()
        except Exception:
            pass
        return "127.0.0.1"   # son çare

    # ── Durum etiketi ─────────────────────────────────────────────────────────
    def _set_status(self, text: str, color: str):
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(
            f"color:{color};font-size:9px;"
            f"font-family:'Share Tech Mono';background:transparent;"
        )

    # ── Sayfa yüklendi callback ───────────────────────────────────────────────
    def _on_load_finished(self, ok: bool):
        if not ok:
            self._set_status("● SAYFA YÜKLENEMEDI", C["red"])
            print("[HATA] Webots sayfası yüklenemedi — sunucu açık mı?")
            return

        self._set_status("● BAĞLANIYOR...", C["warn"])
        self._inject_autoconnect_js()

    # ── JS enjeksiyonu: URL doldur + Connect'e bas ────────────────────────────
    def _inject_autoconnect_js(self):
        ws_url = f"ws://{self._win_ip}:{self.WEBOTS_PORT}"
        js = f"""
(function() {{
    var TARGET = '{ws_url}';
    var MAX    = {self.MAX_RETRIES};
    var DELAY  = {self.RETRY_INTERVAL};

    function fillAndConnect() {{
        // 1) WebSocket URL input alanını doldur
        var inputs = document.querySelectorAll('input[type="text"], input:not([type])');
        for (var i = 0; i < inputs.length; i++) {{
            var v = inputs[i].value || '';
            if (v.indexOf('localhost') !== -1 ||
                v.indexOf('1234')      !== -1 ||
                v.indexOf('ws://')     !== -1 ||
                v === '') {{
                inputs[i].value = TARGET;
                inputs[i].dispatchEvent(new Event('input',  {{bubbles:true}}));
                inputs[i].dispatchEvent(new Event('change', {{bubbles:true}}));
                console.log('[HMI] Input → ' + TARGET);
                break;
            }}
        }}
        
        if (mode) {{
            mode.value = "mjpeg";
            mode.dispatchEvent(new Event('change', {{ bubbles: true }}));
            console.log("Forced mode: mjpeg");
        }}

        // 2) Connect butonunu bul ve tıkla
        
        const btn = document.getElementById('connect-button');
        
        if (btn) {{
                setTimeout(() => {{ 
                    btn.click();
                    console.log("Auto connect clicked");
                }}, 500);
        }}
        
        return true;
    }}

    fillAndConnect();
}})();
        """
        self.browser.page().runJavaScript(js)

    # ── Manuel reload ─────────────────────────────────────────────────────────
    def _manual_reload(self):
        self._set_status("● YENİDEN BAĞLANIYOR...", C["warn"])
        self.browser.reload()