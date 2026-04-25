# ─────────────────────────────────────────────────────────────────────────────
#  viewer.py  —  Webots 3D akış görüntüleyicisi (QWebEngineView)
#
#  ÇALIŞMA MANTIĞI:
#   • __init__: sadece bekleme sayfası gösterir, Webots'a bağlanmaz
#   • connect_to_webots(): main.py'den simülasyon RUNNING olunca çağrılır
#   • disconnect_webots(): main.py'den Stop Sim'de çağrılır
#
#  DÜZELTME:
#   • _connected bayrağı kaldırıldı — setHtml() de loadFinished tetikler,
#     bu yüzden URL kontrolü ile ayırt ediyoruz (http mi data: mi)
#   • _manual_reload: doğrudan connect_to_webots() çağırır (browser.reload()
#     değil — reload bekleme sayfasını da yenileyebilir)
# ─────────────────────────────────────────────────────────────────────────────

import subprocess

from PyQt5.QtCore    import QUrl
from PyQt5.QtWidgets import QFrame, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings, QWebEnginePage

from config  import C
from widgets import make_label


class _WebPage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, msg, line, src):
        if "Unknown message received: \"time:" in msg:
            return
        tag = ["DBG", "INF", "WRN", "ERR"][level] if level < 4 else "???"
        print(f"  [JS:{tag} L{line}] {msg}")


class ViewerFrame(QFrame):
    WEBOTS_PORT = 1234

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

        self._status_lbl = make_label("● SİMÜLASYON BEKLENİYOR", C["t3"], 9, mono=True)
        hl.addWidget(self._status_lbl)
        hl.addSpacing(10)

        self.btn_reload = QPushButton("↻ RELOAD 3D")
        self.btn_reload.setStyleSheet(
            f"background:transparent;color:{C['t2']};"
            f"border:1px solid {C['t2']};border-radius:3px;"
            f"padding:2px 8px;font-size:9px;"
        )
        self.btn_reload.clicked.connect(self._manual_reload)
        self.btn_reload.setEnabled(False)
        hl.addWidget(self.btn_reload)
        ly.addWidget(hdr)

        # ── Windows IP tespiti ─────────────────────────────────────────────
        self._win_ip = self._detect_windows_ip()
        self._webots_url = f"http://{self._win_ip}:{self.WEBOTS_PORT}/index.html"
        print(f"[BİLGİ] WSL2 → Windows IP: {self._win_ip}")
        print(f"[BİLGİ] Webots URL: {self._webots_url}")

        # ── WebEngine ─────────────────────────────────────────────────────
        self.browser = QWebEngineView()
        self.browser.setPage(_WebPage(self.browser))
        self._apply_settings()

        # loadFinished her zaman tetiklenir — URL'e bakarak ne yapacağımızı belirleriz
        self.browser.loadFinished.connect(self._on_load_finished)

        # Başlangıç: bekleme sayfası
        self._show_waiting_page()

        ly.addWidget(self.browser, 1)

    # ── Public API ────────────────────────────────────────────────────────────

    def connect_to_webots(self):
        """Simülasyon RUNNING'e geçince main.py'den çağrılır."""
        self.btn_reload.setEnabled(True)
        self._set_status("● BAĞLANIYOR...", C["warn"])
        print(f"[BİLGİ] Webots'a bağlanılıyor → {self._webots_url}")
        self.browser.load(QUrl(self._webots_url))

    def disconnect_webots(self):
        """Stop Sim'de main.py'den çağrılır."""
        self.btn_reload.setEnabled(False)
        self.browser.stop()
        self._show_waiting_page()

    # ── İç metodlar ───────────────────────────────────────────────────────────

    def _show_waiting_page(self):
        self._set_status("● SİMÜLASYON BEKLENİYOR", C["t3"])
        html = (
            "<html><body style='"
            "margin:0;padding:0;background:#040c14;"
            "display:flex;align-items:center;justify-content:center;"
            "height:100vh;flex-direction:column;gap:12px;'>"
            f"<div style='color:{C['a']};font-family:Rajdhani,monospace;"
            "font-size:18px;font-weight:700;letter-spacing:.15em;'>"
            "WEBOTS 3D GÖRÜNÜM</div>"
            f"<div style='color:{C['t3']};font-family:monospace;font-size:11px;"
            "letter-spacing:.1em;'>▶ LAUNCH SIM ile simülasyonu başlatın</div>"
            "</body></html>"
        )
        self.browser.setHtml(html)

    def _apply_settings(self):
        s = self.browser.settings()
        s.setAttribute(QWebEngineSettings.WebGLEnabled,                    True)
        s.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        s.setAttribute(QWebEngineSettings.PluginsEnabled,                  True)
        s.setAttribute(QWebEngineSettings.JavascriptEnabled,               True)
        s.setAttribute(QWebEngineSettings.AllowRunningInsecureContent,     True)
        s.setAttribute(QWebEngineSettings.Accelerated2dCanvasEnabled,      True)
        s.setAttribute(QWebEngineSettings.ScrollAnimatorEnabled,           False)

    @staticmethod
    def _detect_windows_ip() -> str:
        try:
            out = subprocess.check_output(
                "ip route show default | awk '{print $3}'",
                shell=True, stderr=subprocess.DEVNULL
            ).decode().strip()
            if out:
                return out
        except Exception:
            pass
        try:
            with open("/etc/resolv.conf") as f:
                for line in f:
                    if line.startswith("nameserver"):
                        return line.split()[1].strip()
        except Exception:
            pass
        return "127.0.0.1"

    def _set_status(self, text: str, color: str):
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(
            f"color:{color};font-size:9px;"
            f"font-family:'Share Tech Mono';background:transparent;"
        )

    def _on_load_finished(self, ok: bool):
        """
        setHtml() ve browser.load() ikisi de bu callback'i tetikler.
        Bekleme sayfası için URL "about:blank" ya da "data:..." gelir.
        Webots sayfası için URL http://... gelir.
        Sadece Webots sayfası yüklendiğinde JS enjekte et.
        """
        current_url = self.browser.url().toString()

        # Bekleme sayfası (setHtml) → atla
        if not current_url.startswith("http"):
            return

        # Webots sayfası yüklenemedi
        if not ok:
            self._set_status("● SAYFA YÜKLENEMEDI — sunucu hazır mı?", C["red"])
            print("[HATA] Webots sayfası yüklenemedi")
            return

        # Başarıyla yüklendi → JS enjekte et
        self._set_status("● JS ENJEKTELENİYOR...", C["warn"])
        self._inject_autoconnect_js()

    def _inject_autoconnect_js(self):
        """Orijinal çalışan JS — dokunulmadı."""
        ws_url = f"ws://{self._win_ip}:{self.WEBOTS_PORT}"
        js = f"""
(function() {{

    var TARGET = '{ws_url}';

    var style = document.createElement('style');
    style.innerHTML = `
        #header, .header, header,
        form, #connect-panel, .connect-panel {{
            display: none !important;
        }}
        body {{
            margin: 0 !important;
            padding: 0 !important;
            background-color: #040c14 !important;
            overflow: hidden !important;
        }}
        webots-view, #webots-view {{
            width: 100vw !important;
            height: 100vh !important;
            display: block !important;
        }}
    `;
    document.head.appendChild(style);

    var btn = document.getElementById('connect-button');
    if (btn && btn.parentElement) {{
        btn.parentElement.style.display = 'none';
        if (btn.parentElement.previousElementSibling) {{
            btn.parentElement.previousElementSibling.style.display = 'none';
        }}
    }}

    function fillAndConnect() {{
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
                break;
            }}
        }}
        if (window.mode) {{
            window.mode.value = "mjpeg";
            window.mode.dispatchEvent(new Event('change', {{bubbles: true}}));
        }}
        if (btn) {{
            setTimeout(() => {{ btn.click(); }}, 500);
        }}
    }}

    fillAndConnect();
}})();
        """
        self.browser.page().runJavaScript(
            js,
            lambda result: self._set_status("● BAĞLANDI", C["ok"])
        )

    def _manual_reload(self):
        """RELOAD 3D butonu — sayfayı yeniden yükle ve JS enjekte et."""
        self._set_status("● YENİDEN BAĞLANIYOR...", C["warn"])
        # browser.reload() değil, connect_to_webots() — bu garantili çalışır
        self.connect_to_webots()