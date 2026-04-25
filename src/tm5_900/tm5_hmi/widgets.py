# ─────────────────────────────────────────────────────────────────────────────
#  widgets.py  —  Yeniden kullanılabilir UI bileşenleri
#  Panel | StatCard | JointCard | TcpCell | CmdButton | LogWidget
# ─────────────────────────────────────────────────────────────────────────────

from datetime import datetime

from PyQt5.QtWidgets import (
    QFrame, QWidget, QLabel, QPushButton, QProgressBar,
    QTextEdit, QVBoxLayout, QHBoxLayout,
)
from PyQt5.QtCore import Qt

from config import C


# ── Yardımcı fonksiyonlar ─────────────────────────────────────────────────────

def make_label(text: str, color=C["t1"], size: int = 11,
               bold: bool = False, mono: bool = False) -> QLabel:
    lbl = QLabel(text)
    fam = "'Share Tech Mono'" if mono else "'Rajdhani',sans-serif"
    lbl.setStyleSheet(
        f"color:{color}; font-size:{size}px; "
        f"font-weight:{'bold' if bold else 'normal'}; "
        f"font-family:{fam}; background:transparent;"
    )
    return lbl


def slider_style(color: str) -> str:
    return (
        f"QSlider::groove:horizontal{{"
        f"height:4px;background:rgba(255,255,255,0.07);border-radius:2px;}}"
        f"QSlider::handle:horizontal{{"
        f"width:13px;height:13px;margin:-5px 0;border-radius:7px;"
        f"background:{color};border:2px solid {C['bg']};}}"
        f"QSlider::sub-page:horizontal{{background:{color};border-radius:2px;}}"
    )


# ── Panel ─────────────────────────────────────────────────────────────────────

class Panel(QFrame):
    """Başlık barı olan koyu kart konteyneri."""

    def __init__(self, title: str, tag: str = "", parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"Panel{{background:{C['card']};border:1px solid {C['b']};border-radius:7px;}}"
        )
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # Başlık
        hdr = QFrame()
        hdr.setFixedHeight(24)
        hdr.setStyleSheet(
            f"background:rgba(0,0,0,0.2);border-bottom:1px solid {C['b']};border-radius:0;"
        )
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(11, 0, 11, 0)
        hl.addWidget(make_label(title, C["a"], 10, bold=True))
        hl.addStretch()
        if tag:
            hl.addWidget(make_label(tag, C["t3"], 8, mono=True))
        outer.addWidget(hdr)

        # İçerik alanı
        self.content = QWidget()
        self.content.setStyleSheet("background:transparent;")
        self.body = QVBoxLayout(self.content)
        self.body.setContentsMargins(10, 10, 10, 10)
        self.body.setSpacing(8)
        outer.addWidget(self.content)


# ── StatCard ──────────────────────────────────────────────────────────────────

class StatCard(QFrame):
    """Tek değer gösteren küçük istatistik kartı."""

    def __init__(self, label: str, value: str, sub: str,
                 kind: str = "b", parent=None):
        super().__init__(parent)
        self.setMinimumHeight(60)
        self.setStyleSheet(
            f"StatCard{{background:{C['card2']};border:1px solid {C['b']};border-radius:6px;}}"
        )
        ly = QVBoxLayout(self)
        ly.setContentsMargins(10, 8, 10, 8)
        ly.setSpacing(2)
        self.lbl = make_label(label, C["t3"], 10, mono=True)
        self.val = make_label(value, C["t1"], 18, bold=True)
        self.sub = make_label(sub,   C["t3"],  8, mono=True)
        ly.addWidget(self.lbl)
        ly.addWidget(self.val)
        ly.addWidget(self.sub)

    def set_value(self, v, color: str = None):
        self.val.setText(str(v))
        if color:
            self.val.setStyleSheet(
                f"color:{color};font-size:18px;font-weight:bold;background:transparent;"
            )


# ── JointCard ─────────────────────────────────────────────────────────────────

class JointCard(QFrame):
    """6-DOF eklemlerinden birini gösteren kart."""

    def __init__(self, jid: str, ros_name: str, display_name: str,
                 mn: float, mx: float, parent=None):
        super().__init__(parent)
        self._mn = mn
        self._mx = mx
        self.setMinimumHeight(90)
        self.setStyleSheet(
            f"JointCard{{background:{C['card2']};border:1px solid {C['b']};border-radius:7px;}}"
        )
        ly = QVBoxLayout(self)
        ly.setContentsMargins(10, 8, 10, 8)
        ly.setSpacing(4)

        # Başlık satırı
        hr = QHBoxLayout()
        left = QVBoxLayout(); left.setSpacing(0)
        self.id_lbl = make_label(jid, C["a"], 13, bold=True)
        self.nm_lbl = make_label(ros_name.upper(), C["t3"], 7, mono=True)
        left.addWidget(self.id_lbl)
        left.addWidget(self.nm_lbl)
        hr.addLayout(left); hr.addStretch()
        self.dot = QLabel(); self.dot.setFixedSize(8, 8); self._set_dot("ok")
        hr.addWidget(self.dot)
        ly.addLayout(hr)

        ly.addWidget(make_label(display_name, C["t2"], 9))

        # Pozisyon değeri + bar
        self.pos_lbl = make_label("0.0°", C["t1"], 14, bold=True)
        ly.addWidget(self.pos_lbl)
        self.pos_bar = QProgressBar()
        self.pos_bar.setRange(0, 1000); self.pos_bar.setValue(500)
        self.pos_bar.setTextVisible(False); self.pos_bar.setFixedHeight(4)
        self.pos_bar.setStyleSheet(
            f"QProgressBar{{background:rgba(255,255,255,0.05);border-radius:2px;border:none;}}"
            f"QProgressBar::chunk{{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            f"stop:0 {C['a']},stop:1 {C['a2']});border-radius:2px;}}"
        )
        ly.addWidget(self.pos_bar)

        # Hız ve tork
        row = QHBoxLayout()
        vl = QVBoxLayout(); vl.setSpacing(0)
        vl.addWidget(make_label("VEL", C["t3"], 7, mono=True))
        self.vel_val = make_label("0.00°/s", C["a2"], 9, bold=True)
        vl.addWidget(self.vel_val)
        el = QVBoxLayout(); el.setSpacing(0)
        el.addWidget(make_label("EFF", C["t3"], 7, mono=True))
        self.eff_val = make_label("0.00Nm", C["a3"], 9, bold=True)
        el.addWidget(self.eff_val)
        row.addLayout(vl); row.addLayout(el)
        ly.addLayout(row)

    def _set_dot(self, state: str):
        c = {"ok": C["ok"], "w": C["warn"], "e": C["red"]}.get(state, C["ok"])
        self.dot.setStyleSheet(f"background:{c};border-radius:4px;")

    def update_data(self, deg: float, vel: float, eff: float):
        self.pos_lbl.setText(f"{deg:.1f}°")
        pct = int((deg - self._mn) / (self._mx - self._mn) * 1000)
        self.pos_bar.setValue(max(0, min(1000, pct)))
        self.vel_val.setText(f"{vel:.2f}°/s")
        self.eff_val.setText(f"{eff:.2f}Nm")
        self._set_dot("w" if abs(vel) > 4 else "ok")


# ── TcpCell ───────────────────────────────────────────────────────────────────

class TcpCell(QFrame):
    """TCP Kartezyen pozisyonunun tek eksen hücresi."""

    def __init__(self, axis: str, color: str, unit: str = "mm", parent=None):
        super().__init__(parent)
        self._unit = unit
        self.setStyleSheet(
            f"TcpCell{{background:rgba(0,0,0,0.25);border:1px solid {C['b']};border-radius:5px;}}"
        )
        ly = QVBoxLayout(self)
        ly.setContentsMargins(8, 6, 8, 6)
        ly.setSpacing(2)
        ly.addWidget(make_label(axis, color, 9, mono=True))
        self.val = make_label(f"0.0 {unit}", C["t1"], 12, bold=True)
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000); self.bar.setValue(500)
        self.bar.setTextVisible(False); self.bar.setFixedHeight(3)
        self.bar.setStyleSheet(
            f"QProgressBar{{background:rgba(255,255,255,0.04);border-radius:1px;border:none;}}"
            f"QProgressBar::chunk{{background:{color};border-radius:1px;}}"
        )
        ly.addWidget(self.val)
        ly.addWidget(self.bar)

    def set_value(self, v: float, mn: float = -900, mx: float = 900):
        self.val.setText(f"{v:.1f} {self._unit}")
        pct = int((v - mn) / (mx - mn) * 1000)
        self.bar.setValue(max(0, min(1000, pct)))


# ── CmdButton ─────────────────────────────────────────────────────────────────

class CmdButton(QPushButton):
    """İkon + etiket içeren komut butonu."""

    def __init__(self, icon: str, label: str, kind: str = "normal", parent=None):
        super().__init__(parent)
        ly = QVBoxLayout(self)
        ly.setContentsMargins(4, 9, 4, 9)
        ly.setAlignment(Qt.AlignCenter)
        il = QLabel(icon); il.setAlignment(Qt.AlignCenter)
        il.setStyleSheet("font-size:16px;background:transparent;")
        ll = QLabel(label); ll.setAlignment(Qt.AlignCenter)
        ll.setStyleSheet(
            "font-size:8.5px;letter-spacing:0.15em;"
            "font-family:'Share Tech Mono';background:transparent;"
        )
        ly.addWidget(il); ly.addWidget(ll)
        fc, bg = {
            "start":  (C["a2"], "rgba(0,255,157,0.07)"),
            "stop":   (C["red"],"rgba(255,61,90,0.05)"),
            "normal": (C["t2"], C["card2"]),
        }.get(kind, (C["t2"], C["card2"]))
        self.setStyleSheet(
            f"CmdButton{{color:{fc};background:{bg};"
            f"border:1px solid {C['b']};border-radius:5px;min-height:50px;}}"
            f"CmdButton:hover{{border-color:{C['a']};color:{C['a']};"
            f"background:rgba(0,212,170,0.09);}}"
        )


# ── LogWidget ─────────────────────────────────────────────────────────────────

class LogWidget(QTextEdit):
    """Zaman damgalı, renkli sistem log ekranı."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setStyleSheet(
            f"QTextEdit{{background:#030810;border:1px solid {C['b']};border-radius:4px;"
            f"font-family:'Share Tech Mono';font-size:9px;color:{C['t2']};}}"
        )
        self._count = 0

    def add(self, msg: str, kind: str = "info") -> int: 
        col = {
            "ok":   C["ok"],
            "warn": C["warn"],
            "err":  C["red"],
            "info": C["a"],
        }.get(kind, C["a"])
        ts = datetime.now().strftime("%H:%M:%S")
        self._count += 1
        self.append(
            f'<span style="color:{C["t3"]}">{ts}</span> '
            f'<span style="color:{col}">{msg}</span>'
        )
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())
        return self._count
    