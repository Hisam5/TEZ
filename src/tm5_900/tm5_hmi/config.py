# ─────────────────────────────────────────────────────────────────────────────
#  config.py  —  Renk paleti, eklem tanımları, global stiller
# ─────────────────────────────────────────────────────────────────────────────

C = {
    "bg":   "#060b10", "card":  "#0d1825", "card2": "#0b1520",
    "b":    "#16293e", "bb":    "#1c3650",
    "a":    "#00d4aa", "a2":    "#00ff9d", "a3":    "#ff6b35",
    "t1":   "#c8dced", "t2":    "#4e7290", "t3":    "#273d52",
    "red":  "#ff3d5a", "warn":  "#ffb830", "ok":    "#00e676",
}

STYLE_BASE = f"""
QMainWindow, QWidget {{
    background:{C['bg']}; color:{C['t1']};
    font-family:'Exo 2','Rajdhani',sans-serif; font-size:11px;
}}
QScrollBar:vertical   {{ background:transparent; width:4px; border-radius:2px; }}
QScrollBar::handle:vertical {{ background:{C['bb']}; border-radius:2px; min-height:20px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0; }}
QScrollBar:horizontal {{ background:transparent; height:4px; }}
QScrollBar::handle:horizontal {{ background:{C['bb']}; border-radius:2px; }}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width:0; }}
"""

TM5_JOINTS = [
    {"id": "J1", "ros": "link1_joint", "nm": "Shoulder 1", "mn": -270, "mx":  270, "v":   0},
    {"id": "J2", "ros": "link2_joint", "nm": "Shoulder 2", "mn": -180, "mx":  180, "v":   0},
    {"id": "J3", "ros": "link3_joint", "nm": "Elbow",      "mn": -155, "mx":  155, "v":  90},
    {"id": "J4", "ros": "link4_joint", "nm": "Wrist 1",    "mn": -180, "mx":  180, "v":   0},
    {"id": "J5", "ros": "link5_joint", "nm": "Wrist 2",    "mn": -180, "mx":  180, "v":  90},
    {"id": "J6", "ros": "link6_joint", "nm": "Wrist 3",    "mn": -270, "mx":  270, "v":   0},
]