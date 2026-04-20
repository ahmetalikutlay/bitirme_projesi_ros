#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GRADUATION PROJECT - GROUP 1
Istanbul Aydin University - Industrial Engineering
Dashboard / Monitoring UI

Developer: Levent Arda Padar (B2180.060064)
"""

import sys
import csv
import os
import threading
from datetime import datetime

try:
    import rospy
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTableWidget, QTableWidgetItem, QGroupBox, QProgressBar,
    QFrame, QHeaderView, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette

KPI_FILE = '/tmp/kpi_log.csv'

WASTE_COLORS = {
    'RED':   ('#E74C3C', 'Plastik'),
    'BLUE':  ('#2980B9', 'Cam'),
    'GREEN': ('#27AE60', 'Kağıt'),
    'GRAY':  ('#7F8C8D', 'Metal'),
    'red':   ('#E74C3C', 'Plastik'),
    'green': ('#27AE60', 'Kağıt'),
    'yellow':('#F39C12', 'Plastik'),
}

STATUS_COLORS = {
    'HAZIR':      '#27AE60',
    'TESPIT':     '#F39C12',
    'CALISYOR':   '#E67E22',
    'TAMAMLANDI': '#2ECC71',
    'HATA':       '#E74C3C',
}


class ROSBridge(QObject):
    status_signal = pyqtSignal(str)
    detection_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = False

    def start(self):
        if not ROS_AVAILABLE:
            return
        self._running = True
        t = threading.Thread(target=self._ros_thread, daemon=True)
        t.start()

    def _ros_thread(self):
        try:
            rospy.init_node('dashboard_node', anonymous=True, disable_signals=True)
            rospy.Subscriber('/robot_status', String, self._status_cb)
            rospy.Subscriber('/detected_color', String, self._detection_cb)
            rospy.spin()
        except Exception:
            pass

    def _status_cb(self, msg):
        self.status_signal.emit(msg.data)

    def _detection_cb(self, msg):
        self.detection_signal.emit(msg.data)


class StatusCard(QFrame):
    def __init__(self, title, value, color='#2C3E50', parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: #1E2833;
                border-radius: 8px;
                border: 1px solid #2C3E50;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)

        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("color: #95A5A6; font-size: 11px; font-weight: bold;")
        self.title_label.setAlignment(Qt.AlignCenter)

        self.value_label = QLabel(value)
        self.value_label.setStyleSheet(f"color: {color}; font-size: 28px; font-weight: bold;")
        self.value_label.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)

    def update_value(self, value, color=None):
        self.value_label.setText(str(value))
        if color:
            self.value_label.setStyleSheet(f"color: {color}; font-size: 28px; font-weight: bold;")


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Autonomous Waste Sorting — Monitoring Dashboard")
        self.setMinimumSize(1100, 700)
        self.setStyleSheet("""
            QMainWindow { background-color: #0F1923; }
            QWidget { background-color: #0F1923; color: #ECF0F1; font-family: Arial; }
            QGroupBox {
                border: 1px solid #2C3E50;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
                color: #BDC3C7;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QTableWidget {
                background-color: #1A2535;
                border: none;
                gridline-color: #2C3E50;
                color: #ECF0F1;
                font-size: 12px;
            }
            QTableWidget::item { padding: 6px; }
            QTableWidget::item:selected { background-color: #2C3E50; }
            QHeaderView::section {
                background-color: #1E2D3D;
                color: #BDC3C7;
                padding: 6px;
                border: none;
                font-weight: bold;
                font-size: 11px;
            }
            QProgressBar {
                border: 1px solid #2C3E50;
                border-radius: 4px;
                background-color: #1A2535;
                height: 18px;
                text-align: center;
                color: white;
                font-size: 11px;
            }
            QProgressBar::chunk { background-color: #27AE60; border-radius: 3px; }
            QLabel { color: #ECF0F1; }
        """)

        self.trial_data = []
        self.total_trials = 0
        self.successful_trials = 0
        self.cycle_times = []

        self._build_ui()

        self.ros_bridge = ROSBridge()
        self.ros_bridge.status_signal.connect(self._on_status)
        self.ros_bridge.detection_signal.connect(self._on_detection)
        self.ros_bridge.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh_from_csv)
        self.timer.start(2000)

        self._refresh_from_csv()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(16, 16, 16, 16)
        main_layout.setSpacing(12)

        # ── HEADER ──
        header = QLabel("🤖  Autonomous Waste Sorting Robot — Live Dashboard")
        header.setStyleSheet("font-size: 18px; font-weight: bold; color: #3498DB; padding: 8px 0;")
        header.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(header)

        # ── STATUS BAR ──
        status_frame = QFrame()
        status_frame.setStyleSheet("background-color: #1E2833; border-radius: 8px; border: 1px solid #2C3E50;")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(16, 10, 16, 10)

        self.status_dot = QLabel("●")
        self.status_dot.setStyleSheet("color: #27AE60; font-size: 20px;")
        self.status_label = QLabel("SİSTEM HAZIR")
        self.status_label.setStyleSheet("color: #27AE60; font-size: 14px; font-weight: bold;")

        self.detection_label = QLabel("Son Tespit: —")
        self.detection_label.setStyleSheet("color: #BDC3C7; font-size: 13px;")
        self.detection_label.setAlignment(Qt.AlignRight)

        self.time_label = QLabel("")
        self.time_label.setStyleSheet("color: #7F8C8D; font-size: 11px;")
        self.time_label.setAlignment(Qt.AlignRight)

        status_layout.addWidget(self.status_dot)
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        status_layout.addWidget(self.detection_label)
        status_layout.addWidget(self.time_label)
        main_layout.addWidget(status_frame)

        # ── KPI CARDS ──
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(10)

        self.card_trials = StatusCard("TOPLAM TRIAL", "0", "#3498DB")
        self.card_success = StatusCard("BAŞARILI", "0", "#27AE60")
        self.card_accuracy = StatusCard("ACCURACY", "—", "#F39C12")
        self.card_cycle = StatusCard("ORT. CYCLE TIME", "—", "#9B59B6")
        self.card_grasp = StatusCard("GRASP BAŞARI", "—", "#1ABC9C")

        for card in [self.card_trials, self.card_success, self.card_accuracy,
                     self.card_cycle, self.card_grasp]:
            cards_layout.addWidget(card)
        main_layout.addLayout(cards_layout)

        # ── BOTTOM: Progress + Table ──
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(12)

        # Sol — Atık sınıfı dağılımı
        left_group = QGroupBox("Atık Sınıfı Dağılımı")
        left_layout = QVBoxLayout(left_group)
        left_layout.setSpacing(8)

        self.progress_bars = {}
        waste_types = [
            ('RED',   'Plastik (Kırmızı)', '#E74C3C'),
            ('BLUE',  'Cam (Mavi)',        '#2980B9'),
            ('GREEN', 'Kağıt (Yeşil)',     '#27AE60'),
            ('GRAY',  'Metal (Gri)',        '#7F8C8D'),
        ]
        for key, label, color in waste_types:
            row = QHBoxLayout()
            lbl = QLabel(label)
            lbl.setFixedWidth(150)
            lbl.setStyleSheet(f"color: {color}; font-size: 12px;")
            pb = QProgressBar()
            pb.setRange(0, 100)
            pb.setValue(0)
            pb.setStyleSheet(f"""
                QProgressBar::chunk {{ background-color: {color}; border-radius: 3px; }}
            """)
            count_lbl = QLabel("0")
            count_lbl.setFixedWidth(30)
            count_lbl.setAlignment(Qt.AlignRight)
            count_lbl.setStyleSheet("color: #BDC3C7; font-size: 12px;")
            row.addWidget(lbl)
            row.addWidget(pb)
            row.addWidget(count_lbl)
            left_layout.addLayout(row)
            self.progress_bars[key] = (pb, count_lbl)

        left_layout.addStretch()
        bottom_layout.addWidget(left_group, 1)

        # Sağ — Trial tablosu
        right_group = QGroupBox("Son Triallar")
        right_layout = QVBoxLayout(right_group)

        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Trial", "Atık Tipi", "Cycle Time (s)", "Grasp", "Durum"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet("""
            QTableWidget { alternate-background-color: #1A2535; }
        """)
        right_layout.addWidget(self.table)
        bottom_layout.addWidget(right_group, 2)

        main_layout.addLayout(bottom_layout)

        # Timer for clock
        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()

    def _update_clock(self):
        self.time_label.setText(datetime.now().strftime("%H:%M:%S  |  %d.%m.%Y"))

    def _on_status(self, status):
        color = STATUS_COLORS.get(status, '#BDC3C7')
        self.status_label.setText(status)
        self.status_label.setStyleSheet(f"color: {color}; font-size: 14px; font-weight: bold;")
        self.status_dot.setStyleSheet(f"color: {color}; font-size: 20px;")

    def _on_detection(self, color):
        info = WASTE_COLORS.get(color, ('#BDC3C7', color))
        c, name = info
        self.detection_label.setText(f"Son Tespit: {name}")
        self.detection_label.setStyleSheet(f"color: {c}; font-size: 13px; font-weight: bold;")

    def _refresh_from_csv(self):
        if not os.path.exists(KPI_FILE):
            return

        try:
            with open(KPI_FILE, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
        except Exception:
            return

        if not rows:
            return

        self.trial_data = rows
        total = len(rows)
        successful = sum(1 for r in rows if r.get('grasp_basarili', '').lower() in ('true', '1'))
        times = []
        for r in rows:
            try:
                times.append(float(r.get('cycle_time_s', 0)))
            except Exception:
                pass

        accuracy = round(successful / total * 100, 1) if total > 0 else 0
        avg_time = round(sum(times) / len(times), 2) if times else 0

        # KPI kartları
        self.card_trials.update_value(total, '#3498DB')
        self.card_success.update_value(successful, '#27AE60')
        self.card_accuracy.update_value(f"%{accuracy}", '#F39C12' if accuracy < 90 else '#27AE60')
        self.card_cycle.update_value(f"{avg_time}s", '#9B59B6' if avg_time < 2 else '#E74C3C')
        grasp_rate = round(successful / total * 100, 1) if total > 0 else 0
        self.card_grasp.update_value(f"%{grasp_rate}", '#1ABC9C')

        # Progress bars
        counts = {'RED': 0, 'BLUE': 0, 'GREEN': 0, 'GRAY': 0}
        for r in rows:
            t = r.get('atik_tipi', '').upper()
            if t in counts:
                counts[t] += 1
        for key, (pb, lbl) in self.progress_bars.items():
            count = counts.get(key, 0)
            pct = int(count / total * 100) if total > 0 else 0
            pb.setValue(pct)
            lbl.setText(str(count))

        # Tablo — son 15 trial
        recent = rows[-15:][::-1]
        self.table.setRowCount(len(recent))
        for i, r in enumerate(recent):
            trial_no = r.get('trial_no', '')
            atik = r.get('atik_tipi', '')
            cycle = r.get('cycle_time_s', '')
            grasp = r.get('grasp_basarili', '')
            durum = "✅" if grasp.lower() in ('true', '1') else "❌"

            color_info = WASTE_COLORS.get(atik.upper(), ('#BDC3C7', atik))
            hex_color = color_info[0]
            name = color_info[1]

            items = [
                QTableWidgetItem(str(trial_no)),
                QTableWidgetItem(f"{name} ({atik})"),
                QTableWidgetItem(str(cycle)),
                QTableWidgetItem(durum),
                QTableWidgetItem("Başarılı" if grasp.lower() in ('true', '1') else "Başarısız"),
            ]
            for j, item in enumerate(items):
                item.setTextAlignment(Qt.AlignCenter)
                if j == 1:
                    item.setForeground(QColor(hex_color))
                if durum == "❌" and j == 4:
                    item.setForeground(QColor('#E74C3C'))
                self.table.setItem(i, j, item)


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(15, 25, 35))
    palette.setColor(QPalette.WindowText, QColor(236, 240, 241))
    app.setPalette(palette)

    window = Dashboard()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
