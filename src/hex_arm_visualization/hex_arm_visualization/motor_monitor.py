"""Standalone six-joint Qt table; all network work stays off the GUI thread."""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QApplication, QHeaderView, QLabel, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from hex_arm_tools.pb import robot_api_pb2 as pb
from hex_arm_visualization.motor_monitor_client import DOF, MonitorCache, MonitorClient


HEADERS = (
    "关节", "CAN ID", "位置 (rad)", "速度 (rad/s)", "反馈估计力矩 (N·m)",
    "电机温度 (°C)", "驱动板温度 (°C)", "在线", "反馈", "故障码",
)
MODE_NAMES = {
    pb.OPERATING_MODE_DISABLED: "失能",
    pb.OPERATING_MODE_ACTIVE: "位置控制",
    pb.OPERATING_MODE_PASSIVE: "被动",
    pb.OPERATING_MODE_GRAVITY_COMP: "重力补偿",
    pb.OPERATING_MODE_FAULT: "故障",
    pb.OPERATING_MODE_CALIBRATING: "校准中",
}


def number(value, digits=3):
    return f"{value:.{digits}f}" if math.isfinite(value) else "—"


def stale(received_at, now, timeout):
    return received_at is None or now - received_at > timeout


class MonitorWindow(QWidget):
    def __init__(self, cache, prefix, endpoint):
        super().__init__()
        self.cache = cache
        self.setWindowTitle(f"电机状态监控 — {prefix}")
        self.resize(1240, 360)
        layout = QVBoxLayout(self)
        device = QLabel(f"设备：{prefix}    连接：{endpoint}")
        device.setTextFormat(Qt.PlainText)
        layout.addWidget(device)
        self.status = QLabel()
        self.status.setTextFormat(Qt.PlainText)
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.table = QTableWidget(DOF, len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        for row in range(DOF):
            for column in range(len(HEADERS)):
                self.table.setItem(row, column, QTableWidgetItem("—"))
        self.table.setToolTip("力矩为电机反馈换算的关节侧估计值。灰色数值为最近一次收到的反馈。")
        layout.addWidget(self.table)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.refresh()

    def refresh(self, now=None):
        now = time.monotonic() if now is None else now
        snapshot = self.cache.snapshot()
        joint, driver = snapshot.joint, snapshot.driver
        joint_stale = stale(snapshot.joint_at, now, self.cache.stale_timeout)
        driver_stale = stale(snapshot.driver_at, now, self.cache.stale_timeout)
        diagnostics = joint.motor_diagnostics if joint is not None else ()
        available = len(diagnostics) == DOF
        status = ["等待关节状态" if joint is None else
                  ("状态流中断" if joint_stale else "已连接")]
        if driver is None:
            status.append("等待驱动状态")
        elif driver_stale:
            status.append("驱动状态已过期")
        else:
            status.append(f"模式：{MODE_NAMES.get(driver.mode, str(driver.mode))}")
            if math.isfinite(driver.feedback_age_s):
                status.append(f"CAN 反馈年龄：{driver.feedback_age_s * 1000:.0f} ms")
            if not driver.feedback_fresh:
                status.append("整机反馈未就绪")
            if driver.fault_latched:
                status.append(f"整机故障 0x{driver.fault_code:04X}：{driver.fault_reason}")
        if joint is not None and not available:
            status.append("驱动未提供完整逐电机诊断，请更新驱动至 API 0.5+")
        if not snapshot.names:
            status.append("等待关节名称")
        if snapshot.error:
            status.append(snapshot.error)
        self.status.setText("    |    ".join(status))

        names = snapshot.names or tuple(f"J{i + 1}" for i in range(DOF))
        for row in range(DOF):
            motor = diagnostics[row] if available else None
            fault = motor is not None and motor.HasField("fault_code")
            values = [names[row], str(motor.node_id) if motor is not None else "—"]
            values.extend((number(joint.q[row]), number(joint.dq[row]), number(joint.tau_est[row]))
                          if joint is not None else ("—", "—", "—"))
            values.extend((number(motor.motor_temperature_c, 1),
                           number(motor.driver_temperature_c, 1)) if motor is not None else ("—", "—"))
            values.extend((
                "未知" if joint_stale else ("在线" if motor.online else "离线"),
                "状态流中断" if joint_stale else ("正常" if motor.feedback_fresh else "过期"),
                f"0x{motor.fault_code:04X}" if fault else "无",
            ) if motor is not None else ("—", "—", "—"))
            inactive = joint_stale or (motor is not None and (not motor.online or not motor.feedback_fresh))
            color = self.palette().text().color()
            if inactive:
                color = QColor("#707070")
            if fault:
                color = QColor("#b00020")
            for column, value in enumerate(values):
                item = self.table.item(row, column)
                item.setText(value)
                item.setForeground(color)


def positive_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def robot_prefix(value):
    value = value.strip().strip("/")
    if not value or "*" in value or any(char.isspace() for char in value):
        raise argparse.ArgumentTypeError("must identify one robot without wildcards or whitespace")
    return value


def main(args=None):
    parser = argparse.ArgumentParser(description="Read-only motor telemetry window (Zenoh, API 0.5+).")
    parser.add_argument("--robot-prefix", type=robot_prefix, required=True)
    parser.add_argument("--zenoh-connect", default="tcp/127.0.0.1:7448")
    parser.add_argument("--stale-timeout-sec", type=positive_seconds, default=0.5)
    options = parser.parse_args(args)
    app = QApplication([sys.argv[0]])
    cache = MonitorCache(options.stale_timeout_sec)
    client = MonitorClient(cache, options.robot_prefix, options.zenoh_connect)
    window = MonitorWindow(cache, options.robot_prefix, options.zenoh_connect)
    previous = {signum: signal.signal(signum, lambda *_: app.quit())
                for signum in (signal.SIGINT, signal.SIGTERM)}
    app.aboutToQuit.connect(window.timer.stop)
    try:
        window.show()
        client.start()
        return app.exec_()
    finally:
        client.close()
        window.close()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    sys.exit(main())
