# -*- coding: utf-8 -*-
import sys

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from qwt import QwtPlot, QwtPlotCurve

from .layout import Ui_MainWindow

import rclpy
from rclpy.node import Node

from core_msgs.msg import CAN, CANArray

MODE_CURRENT = "Current"
MODE_SPEED = "Speed"
MODE_POSITION = "Position"
MODES = (MODE_CURRENT, MODE_SPEED, MODE_POSITION)


def map_range(value, from_low, from_high, to_low, to_high):
    if from_high == from_low:
        return to_low
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


def clamp(value, low, high):
    return max(low, min(high, value))


# ROS2の処理スレッド
class Worker(QObject):
    finished = pyqtSignal()
    robomas_rx = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.__id = 0
        self.__node = None
        self.__robomas_tx_pub = None

    def run(self):
        try:
            rclpy.init()
            self.__node = Node("motor_tool")
            self.__robomas_tx_pub = self.__node.create_publisher(CANArray, "can/tx", 10)
            self.__robomas_rx_sub = self.__node.create_subscription(CANArray, "can/rx", self.__robomas_rx_cb, 10)
            rclpy.spin(self.__node)
        except Exception:
            self.finished.emit()

    def stop(self):
        if self.__node is not None:
            self.__node.destroy_node()
            self.__node = None
        if rclpy.ok():
            rclpy.shutdown()

    def send(self, data):
        if self.__robomas_tx_pub is None:
            return
        can_array = CANArray()
        can = CAN()
        can.id = self.__id
        for value in data:
            can.data.append(float(value))
        can_array.array.append(can)
        self.__robomas_tx_pub.publish(can_array)

    def set_id(self, can_id: int):
        self.__id = can_id

    def __robomas_rx_cb(self, can_array):
        for can in can_array.array:
            if can.id == self.__id:
                self.robomas_rx.emit(list(can.data))
                break


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent=parent)
        ui = Ui_MainWindow()
        ui.setupUi(self)
        self.ui = ui

        # 状態
        self.send_mode = MODE_POSITION
        self.space_pressed = False
        self._syncing_widgets = False
        self.id_states = {}

        self.slider_by_mode = {
            MODE_CURRENT: ui.curr_ref_slider,
            MODE_SPEED: ui.vel_ref_slider,
            MODE_POSITION: ui.pos_ref_slider,
        }
        self.tx_box_by_mode = {
            MODE_CURRENT: ui.curr_ref_box,
            MODE_SPEED: ui.vel_ref_box,
            MODE_POSITION: ui.pos_ref_box,
        }
        self.rx_box_by_mode = {
            MODE_CURRENT: ui.curr_rx_target_box,
            MODE_SPEED: ui.vel_rx_target_box,
            MODE_POSITION: ui.pos_rx_target_box,
        }
        self.measured_box_by_mode = {
            MODE_CURRENT: ui.curr_state,
            MODE_SPEED: ui.vel_state,
            MODE_POSITION: ui.pos_state,
        }

        # layout.py 側の静的UIを使う
        self.mode_status_label = ui.mode_status_label
        self.tx_status_label = ui.tx_status_label

        # Target(/can/tx)は編集可能にする
        for edit in self.tx_box_by_mode.values():
            edit.setReadOnly(False)
            edit.setValidator(QDoubleValidator(edit))

        # Spaceショートカット衝突を無効化
        ui.stop_btn.setShortcut(QKeySequence())
        ui.gain_btn.setShortcut(QKeySequence())

        # 100Hz再送タイマ
        self.send_timer = QTimer(self)
        self.send_timer.setInterval(10)
        self.send_timer.timeout.connect(self.periodic_send_cb)

        # ID
        ui.can_id.valueChanged.connect(self.can_id_cb)

        # スライダー
        ui.pos_ref_slider.valueChanged.connect(self.pos_ref_slider_cb)
        ui.vel_ref_slider.valueChanged.connect(self.vel_ref_slider_cb)
        ui.curr_ref_slider.valueChanged.connect(self.curr_ref_slider_cb)

        # Target(/can/tx) 即時更新
        ui.pos_ref_box.textEdited.connect(self.pos_ref_text_edited_cb)
        ui.vel_ref_box.textEdited.connect(self.vel_ref_text_edited_cb)
        ui.curr_ref_box.textEdited.connect(self.curr_ref_text_edited_cb)

        # ボタン
        ui.stop_btn.clicked.connect(self.stop_btn_cb)
        ui.gain_btn.clicked.connect(self.gain_btn_cb)

        # プロット
        self.plot_data_len = 1000
        self.data = [[0.0 for _ in range(self.plot_data_len)] for _ in range(6)]
        self.t = [i for i in range(self.plot_data_len)]
        self.dots = []
        self.plot = [ui.curr_plot, ui.vel_plot, ui.pos_plot]

        for i in range(3):
            input_dot = QwtPlotCurve("Dots")
            input_dot.attach(self.plot[i])
            input_dot.setPen(QPen(Qt.red, 3, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            self.dots.append(input_dot)
            state_dot = QwtPlotCurve("Dots")
            state_dot.attach(self.plot[i])
            state_dot.setPen(QPen(Qt.black, 3, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            self.dots.append(state_dot)

        self.plot[0].setAxisScale(QwtPlot.yLeft, -3, 3.0)
        self.plot[1].setAxisScale(QwtPlot.yLeft, -10, 10.0)
        self.plot[2].setAxisAutoScale(QwtPlot.yLeft, True)

        self.refresh_status_labels()

        # Space押下をウィンドウ全体で拾う
        app = QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

        # ROS2 SubscribeをThread処理
        self.thread = QThread()
        self.worker = Worker()
        self.worker.moveToThread(self.thread)
        self.worker.robomas_rx.connect(self.robomas_rx_cb)
        self.worker.finished.connect(self.close)
        self.thread.started.connect(self.worker.run)
        self.thread.start()

        # 初期ID状態
        self.can_id_cb(self.ui.can_id.value())

    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and event.key() == Qt.Key_Space and self.isActiveWindow():
            if not event.isAutoRepeat() and not self.space_pressed:
                self.space_pressed = True
                self.start_periodic_send()
            return True
        if event.type() == QEvent.KeyRelease and event.key() == Qt.Key_Space and self.isActiveWindow():
            if not event.isAutoRepeat():
                self.space_pressed = False
                self.stop_periodic_send()
            return True
        return super().eventFilter(obj, event)

    def changeEvent(self, event):
        if event.type() == QEvent.ActivationChange and not self.isActiveWindow():
            if self.space_pressed:
                self.space_pressed = False
                self.stop_periodic_send()
        super().changeEvent(event)

    def closeEvent(self, event):
        self.stop_periodic_send()
        app = QApplication.instance()
        if app is not None:
            app.removeEventFilter(self)
        if hasattr(self, "worker"):
            self.worker.stop()
        if hasattr(self, "thread"):
            self.thread.quit()
            self.thread.wait(500)
        super().closeEvent(event)

    def refresh_status_labels(self):
        self.mode_status_label.setText(f"Mode: {self.send_mode}")
        tx_state = "Enabled" if self.send_timer.isActive() else "Disabled"
        self.tx_status_label.setText(f"100Hz: {tx_state} (Space)")

    def start_periodic_send(self):
        if not self.send_timer.isActive():
            self.send_timer.start()
        self.refresh_status_labels()

    def stop_periodic_send(self):
        if self.send_timer.isActive():
            self.send_timer.stop()
        self.refresh_status_labels()

    def mode_limits(self, mode):
        if mode == MODE_CURRENT:
            max_ref = self.ui.curr_limit_max.value()
            return -max_ref, max_ref
        if mode == MODE_SPEED:
            max_ref = self.ui.vel_limit_max.value()
            return -max_ref, max_ref
        return self.ui.pos_limit_min.value(), self.ui.pos_limit_max.value()

    def slider_to_target(self, mode, slider_value):
        min_ref, max_ref = self.mode_limits(mode)
        return map_range(float(slider_value), -100.0, 100.0, min_ref, max_ref)

    def target_to_slider(self, mode, target_value):
        min_ref, max_ref = self.mode_limits(mode)
        slider_value = map_range(float(target_value), min_ref, max_ref, -100.0, 100.0)
        return int(round(clamp(slider_value, -100.0, 100.0)))

    def clamp_target(self, mode, target_value):
        min_ref, max_ref = self.mode_limits(mode)
        return clamp(float(target_value), min_ref, max_ref)

    def set_mode(self, mode):
        self.send_mode = mode
        self.refresh_status_labels()

    def new_id_state(self):
        return {
            "tx_target": {
                MODE_CURRENT: 0.0,
                MODE_SPEED: 0.0,
                MODE_POSITION: 0.0,
            },
            "rx_target": {
                MODE_CURRENT: 0.0,
                MODE_SPEED: 0.0,
                MODE_POSITION: 0.0,
            },
            "rx_measured": {
                MODE_CURRENT: 0.0,
                MODE_SPEED: 0.0,
                MODE_POSITION: 0.0,
            },
            "initialized_from_rx": False,
        }

    def ensure_id_state(self, can_id):
        if can_id not in self.id_states:
            self.id_states[can_id] = self.new_id_state()
        return self.id_states[can_id]

    def current_state(self):
        return self.ensure_id_state(int(self.ui.can_id.value()))

    def apply_state_to_ui(self, state, sync_sliders):
        self._syncing_widgets = True
        try:
            for mode in MODES:
                self.tx_box_by_mode[mode].setText(f"{state['tx_target'][mode]:.6f}")
                self.rx_box_by_mode[mode].setText(f"{state['rx_target'][mode]:.6f}")
                self.measured_box_by_mode[mode].setText(f"{state['rx_measured'][mode]:.3f}")
                if sync_sliders:
                    self.slider_by_mode[mode].setValue(self.target_to_slider(mode, state["tx_target"][mode]))
        finally:
            self._syncing_widgets = False

    def update_rx_measured_ui(self, state):
        for mode in MODES:
            self.rx_box_by_mode[mode].setText(f"{state['rx_target'][mode]:.6f}")
            self.measured_box_by_mode[mode].setText(f"{state['rx_measured'][mode]:.3f}")

    def handle_slider_change(self, mode, value):
        if self._syncing_widgets:
            return
        state = self.current_state()
        target = self.clamp_target(mode, self.slider_to_target(mode, value))
        state["tx_target"][mode] = target
        self.tx_box_by_mode[mode].setText(f"{target:.6f}")
        self.set_mode(mode)

    def handle_target_text_edited(self, mode, text):
        if self._syncing_widgets:
            return
        text = text.strip()
        if text in ("", "-", "+", ".", "-.", "+."):
            return
        state = self.current_state()
        try:
            target = float(text)
        except ValueError:
            return

        target = self.clamp_target(mode, target)
        state["tx_target"][mode] = target

        self._syncing_widgets = True
        try:
            self.slider_by_mode[mode].setValue(self.target_to_slider(mode, target))
        finally:
            self._syncing_widgets = False

        self.set_mode(mode)

    def build_payload_for_mode(self, mode):
        state = self.current_state()
        tx_target = state["tx_target"]
        if mode == MODE_CURRENT:
            return [tx_target[MODE_CURRENT]]
        if mode == MODE_SPEED:
            curr_max = self.ui.curr_limit_max.value()
            return [curr_max, tx_target[MODE_SPEED]]
        curr_max = self.ui.curr_limit_max.value()
        vel_max = self.ui.vel_limit_max.value()
        return [curr_max, vel_max, tx_target[MODE_POSITION]]

    def periodic_send_cb(self):
        if not self.space_pressed:
            self.stop_periodic_send()
            return
        self.worker.send(self.build_payload_for_mode(self.send_mode))

    def can_id_cb(self, value):
        self.worker.set_id(value)
        state = self.ensure_id_state(int(value))
        self.apply_state_to_ui(state, sync_sliders=True)

    def pos_ref_slider_cb(self, value):
        self.handle_slider_change(MODE_POSITION, value)

    def vel_ref_slider_cb(self, value):
        self.handle_slider_change(MODE_SPEED, value)

    def curr_ref_slider_cb(self, value):
        self.handle_slider_change(MODE_CURRENT, value)

    def pos_ref_text_edited_cb(self, text):
        self.handle_target_text_edited(MODE_POSITION, text)

    def vel_ref_text_edited_cb(self, text):
        self.handle_target_text_edited(MODE_SPEED, text)

    def curr_ref_text_edited_cb(self, text):
        self.handle_target_text_edited(MODE_CURRENT, text)

    def stop_btn_cb(self):
        state = self.current_state()
        state["tx_target"][MODE_CURRENT] = 0.0
        state["tx_target"][MODE_SPEED] = 0.0

        self._syncing_widgets = True
        try:
            self.slider_by_mode[MODE_CURRENT].setValue(self.target_to_slider(MODE_CURRENT, 0.0))
            self.slider_by_mode[MODE_SPEED].setValue(self.target_to_slider(MODE_SPEED, 0.0))
            self.tx_box_by_mode[MODE_CURRENT].setText("0.000000")
            self.tx_box_by_mode[MODE_SPEED].setText("0.000000")
        finally:
            self._syncing_widgets = False

        self.worker.send([])

    def gain_btn_cb(self):
        vel_p = self.ui.vel_gain_p.value()
        vel_i = self.ui.vel_gain_i.value()
        pos_p = self.ui.pos_gain_p.value()
        pos_i = 0.0
        self.worker.send([vel_p, vel_i, pos_p, pos_i])

    def robomas_rx_cb(self, data):
        if len(data) != 6:
            return

        state = self.current_state()
        state["rx_target"][MODE_CURRENT] = float(data[0])
        state["rx_measured"][MODE_CURRENT] = float(data[1])
        state["rx_target"][MODE_SPEED] = float(data[2])
        state["rx_measured"][MODE_SPEED] = float(data[3])
        state["rx_target"][MODE_POSITION] = float(data[4])
        state["rx_measured"][MODE_POSITION] = float(data[5])

        if not state["initialized_from_rx"]:
            for mode in MODES:
                target = self.clamp_target(mode, state["rx_target"][mode])
                state["tx_target"][mode] = target
            state["initialized_from_rx"] = True
            self.apply_state_to_ui(state, sync_sliders=True)
        else:
            self.update_rx_measured_ui(state)

        for i in range(6):
            sample = float(data[i])
            self.data[i].pop(0)
            self.data[i].append(sample)
            self.dots[i].setData(self.t, self.data[i])
        for i in range(3):
            self.plot[i].replot()


def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
    sys.exit()


if __name__ == "__main__":
    main()
