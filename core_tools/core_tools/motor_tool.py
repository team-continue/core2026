# -*- coding: utf-8 -*-
import sys
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from qwt import QwtPlotCurve, QwtPlot
#.uiファイルから変換後のファイルから読み込む
from .layout import Ui_MainWindow
import os

#ROS
import rclpy
from rclpy.node import Node

from core_msgs.msg import CANArray, CAN

dir = os.path.dirname(__file__)

def map(value, fromLow, fromHigh, toLow, toHigh):
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow

# ROS2の処理スレッド    
class Worker(QObject):
    finished = pyqtSignal()
    robomas_rx = pyqtSignal(list)

    def run(self):
        try:
            rclpy.init()
            self.__node = Node('motor_tool')
            self.__robomas_tx_pub = self.__node.create_publisher(CANArray, 'can/tx', 10)
            self.__robomas_rx_sub = self.__node.create_subscription(CANArray,'can/rx',self.__robomas_rx_cb, 10)
            self.__id = 0
            rclpy.spin(self.__node)
        except Exception:
            self.finished.emit()
        
    def stop(self):
        self.__node.destroy_node()
        rclpy.shutdown()
    
    def send(self, data):
        can_array = CANArray()
        can = CAN()
        can.id = self.__id
        for i in data:
            can.data.append(i)
        can_array.array.append(can)
        self.__robomas_tx_pub.publish(can_array)
        
    def set_id(self, id:int):
        self.__id = id
    
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

        #uiをクラス内からアクセスできるようにする
        self.ui = ui

        # ID
        ui.can_id.valueChanged.connect(self.can_id_cb)
        
        #スライダー
        ui.pos_ref_slider.sliderMoved.connect(self.pos_ref_slider_cb)
        ui.vel_ref_slider.sliderMoved.connect(self.vel_ref_slider_cb)
        ui.curr_ref_slider.sliderMoved.connect(self.curr_ref_slider_cb)

        # ボタン
        ui.stop_btn.clicked.connect(self.stop_btn_cb)
        ui.gain_btn.clicked.connect(self.gain_btn_cb)

        #プロット
        self.plot_data_len = 1000
        self.data = [[0. for x in range(self.plot_data_len)] for y in range(6)] # 縦軸×６
        self.t = [i for i in range(self.plot_data_len)] # 横軸
        self.dots = []
        self.plot = [ui.curr_plot, ui.vel_plot, ui.pos_plot] # プロットエリア
        self.debug = [ui.curr_state, ui.vel_state, ui.pos_state] # テキストボックス
        self.ref =  [ui.curr_ref_box, ui.vel_ref_box, ui.pos_ref_box] # テキストボックス

        # グラフの設定
        for i in range(3):
            input_dot = QwtPlotCurve("Dots")
            input_dot.attach(self.plot[i])
            input_dot.setPen(QPen(Qt.red, 3, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            self.dots.append(input_dot)
            state_dot = QwtPlotCurve("Dots")
            state_dot.attach(self.plot[i])
            state_dot.setPen(QPen(Qt.black, 3, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            self.dots.append(state_dot)
        
        #グラフの最大値
        self.plot[0].setAxisScale(QwtPlot.yLeft, -3, 3.) # 電流
        self.plot[1].setAxisScale(QwtPlot.yLeft, -10, 10.) # 速度
        self.plot[2].setAxisAutoScale(QwtPlot.yLeft, True) # 位置
        #self.plot[2].setAxisScale(QwrPloat.yLeft, -10., 10.)
        

        #ROS2 SubscribeをThread処理
        self.thread = QThread()
        self.worker = Worker()
        self.worker.moveToThread(self.thread)
        self.worker.robomas_rx.connect(self.robomas_rx_cb)
        self.worker.finished.connect(self.close)    
        self.thread.started.connect(self.worker.run)
        self.thread.start()
        
    def can_id_cb(self, value):
        self.worker.set_id(value)

    def pos_ref_slider_cb(self, value):
        min_ref = self.ui.pos_limit_min.value()
        max_ref = self.ui.pos_limit_max.value()
        ref = map(value, -100., 100., min_ref, max_ref)
        self.ui.pos_ref_box.setText(f'{ref:.6f}')
        vel_max = self.ui.vel_limit_max.value()
        curr_max = self.ui.curr_limit_max.value()
        self.worker.send([curr_max,vel_max,ref])

    def vel_ref_slider_cb(self, value):
        min_ref = -self.ui.vel_limit_max.value()
        max_ref = self.ui.vel_limit_max.value()
        ref = map(value, -100., 100., min_ref, max_ref)
        self.ui.vel_ref_box.setText(f'{ref:.6f}')
        curr_max = self.ui.curr_limit_max.value()
        self.worker.send([curr_max,ref])


    def curr_ref_slider_cb(self, value):
        min_ref = -self.ui.curr_limit_max.value()
        max_ref = self.ui.curr_limit_max.value()
        ref = map(value, -100., 100., min_ref, max_ref)
        self.ui.curr_ref_box.setText(f'{ref:.6f}')
        self.worker.send([ref])

    def stop_btn_cb(self):
        self.ui.vel_ref_slider.setValue(0)
        self.ui.curr_ref_slider.setValue(0)
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
        for i in range(6):
            state = data[i] # モーターの情報 [ID, 電流入力, 測定電流, 速度入力, 測定速度, 位置入力, 測定位置]0,1,2,3,4,5,6
            self.data[i].pop(0) # 縦軸の先頭を削除
            self.data[i].append(state) # 縦軸の最後尾に追加
            self.dots[i].setData(self.t, self.data[i]) # グラフのプロットするデータに追加
        for i in range(3):
            state = data[i*2+1] # モーターの情報 [ID, 電流入力, 測定電流, 速度入力, 測定速度, 位置入力, 測定位置]2,4,6
            self.plot[i].replot() # プロット
            self.debug[i].setText(f'{state:.3}') # 測定値テキストボックスに表示
        for i in range(3):
            state = data[i*2] # モーターの情報 [ID, 電流入力, 測定電流, 速度入力, 測定速度, 位置入力, 測定位置]2,4,6
            self.ref[i].setText(f'{state:.3}') # 測定値テキストボックスに表示

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
    sys.exit()

if __name__ == '__main__':
    main()
