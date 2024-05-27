# !/usr/bin/env python
# -*-coding:utf-8 -*-
"""
# File       : parse_pcpng.py
# Time       : 2024/2/28 11:04
# Author     :houxiaowei  liqianwen
# Software   : PyCharm
# Description:
"""
from PyQt5.QtWidgets import QDialog
from PyQt5 import QtWidgets, QtCore
import UI.ui_Dialog_wireshark

import os
import pyshark
from pathlib import Path
import time
# import tqdm  # 显示进程的玩意
from threading import Thread  # 多线程


# 绘图设置
class DialogWireshark(QDialog, UI.ui_Dialog_wireshark.Ui_Dialog):
    # my_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super(DialogWireshark, self).__init__()
        self.setupUi(self)
        self.pushButton_selectFile.clicked.connect(self.selectFiles)
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下
        self.messages = ''
        # self.textBrowser_printLog = textBrowser_printLog

    def selectFiles(self):
        filePath, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "Feather Files(*.pcapng)")  # 打开文件对话框，选择文件
        self.lineEdit_FilePath.setText(filePath)

    def on_accepted(self):
        if not self.lineEdit_FilePath.text():
            self.messages = '并未填写文件路径'
            return
        if not self.lineEdit_hq.text():
            self.messages = '并未填写port value'
            return
        if not self.lineEdit_FilePath.text():
            self.messages = '并未填写port type'
            return
        else:
            try:
                self.parse_pcpng(self.lineEdit_FilePath.text()
                                 , port=int(self.lineEdit_hq.text())
                                 , port_info=self.lineEdit_port_type.text())
            except Exception as e:
                self.output_msg(f"发生 Exception 错误: {e}")

    def parse_pcpng(self, path, port=12302, port_info='udp.srcport'):

        def gen_bin_files(extract_file, cap, protocol, port_type, port):
            with extract_file.open('wb') as f:
                for packet in cap:
                    # for packet in tqdm(cap, desc="Processing", unit="iteration"):
                    try:
                        if protocol in dir(packet) and port_type in dir(packet[protocol]):
                            if port_type == 'srcport':
                                if packet[protocol].srcport == port and 'payload' in dir(packet[protocol]):
                                    frame = packet[protocol].payload
                                else:
                                    continue
                            elif port_type == 'dstport':
                                if packet[protocol].dstport == port and 'payload' in dir(packet[protocol]):
                                    frame = packet[protocol].payload
                                elif packet[protocol].dstport == port and 'options' in dir(packet[protocol]):
                                    frame = packet[protocol].options.value
                                else:
                                    continue
                            elif port_type == 'port':
                                if packet[protocol].port[0] == port:
                                    frame = packet[protocol].payload
                                else:
                                    continue
                            else:
                                print(f"{protocol}.{port_type}")
                                continue
                        else:
                            # print(f"{protocol}.{port_type}")
                            continue

                        f.write(frame)
                    except AttributeError as e:
                        self.output_msg(f"发生 AttributeError 错误: {e}")
                        continue

            # time cost
            end_time = time.time()
            execution_time = end_time - start_time
            self.output_msg(f"代码执行时间为: {execution_time} 秒")

        # time cost
        self.output_msg('开始解析，pcpng解析速度较慢，请稍等...')
        start_time = time.time()

        protocol, port_type = port_info.split('.')[0], port_info.split('.')[1]

        if os.path.exists(path):
            pass
        else:
            return '无该文件: ' + path
        input_file = Path(path)

        if port_type == 'port':
            port_type_list = ['srcport', 'dstport', 'port']
        else:
            port_type_list = [port_type]

        t_list = []
        for port_type in port_type_list:

            # read pcapng file
            display_filter = f"{protocol}.{port_type}=={port} and len({protocol}.payload)>0"
            cap = pyshark.FileCapture(input_file, use_ek=True, display_filter=display_filter)
            extract_file = input_file.with_name(input_file.stem + "_" + str(port_type) + "_" + str(port) + ".bin")
            self.output_msg("output_file:" + str(extract_file))

            # # 12302
            with extract_file.open('wb') as f:
                for packet in cap:
                    try:
                        if protocol in dir(packet) and port_type in dir(packet[protocol]):
                            if port_type == 'srcport':
                                if packet[protocol].srcport == port and 'payload' in dir(packet[protocol]):
                                    frame = packet[protocol].payload
                                else:
                                    continue
                            elif port_type == 'dstport':
                                if packet[protocol].dstport == port and 'payload' in dir(packet[protocol]):
                                    frame = packet[protocol].payload
                                elif packet[protocol].dstport == port and 'options' in dir(packet[protocol]):
                                    frame = packet[protocol].options.value
                                else:
                                    continue
                            elif port_type == 'port':
                                if packet[protocol].port[0] == port:
                                    frame = packet[protocol].payload
                                elif packet[protocol].port[1] == port:
                                    frame = packet[protocol].payload
                                else:
                                    continue
                            else:
                                print(port_info)
                                continue
                        else:
                            print(port_info)
                            continue

                        f.write(frame)
                    except AttributeError as e:
                        self.output_msg(f"发生 AttributeError 错误: {e}")
                        continue

            # t_list.append(Thread(target=gen_bin_files
            #                 , args=[extract_file, cap, protocol, port_type, port]))  # 创建一个线程
            # t_list[-1].setDaemon(True)  # 设置守护线程，如果进程结束，会自动去结束线程
            # t_list[-1].start()  # 启动线程

        # time cost
        end_time = time.time()
        execution_time = end_time - start_time
        self.output_msg(f"代码执行时间为: {execution_time} 秒")

    def output_msg(self, msg):
        print(msg)
        self.messages += '\n'
        self.messages += msg


if __name__ == "__main__":
    DialogWireshark().parse_pcpng(r"E:\Downloads\0227原车件绕8字_09版本.pcapng", )
