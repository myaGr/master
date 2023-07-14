# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow_DataAnalysis.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(656, 693)
        font = QtGui.QFont()
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        MainWindow.setFont(font)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("C:/Users/wenzixuan/.designer/backup/AsensingIco.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.textBro_PrintMsg2 = QtWidgets.QTabWidget(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.textBro_PrintMsg2.setFont(font)
        self.textBro_PrintMsg2.setObjectName("textBro_PrintMsg2")
        self.tab1 = QtWidgets.QWidget()
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.tab1.setFont(font)
        self.tab1.setObjectName("tab1")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.tab1)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget = QtWidgets.QWidget(self.tab1)
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(0, 6, 0, 6)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtWidgets.QLabel(self.widget)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.lineEdit_GetFile = QtWidgets.QLineEdit(self.widget)
        self.lineEdit_GetFile.setObjectName("lineEdit_GetFile")
        self.horizontalLayout.addWidget(self.lineEdit_GetFile)
        self.pushBtn_SelectFile = QtWidgets.QPushButton(self.widget)
        self.pushBtn_SelectFile.setObjectName("pushBtn_SelectFile")
        self.horizontalLayout.addWidget(self.pushBtn_SelectFile)
        self.verticalLayout_2.addWidget(self.widget)
        self.widget_2 = QtWidgets.QWidget(self.tab1)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_2)
        self.horizontalLayout_2.setContentsMargins(0, 9, 0, 9)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_3 = QtWidgets.QLabel(self.widget_2)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_2.addWidget(self.label_3)
        self.checkBox_csv = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_csv.setObjectName("checkBox_csv")
        self.horizontalLayout_2.addWidget(self.checkBox_csv)
        self.checkBox_mat = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_mat.setObjectName("checkBox_mat")
        self.horizontalLayout_2.addWidget(self.checkBox_mat)
        self.verticalLayout_2.addWidget(self.widget_2)
        self.widget_12 = QtWidgets.QWidget(self.tab1)
        self.widget_12.setObjectName("widget_12")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.widget_12)
        self.horizontalLayout_10.setContentsMargins(0, 9, 0, 9)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_6 = QtWidgets.QLabel(self.widget_12)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_10.addWidget(self.label_6)
        self.radioButton_tow = QtWidgets.QRadioButton(self.widget_12)
        self.radioButton_tow.setChecked(True)
        self.radioButton_tow.setObjectName("radioButton_tow")
        self.horizontalLayout_10.addWidget(self.radioButton_tow)
        self.radioButton_time = QtWidgets.QRadioButton(self.widget_12)
        self.radioButton_time.setObjectName("radioButton_time")
        self.horizontalLayout_10.addWidget(self.radioButton_time)
        self.verticalLayout_2.addWidget(self.widget_12)
        self.widget_7 = QtWidgets.QWidget(self.tab1)
        self.widget_7.setObjectName("widget_7")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.widget_7)
        self.horizontalLayout_7.setContentsMargins(0, 6, 0, 6)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.pushBtn_StartParse1 = QtWidgets.QPushButton(self.widget_7)
        self.pushBtn_StartParse1.setMinimumSize(QtCore.QSize(0, 35))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.pushBtn_StartParse1.setFont(font)
        self.pushBtn_StartParse1.setAutoRepeatDelay(300)
        self.pushBtn_StartParse1.setObjectName("pushBtn_StartParse1")
        self.horizontalLayout_7.addWidget(self.pushBtn_StartParse1)
        self.pushBtn_StartPlot1 = QtWidgets.QPushButton(self.widget_7)
        self.pushBtn_StartPlot1.setEnabled(False)
        self.pushBtn_StartPlot1.setMinimumSize(QtCore.QSize(0, 35))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.pushBtn_StartPlot1.setFont(font)
        self.pushBtn_StartPlot1.setAutoRepeatDelay(300)
        self.pushBtn_StartPlot1.setObjectName("pushBtn_StartPlot1")
        self.horizontalLayout_7.addWidget(self.pushBtn_StartPlot1)
        self.verticalLayout_2.addWidget(self.widget_7)
        self.textBrowser_showmsg1 = QtWidgets.QTextBrowser(self.tab1)
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.textBrowser_showmsg1.setFont(font)
        self.textBrowser_showmsg1.setObjectName("textBrowser_showmsg1")
        self.verticalLayout_2.addWidget(self.textBrowser_showmsg1)
        self.widget_3 = QtWidgets.QWidget(self.tab1)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setSpacing(6)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        spacerItem = QtWidgets.QSpacerItem(306, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)
        self.pushBtn_ClearMsg1 = QtWidgets.QPushButton(self.widget_3)
        self.pushBtn_ClearMsg1.setObjectName("pushBtn_ClearMsg1")
        self.horizontalLayout_3.addWidget(self.pushBtn_ClearMsg1)
        self.verticalLayout_2.addWidget(self.widget_3)
        self.textBro_PrintMsg2.addTab(self.tab1, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setEnabled(True)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.tab_2.setFont(font)
        self.tab_2.setObjectName("tab_2")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.tab_2)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.widget_5 = QtWidgets.QWidget(self.tab_2)
        self.widget_5.setObjectName("widget_5")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.widget_5)
        self.horizontalLayout_5.setContentsMargins(0, 6, 0, 6)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_5 = QtWidgets.QLabel(self.widget_5)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_5.addWidget(self.label_5)
        self.lineEdit_GetRefFile = QtWidgets.QLineEdit(self.widget_5)
        self.lineEdit_GetRefFile.setObjectName("lineEdit_GetRefFile")
        self.horizontalLayout_5.addWidget(self.lineEdit_GetRefFile)
        self.pushBtn_SelectRef = QtWidgets.QPushButton(self.widget_5)
        self.pushBtn_SelectRef.setObjectName("pushBtn_SelectRef")
        self.horizontalLayout_5.addWidget(self.pushBtn_SelectRef)
        self.select_type = QtWidgets.QComboBox(self.widget_5)
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.select_type.setFont(font)
        self.select_type.setObjectName("select_type")
        self.select_type.addItem("")
        self.select_type.addItem("")
        self.select_type.addItem("")
        self.horizontalLayout_5.addWidget(self.select_type)
        self.verticalLayout_6.addWidget(self.widget_5)
        self.widget_10 = QtWidgets.QWidget(self.tab_2)
        self.widget_10.setObjectName("widget_10")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget_10)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget_6 = QtWidgets.QWidget(self.widget_10)
        self.widget_6.setObjectName("widget_6")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.widget_6)
        self.horizontalLayout_6.setContentsMargins(0, 6, 0, 6)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label = QtWidgets.QLabel(self.widget_6)
        self.label.setObjectName("label")
        self.horizontalLayout_6.addWidget(self.label)
        self.lineEdit_GetInsFile = QtWidgets.QLineEdit(self.widget_6)
        self.lineEdit_GetInsFile.setObjectName("lineEdit_GetInsFile")
        self.horizontalLayout_6.addWidget(self.lineEdit_GetInsFile)
        self.pushBtn_SelectIns = QtWidgets.QPushButton(self.widget_6)
        self.pushBtn_SelectIns.setObjectName("pushBtn_SelectIns")
        self.horizontalLayout_6.addWidget(self.pushBtn_SelectIns)
        self.select_test_type = QtWidgets.QComboBox(self.widget_6)
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.select_test_type.setFont(font)
        self.select_test_type.setObjectName("select_test_type")
        self.select_test_type.addItem("")
        self.select_test_type.addItem("")
        self.horizontalLayout_6.addWidget(self.select_test_type)
        self.toolButton = QtWidgets.QToolButton(self.widget_6)
        self.toolButton.setObjectName("toolButton")
        self.horizontalLayout_6.addWidget(self.toolButton)
        self.verticalLayout.addWidget(self.widget_6)
        self.verticalLayout_6.addWidget(self.widget_10)
        self.widget_9 = QtWidgets.QWidget(self.tab_2)
        self.widget_9.setObjectName("widget_9")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.widget_9)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.pushBtn_SelectIns_2 = QtWidgets.QPushButton(self.widget_9)
        self.pushBtn_SelectIns_2.setObjectName("pushBtn_SelectIns_2")
        self.horizontalLayout_9.addWidget(self.pushBtn_SelectIns_2)
        self.clear_test_file = QtWidgets.QToolButton(self.widget_9)
        self.clear_test_file.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.clear_test_file.setWhatsThis("")
        self.clear_test_file.setObjectName("clear_test_file")
        self.horizontalLayout_9.addWidget(self.clear_test_file)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_9.addItem(spacerItem1)
        self.pushBtn_addScene = QtWidgets.QPushButton(self.widget_9)
        self.pushBtn_addScene.setObjectName("pushBtn_addScene")
        self.horizontalLayout_9.addWidget(self.pushBtn_addScene)
        self.importScene = QtWidgets.QToolButton(self.widget_9)
        self.importScene.setObjectName("importScene")
        self.horizontalLayout_9.addWidget(self.importScene)
        self.verticalLayout_6.addWidget(self.widget_9)
        self.widget_16 = QtWidgets.QWidget(self.tab_2)
        self.widget_16.setObjectName("widget_16")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget_16)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.widget_11 = QtWidgets.QWidget(self.widget_16)
        self.widget_11.setObjectName("widget_11")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout(self.widget_11)
        self.horizontalLayout_11.setContentsMargins(0, 6, 0, 6)
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_9 = QtWidgets.QLabel(self.widget_11)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_11.addWidget(self.label_9)
        self.lineEdit_scene = QtWidgets.QLineEdit(self.widget_11)
        self.lineEdit_scene.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.lineEdit_scene.setText("")
        self.lineEdit_scene.setObjectName("lineEdit_scene")
        self.horizontalLayout_11.addWidget(self.lineEdit_scene)
        self.label_7 = QtWidgets.QLabel(self.widget_11)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_11.addWidget(self.label_7)
        self.lineEdit_StarTime = QtWidgets.QLineEdit(self.widget_11)
        self.lineEdit_StarTime.setText("")
        self.lineEdit_StarTime.setObjectName("lineEdit_StarTime")
        self.horizontalLayout_11.addWidget(self.lineEdit_StarTime)
        self.label_8 = QtWidgets.QLabel(self.widget_11)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_11.addWidget(self.label_8)
        self.lineEdit_EndTime = QtWidgets.QLineEdit(self.widget_11)
        self.lineEdit_EndTime.setText("")
        self.lineEdit_EndTime.setObjectName("lineEdit_EndTime")
        self.horizontalLayout_11.addWidget(self.lineEdit_EndTime)
        self.verticalLayout_4.addWidget(self.widget_11)
        self.verticalLayout_6.addWidget(self.widget_16)
        self.widget_8 = QtWidgets.QWidget(self.tab_2)
        self.widget_8.setObjectName("widget_8")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.widget_8)
        self.horizontalLayout_8.setContentsMargins(0, 2, 0, 6)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.pushBtn_StartParse2 = QtWidgets.QPushButton(self.widget_8)
        self.pushBtn_StartParse2.setEnabled(True)
        self.pushBtn_StartParse2.setMinimumSize(QtCore.QSize(0, 35))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.pushBtn_StartParse2.setFont(font)
        self.pushBtn_StartParse2.setObjectName("pushBtn_StartParse2")
        self.horizontalLayout_8.addWidget(self.pushBtn_StartParse2)
        self.pushBtn_StartPlot2 = QtWidgets.QPushButton(self.widget_8)
        self.pushBtn_StartPlot2.setEnabled(False)
        self.pushBtn_StartPlot2.setMinimumSize(QtCore.QSize(0, 35))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.pushBtn_StartPlot2.setFont(font)
        self.pushBtn_StartPlot2.setObjectName("pushBtn_StartPlot2")
        self.horizontalLayout_8.addWidget(self.pushBtn_StartPlot2)
        self.widget_13 = QtWidgets.QWidget(self.widget_8)
        self.widget_13.setObjectName("widget_13")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout(self.widget_13)
        self.horizontalLayout_13.setContentsMargins(0, 6, 0, 6)
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_10 = QtWidgets.QLabel(self.widget_13)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_13.addWidget(self.label_10)
        self.radioButton_tow2 = QtWidgets.QRadioButton(self.widget_13)
        self.radioButton_tow2.setChecked(True)
        self.radioButton_tow2.setObjectName("radioButton_tow2")
        self.horizontalLayout_13.addWidget(self.radioButton_tow2)
        self.radioButton_time2 = QtWidgets.QRadioButton(self.widget_13)
        self.radioButton_time2.setObjectName("radioButton_time2")
        self.horizontalLayout_13.addWidget(self.radioButton_time2)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_13.addItem(spacerItem2)
        self.label_4 = QtWidgets.QLabel(self.widget_13)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_13.addWidget(self.label_4)
        self.showGPS_spinBox = QtWidgets.QSpinBox(self.widget_13)
        self.showGPS_spinBox.setMinimum(1)
        self.showGPS_spinBox.setObjectName("showGPS_spinBox")
        self.horizontalLayout_13.addWidget(self.showGPS_spinBox)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_13.addItem(spacerItem3)
        self.horizontalLayout_8.addWidget(self.widget_13)
        self.verticalLayout_6.addWidget(self.widget_8)
        self.textBrowser_showmsg2 = QtWidgets.QTextBrowser(self.tab_2)
        self.textBrowser_showmsg2.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.IBeamCursor))
        self.textBrowser_showmsg2.setObjectName("textBrowser_showmsg2")
        self.verticalLayout_6.addWidget(self.textBrowser_showmsg2)
        self.widget_4 = QtWidgets.QWidget(self.tab_2)
        self.widget_4.setObjectName("widget_4")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widget_4)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setSpacing(9)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        spacerItem4 = QtWidgets.QSpacerItem(306, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem4)
        self.pushBtn_ClearMsg2 = QtWidgets.QPushButton(self.widget_4)
        self.pushBtn_ClearMsg2.setObjectName("pushBtn_ClearMsg2")
        self.horizontalLayout_4.addWidget(self.pushBtn_ClearMsg2)
        self.verticalLayout_6.addWidget(self.widget_4)
        self.textBro_PrintMsg2.addTab(self.tab_2, "")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.tab)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.widget_15 = QtWidgets.QWidget(self.tab)
        self.widget_15.setObjectName("widget_15")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout(self.widget_15)
        self.horizontalLayout_12.setContentsMargins(0, 6, -1, 6)
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_dbc = QtWidgets.QLabel(self.widget_15)
        self.label_dbc.setObjectName("label_dbc")
        self.horizontalLayout_12.addWidget(self.label_dbc)
        self.lineEdit_getDBCtext = QtWidgets.QLineEdit(self.widget_15)
        self.lineEdit_getDBCtext.setObjectName("lineEdit_getDBCtext")
        self.horizontalLayout_12.addWidget(self.lineEdit_getDBCtext)
        self.pushBtn_getDBCtext = QtWidgets.QPushButton(self.widget_15)
        self.pushBtn_getDBCtext.setObjectName("pushBtn_getDBCtext")
        self.horizontalLayout_12.addWidget(self.pushBtn_getDBCtext)
        self.verticalLayout_3.addWidget(self.widget_15)
        self.lineEdit_getfiletext = QtWidgets.QWidget(self.tab)
        self.lineEdit_getfiletext.setObjectName("lineEdit_getfiletext")
        self.horizontalLayout_16 = QtWidgets.QHBoxLayout(self.lineEdit_getfiletext)
        self.horizontalLayout_16.setContentsMargins(0, 6, -1, 6)
        self.horizontalLayout_16.setObjectName("horizontalLayout_16")
        self.label_file = QtWidgets.QLabel(self.lineEdit_getfiletext)
        self.label_file.setObjectName("label_file")
        self.horizontalLayout_16.addWidget(self.label_file)
        self.lineEdit_GetFiletext = QtWidgets.QLineEdit(self.lineEdit_getfiletext)
        self.lineEdit_GetFiletext.setObjectName("lineEdit_GetFiletext")
        self.horizontalLayout_16.addWidget(self.lineEdit_GetFiletext)
        self.pushBtn_GetFiletext = QtWidgets.QPushButton(self.lineEdit_getfiletext)
        self.pushBtn_GetFiletext.setObjectName("pushBtn_GetFiletext")
        self.horizontalLayout_16.addWidget(self.pushBtn_GetFiletext)
        self.verticalLayout_3.addWidget(self.lineEdit_getfiletext)
        self.widget_14 = QtWidgets.QWidget(self.tab)
        self.widget_14.setObjectName("widget_14")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.widget_14)
        self.verticalLayout_5.setContentsMargins(0, 6, 0, 6)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.pushBtn_decode = QtWidgets.QPushButton(self.widget_14)
        self.pushBtn_decode.setEnabled(True)
        self.pushBtn_decode.setMinimumSize(QtCore.QSize(0, 35))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.pushBtn_decode.setFont(font)
        self.pushBtn_decode.setObjectName("pushBtn_decode")
        self.verticalLayout_5.addWidget(self.pushBtn_decode)
        self.textBrowser_can = QtWidgets.QTextBrowser(self.widget_14)
        self.textBrowser_can.setObjectName("textBrowser_can")
        self.verticalLayout_5.addWidget(self.textBrowser_can)
        self.verticalLayout_3.addWidget(self.widget_14)
        self.widget_17 = QtWidgets.QWidget(self.tab)
        self.widget_17.setObjectName("widget_17")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout(self.widget_17)
        self.horizontalLayout_14.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        spacerItem5 = QtWidgets.QSpacerItem(578, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_14.addItem(spacerItem5)
        self.pushBtn_ClearMsg_can = QtWidgets.QPushButton(self.widget_17)
        self.pushBtn_ClearMsg_can.setObjectName("pushBtn_ClearMsg_can")
        self.horizontalLayout_14.addWidget(self.pushBtn_ClearMsg_can)
        self.verticalLayout_3.addWidget(self.widget_17)
        self.textBro_PrintMsg2.addTab(self.tab, "")
        self.verticalLayout_7.addWidget(self.textBro_PrintMsg2)
        self.botton_help = QtWidgets.QPushButton(self.centralwidget)
        self.botton_help.setObjectName("botton_help")
        self.verticalLayout_7.addWidget(self.botton_help)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.textBro_PrintMsg2.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "数据解析与分析工具V2.0.6.7   Powered by 算法部 2023/07/14"))
        self.label_2.setText(_translate("MainWindow", "文件路径* ："))
        self.pushBtn_SelectFile.setText(_translate("MainWindow", "选择文件"))
        self.label_3.setText(_translate("MainWindow", "保存文件类型："))
        self.checkBox_csv.setText(_translate("MainWindow", ".csv"))
        self.checkBox_mat.setText(_translate("MainWindow", ".mat"))
        self.label_6.setText(_translate("MainWindow", "显示时间类型："))
        self.radioButton_tow.setText(_translate("MainWindow", "周内秒"))
        self.radioButton_time.setText(_translate("MainWindow", "计时时间"))
        self.pushBtn_StartParse1.setText(_translate("MainWindow", "开始解析"))
        self.pushBtn_StartPlot1.setText(_translate("MainWindow", "开始画图"))
        self.pushBtn_ClearMsg1.setText(_translate("MainWindow", "清空消息和所有绘制图"))
        self.textBro_PrintMsg2.setTabText(self.textBro_PrintMsg2.indexOf(self.tab1), _translate("MainWindow", "INS解析画图分析"))
        self.label_5.setText(_translate("MainWindow", "参考文件*  ："))
        self.lineEdit_GetRefFile.setPlaceholderText(_translate("MainWindow", ""))
        self.pushBtn_SelectRef.setText(_translate("MainWindow", "选择文件"))
        self.select_type.setItemText(0, _translate("MainWindow", "100C"))
        self.select_type.setItemText(1, _translate("MainWindow", "320"))
        self.select_type.setItemText(2, _translate("MainWindow", "华测特制"))
        self.label.setText(_translate("MainWindow", "测试文件*  ："))
        self.pushBtn_SelectIns.setText(_translate("MainWindow", "选择文件"))
        self.select_test_type.setItemText(0, _translate("MainWindow", "导远自定义"))
        self.select_test_type.setItemText(1, _translate("MainWindow", "北云明文"))
        self.toolButton.setWhatsThis(_translate("MainWindow", "<html><head/><body><p>参数设置</p></body></html>"))
        self.toolButton.setText(_translate("MainWindow", "..."))
        self.pushBtn_SelectIns_2.setText(_translate("MainWindow", "+ 添加测试文件"))
        self.clear_test_file.setText(_translate("MainWindow", "清除测试文件"))
        self.pushBtn_addScene.setText(_translate("MainWindow", "+添加场景"))
        self.importScene.setWhatsThis(_translate("MainWindow", "<html><head/><body><p>参数设置</p></body></html>"))
        self.importScene.setText(_translate("MainWindow", "导入/清除场景"))
        self.label_9.setWhatsThis(_translate("MainWindow", "<html><head/><body><p>默认统计全程时间</p></body></html>"))
        self.label_9.setText(_translate("MainWindow", "场景："))
        self.lineEdit_scene.setPlaceholderText(_translate("MainWindow", "高速/隧道/等..."))
        self.label_7.setWhatsThis(_translate("MainWindow", "<html><head/><body><p>默认统计全程时间</p></body></html>"))
        self.label_7.setText(_translate("MainWindow", "开始(周内秒, s)："))
        self.lineEdit_StarTime.setPlaceholderText(_translate("MainWindow", "0"))
        self.label_8.setText(_translate("MainWindow", " 结束(周内秒, s)："))
        self.lineEdit_EndTime.setPlaceholderText(_translate("MainWindow", "0"))
        self.pushBtn_StartParse2.setText(_translate("MainWindow", "开始统计"))
        self.pushBtn_StartPlot2.setText(_translate("MainWindow", "开始画图"))
        self.label_10.setText(_translate("MainWindow", "显示时间类型："))
        self.radioButton_tow2.setText(_translate("MainWindow", "周内秒"))
        self.radioButton_time2.setText(_translate("MainWindow", "计时时间"))
        self.label_4.setText(_translate("MainWindow", "显示GPS数量："))
        self.pushBtn_ClearMsg2.setText(_translate("MainWindow", "清空消息和所有绘制图"))
        self.textBro_PrintMsg2.setTabText(self.textBro_PrintMsg2.indexOf(self.tab_2), _translate("MainWindow", "INS精度统计分析"))
        self.label_dbc.setText(_translate("MainWindow", "dbc文件/文件夹路径* :"))
        self.lineEdit_getDBCtext.setPlaceholderText(_translate("MainWindow", "dbc文件/文件夹路径"))
        self.pushBtn_getDBCtext.setText(_translate("MainWindow", "选择文件"))
        self.label_file.setText(_translate("MainWindow", "待解析文件*  ："))
        self.lineEdit_GetFiletext.setToolTip(_translate("MainWindow", "blf或asc文件/文件夹路径"))
        self.lineEdit_GetFiletext.setPlaceholderText(_translate("MainWindow", "blf或asc文件/文件夹路径"))
        self.pushBtn_GetFiletext.setText(_translate("MainWindow", "选择文件"))
        self.pushBtn_decode.setText(_translate("MainWindow", "解析CAN数据成mat格式"))
        self.pushBtn_ClearMsg_can.setText(_translate("MainWindow", "清空消息"))
        self.textBro_PrintMsg2.setTabText(self.textBro_PrintMsg2.indexOf(self.tab), _translate("MainWindow", "CAN文件解析"))
        self.botton_help.setText(_translate("MainWindow", "HELP"))
