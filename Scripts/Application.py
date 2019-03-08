from PyQt5.QtWidgets import QApplication, QLabel
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
app = QApplication([])

label = QLabel("1234")
label.show()

app.exec_()


