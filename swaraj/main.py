# main.py
import sys
from PyQt5.QtWidgets import QApplication
from gui_component import RobotMonitorGUI

def main():
    app = QApplication(sys.argv)
    window = RobotMonitorGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
