import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle("ROS2 Joint Control")
        self.setGeometry(100, 100, 300, 200)

        layout = QVBoxLayout()

        # First input window
        self.label1 = QLabel("Joint 1 Position:", self)
        self.input1 = QLineEdit(self)
        layout.addWidget(self.label1)
        layout.addWidget(self.input1)

        # Second input window
        self.label2 = QLabel("Joint 2 Position:", self)
        self.input2 = QLineEdit(self)
        layout.addWidget(self.label2)
        layout.addWidget(self.input2)

        # Button to send command
        self.send_button = QPushButton("Send Command", self)
        self.send_button.clicked.connect(self.send_command)
        layout.addWidget(self.send_button)

        self.setLayout(layout)

    def send_command(self):
        try:
            joint1 = float(self.input1.text())
            joint2 = float(self.input2.text())

            # Construct ROS2 command
            command = f'ros2 service call /move_robot bot_movit/srv/SetJointP "{{joint_positions: [{joint1}, {joint2}]}}"'
            
            # Execute command
            os.system(command)

        except ValueError:
            print("Please enter valid float values.")

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
