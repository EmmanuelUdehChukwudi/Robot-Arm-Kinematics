import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider
from PyQt5.QtCore import Qt
import numpy as np
from math import atan2, acos, sqrt, radians
from forward_kinematics import get_homogeneous_transform

class RobotIKGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.a1 = 14.0
        self.a2 = 10.5
        self.a3 = 19.0
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Inverse Kinematics Solver')
        self.setGeometry(100, 100, 400, 250)

        self.x_slider = QSlider(Qt.Horizontal)
        self.y_slider = QSlider(Qt.Horizontal)
        self.z_slider = QSlider(Qt.Horizontal)

        self.x_slider.setRange(-int(self.a2 + self.a3), int(self.a2 + self.a3))
        self.y_slider.setRange(-int(self.a2 + self.a3), int(self.a2 + self.a3))
        self.z_slider.setRange(0, int(self.a1 + self.a2 + self.a3))

        self.x_slider.valueChanged.connect(self.update_angles)
        self.y_slider.valueChanged.connect(self.update_angles)
        self.z_slider.valueChanged.connect(self.update_angles)

        layout = QVBoxLayout()
        layout.addWidget(QLabel('Move the sliders to set the coordinates:', self))

        sliders_layout = QVBoxLayout()
        sliders_layout.addWidget(QLabel('X:', self))
        sliders_layout.addWidget(self.x_slider)
        sliders_layout.addWidget(QLabel('Y:', self))
        sliders_layout.addWidget(self.y_slider)
        sliders_layout.addWidget(QLabel('Z:', self))
        sliders_layout.addWidget(self.z_slider)
        layout.addLayout(sliders_layout)

        self.current_values_label = QLabel('', self)
        layout.addWidget(self.current_values_label)

        self.result_label = QLabel('', self)
        layout.addWidget(self.result_label)

        self.setLayout(layout)

    def update_angles(self):
        x = self.x_slider.value()
        y = self.y_slider.value()
        z = self.z_slider.value()

        self.current_values_label.setText(f"Current Values: X={x}, Y={y}, Z={z}")

        max_pos = self.a1 + self.a2 + self.a3
        if sqrt(x**2 + y**2 + z**2) > max_pos:
            self.result_label.setText("Coordinates Out of range of the end effector")
            return

        r1 = sqrt(x**2 + y**2)
        theta1 = atan2(y, x)
        r2 = z - self.a1
        r = sqrt(r1**2 + r2**2)
        alpha = atan2(r2, r1)

        # Check if the argument is within the valid range for acos
        try:
            beta = acos((self.a2**2 + r**2 - self.a3**2) / (2 * self.a2 * r))
        except ValueError:
            self.result_label.setText("Error: Invalid coordinates")
            return

        theta2 = alpha - beta

        # Check if the argument is within the valid range for acos
        try:
            phi = acos((self.a2**2 + self.a3**2 - r**2) / (2 * self.a2 * self.a3))
        except ValueError:
            self.result_label.setText("Error: Invalid coordinates")
            return

        theta3 = phi - np.pi / 2

        DH = np.array([
            [theta1, np.pi/2, 0.0, self.a1],
            [theta2, np.pi, self.a2, 0.0],
            [theta3, np.pi/2, 0.0, 0.0],
        ], dtype=np.float32)

        R0_4 = np.zeros((3, 3))
        R0_1 = get_homogeneous_transform(theta=DH[0][0], alpha=DH[0][1], r=DH[0][2], d=DH[0][3])[:3, :3]
        R1_2 = get_homogeneous_transform(theta=DH[1][0], alpha=DH[1][1], r=DH[1][2], d=DH[1][3])[:3, :3]
        R2_3 = get_homogeneous_transform(theta=DH[2][0], alpha=DH[2][1], r=DH[2][2], d=DH[2][3])[:3, :3]

        R0_3 = np.dot(np.dot(R0_1, R1_2), R2_3)
        R0_3_inv = np.transpose(R0_3)
        R3_4 = np.dot(R0_3_inv, R0_4)

        theta4 = atan2(R3_4[1][0], R3_4[0][0])

        theta1 = int(np.rad2deg(theta1))
        theta2= int(np.rad2deg(theta2))
        theta3 = int(np.rad2deg(theta3))
        theta4 = int(np.rad2deg(theta4))

        self.result_label.setText(f"""
            Theta1 = {theta1}
            Theta2 = {theta2}
            Theta3 = {theta3}
            Theta4 = {theta4}
        """)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotIKGUI()
    ex.show()
    sys.exit(app.exec_())

