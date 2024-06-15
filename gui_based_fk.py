import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSpinBox
from PyQt5.QtCore import Qt
from math import cos, sin, radians
import numpy as np

# Dummy link lengths
a1 = 14.0
a2 = 10.5
a3 = 19.0

def get_homogeneous_transform(theta, alpha, r, d):
    transform = np.array([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), r * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta)],
        [0.0, sin(alpha), cos(alpha), d],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return transform

def return_end_effector_position(T1, T2, T3, T4):
    # Convert angles to radians
    T1 = radians(T1)
    T2 = radians(T2)
    T3 = radians(T3)
    T4 = radians(T4)

    # Denavit-Hartenburg table
    DH = np.array([
        [T1, np.pi/2, 0.0, a1],
        [T2, np.pi, a2, 0.0],
        [T3, np.pi/2, 0.0, 0.0],
        [T4, 0.0, 0.0, a3]
    ])

    # Get transforms between frames
    H0_1 = get_homogeneous_transform(*DH[0])
    H1_2 = get_homogeneous_transform(*DH[1])
    H2_3 = get_homogeneous_transform(*DH[2])
    H3_4 = get_homogeneous_transform(*DH[3])

    H0_2 = np.dot(H0_1, H1_2)
    H0_3 = np.dot(H0_2, H2_3)
    H0_4 = np.dot(H0_3, H3_4)

    end_effector_position = H0_4[:3, -1]
    x = round(end_effector_position[0], 2)
    y = round(end_effector_position[1], 2)
    z = round(end_effector_position[2], 2)
    return (x, y, z)

class RobotGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Robot End Effector Position')
        self.setGeometry(100, 100, 400, 300)

        self.T1_spinbox = QSpinBox(self)
        self.T2_spinbox = QSpinBox(self)
        self.T3_spinbox = QSpinBox(self)
        self.T4_spinbox = QSpinBox(self)

        for spinbox in [self.T1_spinbox, self.T2_spinbox, self.T3_spinbox, self.T4_spinbox]:
            spinbox.setRange(0, 180)
            spinbox.setSingleStep(1)
            spinbox.valueChanged.connect(self.update_position)

        self.T1_spinbox.setValue(45)
        self.T2_spinbox.setValue(60)
        self.T3_spinbox.setValue(67)
        self.T4_spinbox.setValue(0)

        layout = QVBoxLayout()

        for i, spinbox in enumerate([self.T1_spinbox, self.T2_spinbox, self.T3_spinbox, self.T4_spinbox], start=1):
            hlayout = QHBoxLayout()
            hlayout.addWidget(QLabel(f'T{i}:', self))
            hlayout.addWidget(spinbox)
            layout.addLayout(hlayout)

        self.position_label = QLabel(self)
        layout.addWidget(self.position_label)

        self.setLayout(layout)
        self.update_position()

    def update_position(self):
        T1 = self.T1_spinbox.value()
        T2 = self.T2_spinbox.value()
        T3 = self.T3_spinbox.value()
        T4 = self.T4_spinbox.value()

        position = return_end_effector_position(T1, T2, T3, T4)
        if hasattr(self, 'position_label'):
            self.position_label.setText(f"End Effector Position: {position}")
        else:
            print("position_label is not initialized")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotGUI()
    ex.show()
    sys.exit(app.exec_())
