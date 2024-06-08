# Robotic Arm Kinematics

[![License](https://img.shields.io/github/license/emmanueludehchukwudi/Robot-Arm-Kinematics)](LICENSE)
[![Build Status](https://img.shields.io/github/actions/workflow/status/emmanueludehchukwudi/Robot-Arm-Kinematics/ci.yml?branch=main)](https://github.com/emmanueludehchukwudi/Robot-Arm-Kinematics/actions)
[![Version](https://img.shields.io/github/v/release/emmanueludehchukwudi/Robot-Arm-Kinematics)](https://github.com/emmanueludehchukwudi/Robot-Arm-Kinematics/releases)

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Introduction

This project implements forward and inverse kinematics for a 4-DOF robotic arm using Python. It includes two main scripts:
1. `forward_kinematics.py`: Calculates the end-effector position based on given joint angles using the Denavit-Hartenberg convention.
2. `inverse_kinematics.py`: Determines the joint angles required to achieve a specified end-effector position.

## Features

- **Forward Kinematics**: Computes the position and orientation of the robotic arm's end effector from given joint angles.
- **Inverse Kinematics**: Calculates the necessary joint angles to place the end effector at a desired position.

## Installation

### Prerequisites

- Python 3.x
- NumPy library

### Steps

1. Clone the repository:
   ```sh
   git clone https://github.com/emmanueludehchukwudi/Robot-Arm-Kinematics.git

2. Navigate to the project directory:
   ```sh
   cd Robot-Arm-Kinematics
   
3. Install Dependencies:
    ```sh
    pip install numpy
### Configuration
You can modify the following parameters in the scripts:

Link lengths (a1, a2, a3)
Initial joint angles (T1, T2, T3, T4 in forward_kinematics.py)
Desired end-effector position (x, y, z in inverse_kinematics.py)

### Contributions
Contributions are welcome! Please follow these steps to contribute:

1. Fork the repository.
2. Create a new branch (git checkout -b feature-branch).
3. Make your changes.
4. Commit your changes (git commit -m 'Add some feature').
5. Push to the branch (git push origin feature-branch).
6. Open a pull request.

## Acknowledgements

- [NumPy](https://numpy.org/) for providing the numerical computation tools.


