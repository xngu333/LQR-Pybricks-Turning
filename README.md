# LQR-Pybricks-Turning

A Python project for controlling a Pybricks-based robot with turning behavior driven by an LQR (Linear Quadratic Regulator) control strategy.

## Overview

This repository is intended for experiments and implementations involving:

- Pybricks robot control
- Robot turning and motion regulation
- LQR-based feedback control
- LEGO Mindstorms / Pybricks development workflows

The code in this project can be adapted for testing control logic, tuning movement behavior, and developing autonomous robot turning routines.

## Project goals

- Model and control robot turning dynamics
- Use feedback control to improve motion stability
- Implement reusable motion functions in Python
- Run experiments on a Pybricks-enabled robot platform

## Requirements

Before running the project, make sure you have:

- Python 3.9+
- Pybricks installed
- A compatible LEGO Mindstorms robot hub supported by Pybricks
- A development environment such as VS Code, PyCharm, or a terminal session

Install Pybricks:

```bash
pip install pybricks
```

## Repository structure

```text
LQR-Pybricks-Turning/
├── README.md
├── src/
├── scripts/
├── tests/
└── requirements.txt
```

If this repository is still being developed, you may add or rename folders as your implementation grows.

## Typical usage

1. Clone the repository:

```bash
git clone https://github.com/xngu333/LQR-Pybricks-Turning.git
cd LQR-Pybricks-Turning
```

2. Create and activate a virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate   # Linux/macOS
.venv\Scripts\activate      # Windows
```

3. Install dependencies:

```bash
pip install -r requirements.txt
```

4. Run the Python code for your robot control logic:

```bash
python main.py
```

Adjust the entry point to match the actual file names in your project.

## LQR control notes

This project is designed around a state-space feedback controller. In a typical implementation, the robot states may include:

- heading angle
- angular velocity
- position error
- steering response

The controller aims to minimize deviations from the desired motion while keeping the system stable and responsive.

* Please tune your own LQR Control values to achieve the best stability!
