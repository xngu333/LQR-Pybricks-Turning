"""
Example program using LQR turning control.
"""

from lqr_v1_0 import*

def main():
    hub.imu.reset_heading(0)
    DC_XoayLQR(50, 90)

