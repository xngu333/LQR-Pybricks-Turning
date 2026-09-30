"""
Example program using LQR turning control.
"""

from lqr_v1_0 import*

def main():
    """
    Example mission demonstrating LQR turn function.
    """
    # Initialize IMU
    hub.imu.reset_heading(0)
    
    # Example turns
    print("Turning 90 degrees...")
    DC_XoayLQR(50, 90)
    
    print("Turning 180 degrees...")
    DC_XoayLQR(50, 180)
    
    print("Turning -90 degrees...")
    DC_XoayLQR(50, -90)
    
    print("Mission complete!")


if __name__ == "__main__":
    main()
