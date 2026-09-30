"""
LQR-Pybricks-Turning Library v1.0
Linear Quadratic Regulator (LQR) control for Pybricks-based robot turning.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Direction, Port
from pybricks.tools import wait, StopWatch

# ============================================================================
# Hardware Configuration
# ============================================================================

hub = PrimeHub()
motorL = Motor(Port.C, Direction.COUNTERCLOCKWISE)
motorR = Motor(Port.D)

watch = StopWatch()
current_heading = 0

# ============================================================================
# LQR Turn Function
# ============================================================================

def DC_XoayLQR(pwr, deg):
    """
    LQR-based point turn control.
    
    Uses Linear Quadratic Regulator feedback to achieve smooth, stable turning
    by minimizing heading error and angular velocity.
    
    Args:
        pwr: Motor power (0-100)
        deg: Target rotation in degrees
    """
    global current_heading
    
    target = current_heading + deg

    # Normalize target heading to [-180, 180]
    while target > 180:
        target -= 360
    while target <= -180:
        target += 360
    
    # LQR gains
    K1 = 2              # Proportional gain (heading error feedback)
    if pwr > 90: 
        K1 = 1.4        # Reduce gain at high power for stability
    K2 = 0.1            # Derivative gain (angular velocity feedback)

    watch.reset()

    last_time = watch.time()
    last_heading = hub.imu.heading()

    while True:
        current = hub.imu.heading()
        current_time = watch.time()

        # Calculate time delta
        dt = (current_time - last_time) / 1000.0
        if dt <= 0:
            continue

        # Calculate angular velocity (heading change over time)
        delta_heading = current - last_heading
        if delta_heading > 180:
            delta_heading -= 360
        elif delta_heading < -180:
            delta_heading += 360

        omega = delta_heading / dt

        # Calculate heading error
        e = target - current
        if e > 180:
            e -= 360
        elif e < -180:
            e += 360

        # LQR control law: u = -(K1 * e + K2 * omega)
        u = -(K1 * e + K2 * omega)
        u = max(-pwr, min(pwr, u))

        # Apply symmetric control to motors
        motorL.dc(round(-u))
        motorR.dc(round(u))

        # Update state for next iteration
        last_time = current_time
        last_heading = current

        # Exit when heading error and angular velocity are small
        if abs(e) < 1 and abs(omega) < 5:
            break

    # Hold motors in place
    motorL.hold()
    motorR.hold()

    # Update global heading
    current_heading = target
