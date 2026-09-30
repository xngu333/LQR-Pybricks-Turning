# LQR Turning Controller

## What is LQR?

**LQR (Linear Quadratic Regulator)** is a state-feedback control method used to control a system toward a desired state while balancing:

1. How far the system is from the desired state.
2. How much control effort is being used.

The basic LQR control law is:

```text
u = -Kx
```

where:

- `x` = current state of the system
- `u` = control input
- `K` = feedback gain matrix

For a true LQR controller, `K` is calculated from a mathematical model of the system and the weighting matrices `Q` and `R`.

---

# LQR for Robot Turning

For my LEGO robot, LQR is used to control the robot's rotation.

I use two states:

$$
x =
\begin{bmatrix}
e \\
\omega
\end{bmatrix}
$$

where:

- `e` = angle error
- `ω` = angular velocity

The desired state is:

$$
x =
\begin{bmatrix}
0 \\
0
\end{bmatrix}
$$

This means:

- `e = 0` → the robot is at the target angle
- `ω = 0` → the robot has stopped rotating

---

## 1. Angle Error

Angle error tells the controller how far the robot is from its target.

$$
e = \text{target angle} - \text{current angle}
$$

For example, if the target is `90°` and the robot is currently at `60°`:

$$
e = 90 - 60
$$

$$
e = 30^\circ
$$

The robot still needs to turn `30°`.

---

## 2. Angular Velocity

Angular velocity tells the controller **how fast the robot is currently turning**.

$$
\omega = \frac{\Delta \text{angle}}{\Delta t}
$$

For example, if the robot turns from `20°` to `30°` in `0.1` seconds:

$$
\omega = \frac{30 - 20}{0.1}
$$

$$
\omega = 100^\circ/s
$$

So `ω` is basically the robot's **turning speed**.

---

# The LQR Control Equation

With two states, the controller can be written as:

$$
u = -(K_1e + K_2\omega)
$$

where:

- `K₁` determines how strongly the controller reacts to angle error.
- `K₂` determines how strongly the controller reacts to angular velocity.
- `u` is the motor control command.

For example:

$$
e = 10^\circ
$$

$$
\omega = 50^\circ/s
$$

and:

$$
K_1 = 2
$$

$$
K_2 = 0.1
$$

Then:

$$
u = -((2)(10) + (0.1)(50))
$$

$$
u = -(20 + 5)
$$

$$
u = -25
$$

If `u` is sent to `motor.dc()`, then `-25` represents a **-25% motor duty cycle**.

---

# How the Feedback Loop Works

The controller continuously repeats this process:

```text
             Gyro
               ↓
        Current heading
               ↓
        Calculate error
               ↓
     Calculate angular velocity
               ↓
          ┌─────────┐
          │   LQR   │
          │  u = -Kx│
          └─────────┘
               ↓
          Motor power
               ↓
             Robot
               ↓
          Robot turns
               ↓
       New gyro reading
               ↓
             Repeat
```

The controller does not simply calculate the entire turn once.

Instead, it repeatedly measures the robot's actual movement and adjusts the motor command.

---

# Why Angular Velocity Matters

Suppose the robot needs to turn to `90°`.

At the beginning:

```text
Target  = 90°
Current = 20°

Error = 70°
```

The robot is far from the target, so the controller can apply a larger turning command.

Later:

```text
Target  = 90°
Current = 85°

Error = 5°
```

The robot is now very close to the target.

However, it might still be rotating quickly:

```text
Error = 5°
Angular velocity = 100°/s
```

The controller knows that the robot is already rotating quickly.

The `K₂ω` term therefore affects the control command and provides damping.

This helps the robot slow down before reaching the target instead of continuing to rotate too quickly and overshooting.

---

# Typical Turning Behavior

A well-tuned controller can produce behavior similar to:

```text
Turning speed
     ↑
     |          /\
     |         /  \
     |        /    \
     |       /      \
     |______/        \____
     |
     +------------------------→ Time
        Accelerate  Decelerate
```

The robot generally:

1. Accelerates toward the target.
2. Reaches a higher turning speed.
3. Gradually reduces its turning speed.
4. Approaches the target.
5. Stops rotating near the target.

The exact behavior depends on the robot, its dynamics, and the controller gains.

---

# LQR vs PID

LQR and PID are both feedback controllers, but they approach the problem differently.

## PID

PID uses:

$$
u =
K_Pe
+
K_I\int e\,dt
+
K_D\frac{de}{dt}
$$

It considers:

- **P** → current error
- **I** → accumulated error
- **D** → rate of change of error

---

## LQR

LQR uses the system's state:

$$
u = -Kx
$$

For this robot:

$$
x =
\begin{bmatrix}
e \\
\omega
\end{bmatrix}
$$

Therefore:

$$
u = -(K_1e + K_2\omega)
$$

The key difference is that LQR is designed around a **state-space model and an optimization problem**.

PID directly uses proportional, integral, and derivative terms, while LQR considers the system's state and finds a control strategy that minimizes a defined cost.

---

# Why Is LQR Called "Optimal"?

The "optimal" part comes from the cost function.

A standard infinite-horizon LQR cost is:

$$
J =
\int_0^\infty
\left(
x^TQx + u^TRu
\right)dt
$$

where:

- `Q` determines how much the controller cares about state error.
- `R` determines how much the controller cares about control effort.

In simple terms:

```text
        Smaller state error
                 ↕
        Smaller motor effort
```

LQR finds a balance between these two goals.

The controller uses the system model:

$$
\dot{x} = Ax + Bu
$$

where:

- `A` describes how the system naturally behaves.
- `B` describes how the control input affects the system.

The Riccati equation is then solved to obtain the gain matrix `K`.

The resulting controller is:

$$
u = -Kx
$$

---

# Robot Implementation

The basic feedback loop looks like this:

```python
last_time = watch.time()
last_heading = hub.imu.heading()

while True:

    current_time = watch.time()
    current_heading = hub.imu.heading()

    # Time difference
    dt = (current_time - last_time) / 1000

    if dt <= 0:
        continue

    # Heading difference
    delta_heading = current_heading - last_heading

    # Handle gyro wraparound
    if delta_heading > 180:
        delta_heading -= 360
    elif delta_heading < -180:
        delta_heading += 360

    # Angular velocity
    omega = delta_heading / dt

    # Angle error
    e = target - current_heading

    # Handle target wraparound
    if e > 180:
        e -= 360
    elif e < -180:
        e += 360

    # LQR state feedback
    u = -(K1 * e + K2 * omega)

    # Limit motor command
    u = max(-100, min(100, u))

    # Apply motor command
    motorL.dc(round(u))
    motorR.dc(round(-u))

    # Save values for the next loop
    last_time = current_time
    last_heading = current_heading
```

---

# What Happens During Each Loop?

Each loop does four main things:

### 1. Read the gyro

The robot gets its current heading.

```python
current_heading = hub.imu.heading()
```

### 2. Calculate angular velocity

The controller compares the current heading with the previous heading.

```python
omega = delta_heading / dt
```

This tells the controller how quickly the robot is turning.

### 3. Calculate the error

```python
e = target - current_heading
```

This tells the controller how far the robot is from the target.

### 4. Calculate the motor command

```python
u = -(K1 * e + K2 * omega)
```

The motor command is based on both:

```text
Angle error
     +
Turning speed
     ↓
Motor correction
```

Then the robot moves, the gyro measures the new state, and the process repeats.

---

# Important Note About This Implementation

The simplified controller above uses manually selected values for `K1` and `K2`.

For example:

```python
K1 = 2
K2 = 0.1
```

This demonstrates the **state-feedback structure** used by LQR, but these values were not calculated from a complete LQR optimization.

A mathematically designed LQR controller would first create a state-space model:

$$
\dot{x} = Ax + Bu
$$

Then choose appropriate `Q` and `R` matrices and solve the Riccati equation.

This produces the feedback gain matrix:

$$
K
$$

which is then used in:

$$
u = -Kx
$$

Therefore, the current implementation can be described as a **two-state LQR-style/state-feedback controller**, while a future version can calculate the actual LQR gains from the robot's dynamics.

---

# Summary

For this robot, the basic idea is:

$$
x =
\begin{bmatrix}
e \\
\omega
\end{bmatrix}
$$

The controller observes:

- **`e`** → how far the robot is from the target.
- **`ω`** → how fast the robot is currently turning.

Then it calculates:

$$
u = -(K_1e + K_2\omega)
$$

The motor command changes the robot's motion.

The gyro measures the new motion.

The controller calculates the new state.

And the process repeats:

```text
Measure
   ↓
Calculate state
   ↓
Calculate control
   ↓
Move robot
   ↓
Measure again
   ↓
Repeat
```

This feedback process allows the robot to continuously correct its motion while turning toward the desired angle.
