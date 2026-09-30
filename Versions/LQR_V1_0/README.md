# LQR Turning Controller

## What is LQR?

**LQR (Linear Quadratic Regulator)** is a state-feedback control method used to make a system reach a desired state while balancing two things:

1. How far the system is from the desired state.
2. How much control effort is being used.

LQR uses a mathematical model of the system and finds a feedback gain matrix `K` that minimizes a quadratic cost function. The resulting control law is:

\[
u = -Kx
\]

where:

- `x` = current state of the system
- `u` = control input
- `K` = feedback gain matrix

For a standard LQR controller, `K` is calculated from the system model and the weighting matrices `Q` and `R` by solving the Riccati equation.

---

# LQR for Robot Turning

For my robot, I use LQR to control the robot's rotation.

The two states I use are:

\[
x =
\begin{bmatrix}
e\\
\omega
\end{bmatrix}
\]

where:

- `e` = angle error
- `ω` = angular velocity

### 1. Angle Error

The angle error tells the controller how far the robot is from its target.

\[
e = \text{target angle} - \text{current angle}
\]

For example, if the target is `90°` and the robot is currently at `60°`:

\[
e = 90 - 60 = 30°
\]

So the robot still needs to turn `30°`.

---

### 2. Angular Velocity

Angular velocity tells the controller **how quickly the robot is currently turning**.

\[
\omega = \frac{\Delta\text{angle}}{\Delta t}
\]

For example, if the robot turns from `20°` to `30°` in `0.1` seconds:

\[
\omega = \frac{30-20}{0.1}
\]

\[
\omega = 100°/s
\]

This is important because the robot could have a small angle error but still be rotating very quickly.

---

# The Controller

With two states, the feedback equation can be written as:

\[
u = -(K_1e + K_2\omega)
\]

`K1` controls how strongly the controller reacts to angle error.

`K2` controls how strongly the controller reacts to angular velocity.

For example:

\[
e = 10°
\]

\[
\omega = 50°/s
\]

and:

\[
K_1=2
\]

\[
K_2=0.1
\]

Then:

\[
u=-((2)(10)+(0.1)(50))
\]

\[
u=-(20+5)
\]

\[
u=-25
\]

The resulting `u` is then converted into the motor command.

---

# How the Feedback Loop Works

The controller continuously repeats the following process:

```text
        Gyro
          ↓
    Current angle
          ↓
    Calculate error
          ↓
   Calculate angular
      velocity ω
          ↓
     ┌──────────┐
     │   LQR    │
     │ u = -Kx  │
     └──────────┘
          ↓
     Motor power
          ↓
       Robot
          ↓
    Robot turns
          ↓
     New angle
          ↓
       Repeat
```

The important thing is that the controller does **not** decide the entire movement in advance.

Instead, it repeatedly measures what the robot is actually doing and changes the motor command based on the current state.

---

# Why Use Angular Velocity?

Suppose the robot needs to turn to `90°`.

When it is far away:

```text
Target = 90°
Current = 20°

Error = 70°
```

The controller can apply a relatively large turning command.

As the robot approaches:

```text
Target = 90°
Current = 85°

Error = 5°
```

The angle error is now small.

However, the robot might still be rotating quickly.

For example:

```text
Error = 5°
Angular velocity = 100°/s
```

The controller can see that the robot is already rotating quickly and reduce the control effort.

This helps provide damping and reduces overshoot.

---

# LQR vs PID

LQR and PID are both feedback controllers, but they use information differently.

### PID

PID uses:

\[
u=K_Pe+K_I\int e\,dt+K_D\frac{de}{dt}
\]

It considers:

- `P` → current error
- `I` → accumulated error
- `D` → rate of change of error

### LQR

LQR uses the system's state:

\[
u=-Kx
\]

For my simplified turning example:

\[
x=
\begin{bmatrix}
e\\
\omega
\end{bmatrix}
\]

so:

\[
u=-(K_1e+K_2\omega)
\]

The important difference is that LQR is designed around a **state-space model and an optimization problem**, rather than simply tuning proportional, integral, and derivative terms.

---

# What Makes LQR "Optimal"?

The "optimal" part comes from the cost function.

LQR minimizes a cost of the general form:

\[
J=\int_0^\infty(x^TQx+u^TRu)\,dt
\]

`Q` determines how much the controller cares about state error.

`R` determines how much the controller cares about using large control inputs.

Therefore, LQR tries to find a balance between:

```text
        Small state error
               ↕
       Small control effort
```

The gain matrix `K` is obtained by solving the associated Riccati equation.

---

# My Robot Implementation

For the robot implementation, the basic control loop is:

```python
last_time = watch.time()
last_heading = hub.imu.heading()

while True:

    current_time = watch.time()
    current_heading = hub.imu.heading()

    dt = (current_time - last_time) / 1000

    delta_heading = current_heading - last_heading

    # Handle gyro wraparound
    if delta_heading > 180:
        delta_heading -= 360
    elif delta_heading < -180:
        delta_heading += 360

    omega = delta_heading / dt

    # Angle error
    e = target - current_heading

    if e > 180:
        e -= 360
    elif e < -180:
        e += 360

    # State feedback
    u = -(K1 * e + K2 * omega)

    # Apply motor command
    motorL.dc(round(u))
    motorR.dc(round(-u))

    last_time = current_time
    last_heading = current_heading
```

The loop continuously measures:

```text
current angle
      +
angular velocity
      ↓
   controller
      ↓
 motor command
```

This allows the robot to react to its actual motion instead of simply assuming that the robot will turn exactly as commanded.

---

# Important Note

The simplified controller above uses manually selected values for `K1` and `K2`.

For a **true LQR design**, the robot's dynamics would first be represented using a state-space model:

\[
\dot{x}=Ax+Bu
\]

Then `Q` and `R` would be selected to represent the desired trade-off between state error and control effort.

The LQR algorithm solves the Riccati equation and produces the gain matrix:

\[
K
\]

which is then used in:

\[
u=-Kx
\]

This is the standard LQR formulation.

So the current robot implementation is best described as a **two-state LQR-style controller / state-feedback controller**, while the next step would be to derive the actual `K` from the robot's dynamics.
