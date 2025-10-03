# Swerve Drive Training

The aim of this project is to practice writing ROS2 nodes and learn to control motor controllers. In this project, you are expected to write three nodes that work together:

1. **Xbox Publisher** (`xbox_publisher.py`) - Captures Xbox controller input
2. **Swerve Motor** (`swerve_motor.py`) - Calculates inverse kinematics for swerve drive
3. **Swerve Control** (`swerve_control.py`) - Generates CAN bus commands for motor controllers

```
Xbox Controller → xbox_publisher → swerve_motor → swerve_control → CAN Bus → VESCs
```

## Prerequisites

- ROS2 (tested with Foxy/Humble)
- Python 3
- pygame library: `pip install pygame`
- Custom message package: `custom_msgs_srvs` (contains `SwervePrevAngle` message)

## Rover Configuration

- **Vehicle Dimensions:**
  - Body Height: 0.93m (front-to-back)
  - Body Width: 0.60m (side-to-side)

- **VESC Motor Controllers:**
  - FL (Front Left): IDs 70 (drive), 71 (steering)
  - FR (Front Right): IDs 72 (drive), 73 (steering)
  - BL (Back Left): IDs 74 (drive), 75 (steering)
  - BR (Back Right): IDs 76 (drive), 77 (steering)

- **Maximum RPM:** 6000

## Node 1: Xbox Publisher

### Purpose
Reads Xbox controller input and publishes motion commands to the `swerve` topic.

### Topics Published
- `/swerve` (Float32MultiArray): `[forward/back, left/right, left_trigger, right_trigger]`

### Controller Mapping
- **Left Stick Y-axis** (inverted): Forward/backward translation
- **Right Stick X-axis**: Left/right translation (strafing)
- **Left Trigger**: Rotation control (counter-clockwise)
- **Right Trigger**: Rotation control (clockwise)

### Key Features
- **Hot-plugging support:** Automatically detects controller connection/disconnection
- **Safety:** Sends zero motion command when controller disconnects
- **Update rate:** 20 Hz (50ms timer period)

### Usage
```bash
ros2 run <package_name> xbox_publisher
```

## Node 2: Swerve Motor (Inverse Kinematics)

### Purpose
Converts vehicle motion commands into individual wheel velocities and steering angles using swerve drive inverse kinematics.

### Topics Subscribed
- `/swerve` (Float32MultiArray): Motion commands from Xbox controller
- `/prev_pid` (SwervePrevAngle): Previous PID angles (for monitoring)

### Topics Published
- `/swerve_FL` (Float32MultiArray): `[speed, angle]` for front-left wheel
- `/swerve_FR` (Float32MultiArray): `[speed, angle]` for front-right wheel
- `/swerve_BL` (Float32MultiArray): `[speed, angle]` for back-left wheel
- `/swerve_BR` (Float32MultiArray): `[speed, angle]` for back-right wheel

### Swerve Drive Kinematics

The inverse kinematics are based on the standard swerve drive equations. For a vehicle with translational velocity `(Vx, Vy)` and rotational velocity `ω`, each wheel's velocity vector is calculated as:

#### Step 1: Calculate intermediate values
```
A = Vx - ω × (BODY_HEIGHT / 2)
B = Vx + ω × (BODY_HEIGHT / 2)
C = Vy - ω × (BODY_WIDTH / 2)
D = Vy + ω × (BODY_WIDTH / 2)
```

#### Step 2: Determine wheel vectors
```
Front Left  (FL): [B, D]
Front Right (FR): [B, C]
Back Left   (BL): [A, D]
Back Right  (BR): [A, C]
```

#### Step 3: Calculate wheel speed and angle
For each wheel:
```
Speed = √(x² + y²)
Angle = atan2(x, y) × (180/π)
```

#### Step 4: Angle optimization
To minimize wheel rotation, angles are constrained to ±90°:
```
if angle < -90°:
    angle += 180°
    speed *= -1
else if angle ≥ 90°:
    angle -= 180°
    speed *= -1
```

This allows wheels to drive in reverse rather than rotating more than 90°.

### Rotational Velocity Calculation
The rotational velocity is derived from the Xbox triggers:
```
ω = -((left_trigger + 1) / 2) + ((right_trigger + 1) / 2)
```

### Usage
```bash
ros2 run <package_name> swerve_motor
```

## Node 3: Swerve Control (CAN Bus Interface)

### Purpose
Converts wheel speeds and angles into CAN bus commands for VESC motor controllers.

### Topics Subscribed
- `/swerve_FL`, `/swerve_FR`, `/swerve_BL`, `/swerve_BR` (Float32MultiArray)

### Topics Published
- `/can_msg` (String): CAN bus commands in format: `"<ID> <COMMAND> <VALUE> <TYPE>"`

### CAN Command Format

#### RPM Command (Drive Motors)
```
<VESC_ID> CAN_PACKET_SET_RPM <rpm_value> float
```
Example: `"70 CAN_PACKET_SET_RPM 3000.0 float"`

#### Position Command (Steering Motors)
```
<VESC_ID> CAN_PACKET_SET_POS <angle_value> float
```
Example: `"71 CAN_PACKET_SET_POS 180.0 float"`

### Angle Transformation
The steering angle is transformed to the VESC coordinate system:
```
turn_amount = (wheel_angle / 4) + 180
```

This maps the ±180° wheel angle to the VESC's expected range (with safety limits between 135-225).

### Safety Features
- **Angle limiting:** Warns if commanded angle is outside safe range (135° to 225°)
- **RPM scaling:** Multiplies normalized speed by MAX_RPM (6000)

### Usage
```bash
ros2 run <package_name> swerve_control
```

## Theory: Swerve Drive Kinematics

Swerve drive allows omnidirectional movement by independently controlling each wheel's speed and angle. The system can:
- Translate in any direction without changing heading
- Rotate in place
- Combine translation and rotation simultaneously

The inverse kinematics transform desired vehicle motion (Vx, Vy, ω) into individual wheel commands. Each wheel's position relative to the vehicle center determines how rotation affects its velocity vector, creating the characteristic swerve drive behavior.
