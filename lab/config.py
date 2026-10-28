# ============================================================
# MOTOR GPIO CONFIGURATION
# ============================================================

# BCM GPIO numbering

MOTOR_A = {
    "forward": 12,
    "backward": 18
}

MOTOR_B = {
    "forward": 13,
    "backward": 19
}


# ============================================================
# ============================================================
# MOTOR GPIO CONFIGURATION
# ============================================================

# BCM GPIO numbering

MOTOR_A = {
    "forward": 12,
    "backward": 18
}

MOTOR_B = {
    "forward": 13,
    "backward": 19
}


# ============================================================
# MOTOR PHYSICAL ORIENTATION
# ============================================================

# Logical positive speed = physical forward of the rover

MOTOR_A_SIGN = 1.0
MOTOR_B_SIGN = -1.0


# ============================================================
# ENCODERS
# ============================================================

PULSES_PER_REV_A = 700.0
PULSES_PER_REV_B = 700.0

WHEEL_DIAMETER_CM = 4.8


# ============================================================
# CONTROL LOOP
# ============================================================

CONTROL_INTERVAL = 0.10

# Used by rover.forward(), rover.backward(), etc.
MANUAL_SPEED = 0.30


# ============================================================
# DRIVING SPEEDS
# ============================================================

FORWARD_RPM = 50.0

# Backward was too fast at 50 rpm
BACKWARD_RPM = 35.0

SPIN_RPM = 35.0


# ============================================================
# TURN CALIBRATION
# ============================================================

# LEFT
LEFT_A_RPM = 50.0
LEFT_B_RPM = 30.0

# RIGHT
# D was turning less than A
RIGHT_A_RPM = 22.0
RIGHT_B_RPM = 50.0


# ============================================================
# FEED-FORWARD CALIBRATION
# ============================================================

# Physical forward
FF_A_FORWARD_PER_RPM = 0.0100
FF_B_FORWARD_PER_RPM = 0.0015

# Physical backward
FF_A_BACKWARD_PER_RPM = 0.0012
FF_B_BACKWARD_PER_RPM = 0.25 / 174.0


# ============================================================
# PI GAINS
# ============================================================

KP_A = 0.0025
KI_A = 0.0015

KP_B = 0.0010
KI_B = 0.0010


# ============================================================
# ANTI-WINDUP
# ============================================================

MAX_INTEGRAL = 150.0# MOTOR PHYSICAL ORIENTATION
# ============================================================

# Logical positive speed = physical forward of the rover

MOTOR_A_SIGN = 1.0
MOTOR_B_SIGN = -1.0


# ============================================================
# ENCODERS
# ============================================================

PULSES_PER_REV_A = 700.0
PULSES_PER_REV_B = 700.0

WHEEL_DIAMETER_CM = 4.8


# ============================================================
# CONTROL LOOP
# ============================================================

CONTROL_INTERVAL = 0.10

# Used by rover.forward(), rover.backward(), etc.
MANUAL_SPEED = 0.30


# ============================================================
# DRIVING SPEEDS
# ============================================================

FORWARD_RPM = 50.0

# Backward was too fast at 50 rpm
BACKWARD_RPM = 35.0

SPIN_RPM = 35.0


# ============================================================
# TURN CALIBRATION
# ============================================================

# LEFT
LEFT_A_RPM = 50.0
LEFT_B_RPM = 30.0

# RIGHT
# D was turning less than A
RIGHT_A_RPM = 22.0
RIGHT_B_RPM = 50.0


# ============================================================
# FEED-FORWARD CALIBRATION
# ============================================================

# Physical forward
FF_A_FORWARD_PER_RPM = 0.0100
FF_B_FORWARD_PER_RPM = 0.0015

# Physical backward
FF_A_BACKWARD_PER_RPM = 0.0012
FF_B_BACKWARD_PER_RPM = 0.25 / 174.0


# ============================================================
# PI GAINS
# ============================================================

KP_A = 0.0025
KI_A = 0.0015

KP_B = 0.0010
KI_B = 0.0010


# ============================================================
# ANTI-WINDUP
# ============================================================

MAX_INTEGRAL = 150.0
