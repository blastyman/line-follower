from machine import Pin, PWM
import time

# Motor driver pin assignments (based on L298N or similar H-bridge)
IN1_PIN = 2  # Left motor direction pin 1
IN2_PIN = 3  # Left motor direction pin 2
IN3_PIN = 4  # Right motor direction pin 1
IN4_PIN = 5  # Right motor direction pin 2
ENA_PIN = 6  # Left motor PWM enable
ENB_PIN = 1  # Right motor PWM enable

# Initialize motor direction control pins
IN1 = Pin(IN1_PIN, Pin.OUT)
IN2 = Pin(IN2_PIN, Pin.OUT)
IN3 = Pin(IN3_PIN, Pin.OUT)
IN4 = Pin(IN4_PIN, Pin.OUT)

# Initialize PWM for motor speed control
ENA = PWM(Pin(ENA_PIN))  # Left motor
ENB = PWM(Pin(ENB_PIN))  # Right motor

# Set PWM frequency
PWM_FREQ = 1000  # 1 kHz
ENA.freq(PWM_FREQ)
ENB.freq(PWM_FREQ)

# Max duty cycle for 16-bit PWM
MAX_DUTY = 65535

# Infrared (IR) sensor pins for line following (left, middle, right)
IR_L_PIN = 8
IR_M_PIN = 7
IR_R_PIN = 9

# Initialize IR sensor pins
IR_L = Pin(IR_L_PIN, Pin.IN)
IR_M = Pin(IR_M_PIN, Pin.IN)
IR_R = Pin(IR_R_PIN, Pin.IN)


# --- LOW-LEVEL MOTOR CONTROL FUNCTIONS ---

# Run left motor forward with given PWM duty
def _left_motor_forward_raw(duty):
    IN1.high()
    IN2.low()
    ENA.duty_u16(duty)

# Run left motor backward with given PWM duty
def _left_motor_backward_raw(duty):
    IN1.low()
    IN2.high()
    ENA.duty_u16(duty)

# Stop the left motor
def _left_motor_stop_raw():
    IN1.low()
    IN2.low()
    ENA.duty_u16(0)

# Run right motor forward with given PWM duty
def _right_motor_forward_raw(duty):
    IN4.high()
    IN3.low()
    ENB.duty_u16(duty)

# Run right motor backward with given PWM duty
def _right_motor_backward_raw(duty):
    IN3.high()
    IN4.low()
    ENB.duty_u16(duty)

# Stop the right motor
def _right_motor_stop_raw():
    IN3.low()
    IN4.low()
    ENB.duty_u16(0)


# --- HIGH-LEVEL MOTOR INTERFACES (IN % SPEED) ---

# Control left motor using a percentage-based speed (-100 to 100)
def set_left_motor_percent(speed_percent):
    duty = int(MAX_DUTY * (abs(speed_percent) / 100.0))  # Convert to duty cycle
    duty = max(0, min(MAX_DUTY, duty))  # Clamp to valid range

    if speed_percent > 0:
        _left_motor_forward_raw(duty)
    elif speed_percent < 0:
        _left_motor_backward_raw(duty)
    else:
        _left_motor_stop_raw()

# Control right motor using a percentage-based speed (-100 to 100)
def set_right_motor_percent(speed_percent):
    duty = int(MAX_DUTY * (abs(speed_percent) / 100.0))
    duty = max(0, min(MAX_DUTY, duty))

    if speed_percent > 0:
        _right_motor_forward_raw(duty)
    elif speed_percent < 0:
        _right_motor_backward_raw(duty)
    else:
        _right_motor_stop_raw()

# Adjust both motors' speeds using throttle and steering control
# steering: left/right adjustment (-100 to 100)
# throttle_percent: base forward or reverse speed
def set_motor_speeds_from_steering(steering, throttle_percent):
    throttle_percent = max(-100, min(100, throttle_percent))  # Clamp input

    # Combine steering with throttle to calculate each wheel's speed
    left_final_percent = throttle_percent - steering
    right_final_percent = throttle_percent + steering

    # Clamp individual wheel speeds
    left_final_percent = max(-100, min(100, left_final_percent))
    right_final_percent = max(-100, min(100, right_final_percent))

    set_left_motor_percent(left_final_percent)
    set_right_motor_percent(right_final_percent)

# Stop both motors immediately
def stop_all_motors():
    set_left_motor_percent(0)
    set_right_motor_percent(0)


# --- CONSTANTS AND STATE VARIABLES ---

# Base throttle for forward motion
BASE_THROTTLE_PERCENT = 35
# Throttle used during sharp turns
TURN_THROTTLE_PERCENT = 50
# Recovery speed when line is lost
PIVOT_RECOVERY_THROTTLE_PERCENT = 30

# PID control constants
Kp = 30   # Proportional gain
Ki = 0    # Integral gain
Kd = 5    # Derivative gain

# PID state tracking
integral = 0
previous_error = 0
pid_active_previously = False

# Maps IR sensor combinations to error values for PID control
pid_error_mapping_when_middle_on = {
    (1, 0, 1): 0,    # On track
    (0, 0, 1): -0.5, # Line drifting right
    (1, 0, 0): 0.5,  # Line drifting left
    (0, 0, 0): 0,    # Lost
}

# Last known turn direction to help with recovery logic
last_significant_turn_direction = 0  # -1 = left, 1 = right, 0 = unknown

# Timing to detect if line has been lost
LINE_LOST_TIMEOUT_MS = 1000
last_line_seen_time_ms = time.ticks_ms()

# Timing for PID derivative calculation
last_time_us = time.ticks_us()


# --- MAIN CONTROL LOOP ---

try:
    while True:
        # Calculate time delta for PID
        current_time_us = time.ticks_us()
        dt_us = time.ticks_diff(current_time_us, last_time_us)
        if dt_us <= 0:
            dt_us = 1
        last_time_us = current_time_us
        dt_s = dt_us / 1_000_000.0  # Convert to seconds

        # Read IR sensor states
        s_left = IR_L.value()
        s_middle = IR_M.value()
        s_right = IR_R.value()
        sensor_state = (s_left, s_middle, s_right)

        # All sensors see white — likely line completely lost
        if sensor_state == (0, 0, 0):
            stop_all_motors()
            time.sleep(5)

        # All sensors see black — line lost or at a junction
        if sensor_state == (1, 1, 1):
            pid_active_previously = False
            integral = 0
            previous_error = 0

            if time.ticks_diff(time.ticks_ms(), last_line_seen_time_ms) > LINE_LOST_TIMEOUT_MS:
                # Timeout hit — stop until line is found again
                stop_all_motors()
                while IR_L.value() == 1 and IR_M.value() == 1 and IR_R.value() == 1:
                    stop_all_motors()
                    time.sleep(0.1)
                last_line_seen_time_ms = time.ticks_ms()
                last_time_us = time.ticks_us()
                pid_active_previously = False
                continue
            else:
                # Try to recover by pivoting in last known direction
                if last_significant_turn_direction == -1:
                    set_left_motor_percent(-PIVOT_RECOVERY_THROTTLE_PERCENT)
                    set_right_motor_percent(PIVOT_RECOVERY_THROTTLE_PERCENT)
                elif last_significant_turn_direction == 1:
                    set_left_motor_percent(PIVOT_RECOVERY_THROTTLE_PERCENT)
                    set_right_motor_percent(-PIVOT_RECOVERY_THROTTLE_PERCENT)
                else:
                    # Unknown direction — slow rotate
                    set_left_motor_percent(PIVOT_RECOVERY_THROTTLE_PERCENT * 0.7)
                    set_right_motor_percent(-PIVOT_RECOVERY_THROTTLE_PERCENT * 0.7)
                time.sleep(0.05)
                continue

        # Middle sensor sees black (on line) — use PID control
        elif s_middle == 0:
            last_line_seen_time_ms = time.ticks_ms()
            current_error_for_pid = pid_error_mapping_when_middle_on.get(sensor_state, 0)

            if not pid_active_previously:
                previous_error = current_error_for_pid
                integral = 0
            pid_active_previously = True

            # Track last turn direction
            if current_error_for_pid < -0.1:
                last_significant_turn_direction = -1
            elif current_error_for_pid > 0.1:
                last_significant_turn_direction = 1
            else:
                last_significant_turn_direction = 0

            # PID control calculations
            P_term = Kp * current_error_for_pid

            integral += current_error_for_pid * dt_s
            MAX_INTEGRAL_EFFECT_PID = 30
            if Ki != 0:
                if Ki * integral > MAX_INTEGRAL_EFFECT_PID:
                    integral = MAX_INTEGRAL_EFFECT_PID / Ki
                elif Ki * integral < -MAX_INTEGRAL_EFFECT_PID:
                    integral = -MAX_INTEGRAL_EFFECT_PID / Ki
            I_term = Ki * integral

            derivative = (current_error_for_pid - previous_error) / dt_s
            D_term = Kd * derivative

            previous_error = current_error_for_pid

            # Final steering value from PID
            pid_steering_output = P_term + I_term + D_term

            # Apply PID to motor speeds
            set_motor_speeds_from_steering(steering=pid_steering_output, throttle_percent=BASE_THROTTLE_PERCENT)

        # Middle sensor sees white, but side sensors may still detect line
        elif s_middle == 1:
            last_line_seen_time_ms = time.ticks_ms()
            pid_active_previously = False
            integral = 0
            previous_error = 0

            # Handle sharp turns based on side sensors
            if s_left == 1 and s_right == 0:
                set_left_motor_percent(0)
                set_right_motor_percent(TURN_THROTTLE_PERCENT)
                last_significant_turn_direction = -1
            elif s_left == 0 and s_right == 1:
                set_left_motor_percent(TURN_THROTTLE_PERCENT)
                set_right_motor_percent(0)
                last_significant_turn_direction = 1
            elif s_left == 0 and s_right == 0:
                # Use last direction to decide recovery
                if last_significant_turn_direction == -1:
                    set_left_motor_percent(0)
                    set_right_motor_percent(TURN_THROTTLE_PERCENT * 0.7)
                elif last_significant_turn_direction == 1:
                    set_left_motor_percent(TURN_THROTTLE_PERCENT * 0.7)
                    set_right_motor_percent(0)
                else:
                    # Go slow and straight
                    set_motor_speeds_from_steering(steering=0, throttle_percent=BASE_THROTTLE_PERCENT * 0.5)
            else:
                # Unexpected pattern — stop for safety
                stop_all_motors()

except KeyboardInterrupt:
    # Graceful exit on Ctrl+C
    pass

finally:
    # Always stop motors at the end
    stop_all_motors()
