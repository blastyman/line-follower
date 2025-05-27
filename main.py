from machine import Pin, PWM
import time

IN1_PIN = 2
IN2_PIN = 3
IN3_PIN = 4
IN4_PIN = 5
ENA_PIN = 6
ENB_PIN = 1

IN1 = Pin(IN1_PIN, Pin.OUT)
IN2 = Pin(IN2_PIN, Pin.OUT)
IN3 = Pin(IN3_PIN, Pin.OUT)
IN4 = Pin(IN4_PIN, Pin.OUT)

ENA = PWM(Pin(ENA_PIN))
ENB = PWM(Pin(ENB_PIN))

PWM_FREQ = 1000
ENA.freq(PWM_FREQ)
ENB.freq(PWM_FREQ)
MAX_DUTY = 65535

IR_L_PIN = 8
IR_M_PIN = 7
IR_R_PIN = 9

IR_L = Pin(IR_L_PIN, Pin.IN)
IR_M = Pin(IR_M_PIN, Pin.IN)
IR_R = Pin(IR_R_PIN, Pin.IN)


def _left_motor_forward_raw(duty):
    IN1.high()
    IN2.low()
    ENA.duty_u16(duty)


def _left_motor_backward_raw(duty):
    IN1.low()
    IN2.high()
    ENA.duty_u16(duty)


def _left_motor_stop_raw():
    IN1.low()
    IN2.low()
    ENA.duty_u16(0)


def _right_motor_forward_raw(duty):
    IN4.high()
    IN3.low()
    ENB.duty_u16(duty)


def _right_motor_backward_raw(duty):
    IN3.high()
    IN4.low()
    ENB.duty_u16(duty)


def _right_motor_stop_raw():
    IN3.low()
    IN4.low()
    ENB.duty_u16(0)


def set_left_motor_percent(speed_percent):
    duty = int(MAX_DUTY * (abs(speed_percent) / 100.0))
    duty = max(0, min(MAX_DUTY, duty))

    if speed_percent > 0:
        _left_motor_forward_raw(duty)
    elif speed_percent < 0:
        _left_motor_backward_raw(duty)
    else:
        _left_motor_stop_raw()


def set_right_motor_percent(speed_percent):
    duty = int(MAX_DUTY * (abs(speed_percent) / 100.0))
    duty = max(0, min(MAX_DUTY, duty))

    if speed_percent > 0:
        _right_motor_forward_raw(duty)
    elif speed_percent < 0:
        _right_motor_backward_raw(duty)
    else:
        _right_motor_stop_raw()


def set_motor_speeds_from_steering(steering, throttle_percent):
    throttle_percent = max(-100, min(100, throttle_percent))

    left_final_percent = throttle_percent - steering
    right_final_percent = throttle_percent + steering

    left_final_percent = max(-100, min(100, left_final_percent))
    right_final_percent = max(-100, min(100, right_final_percent))

    set_left_motor_percent(left_final_percent)
    set_right_motor_percent(right_final_percent)


def stop_all_motors():
    set_left_motor_percent(0)
    set_right_motor_percent(0)


BASE_THROTTLE_PERCENT = 35
TURN_THROTTLE_PERCENT = 50
PIVOT_RECOVERY_THROTTLE_PERCENT = 30

Kp = 30
Ki = 0
Kd = 5

integral = 0
previous_error = 0
pid_active_previously = False

pid_error_mapping_when_middle_on = {
    (1, 0, 1): 0,
    (0, 0, 1): -0.5,
    (1, 0, 0): 0.5,
    (0, 0, 0): 0,
}

last_significant_turn_direction = 0
LINE_LOST_TIMEOUT_MS = 1000
last_line_seen_time_ms = time.ticks_ms()

last_time_us = time.ticks_us()

try:
    while True:
        current_time_us = time.ticks_us()
        dt_us = time.ticks_diff(current_time_us, last_time_us)
        if dt_us <= 0:
            dt_us = 1
        last_time_us = current_time_us
        dt_s = dt_us / 1_000_000.0

        s_left = IR_L.value()
        s_middle = IR_M.value()
        s_right = IR_R.value()
        sensor_state = (s_left, s_middle, s_right)

        if sensor_state == (0, 0, 0):
            stop_all_motors()
            time.sleep(5)

        if sensor_state == (1, 1, 1):
            pid_active_previously = False
            integral = 0
            previous_error = 0

            if time.ticks_diff(time.ticks_ms(), last_line_seen_time_ms) > LINE_LOST_TIMEOUT_MS:
                stop_all_motors()
                while IR_L.value() == 1 and IR_M.value() == 1 and IR_R.value() == 1:
                    stop_all_motors()
                    time.sleep(0.1)
                last_line_seen_time_ms = time.ticks_ms()
                last_time_us = time.ticks_us()
                pid_active_previously = False
                continue
            else:
                if last_significant_turn_direction == -1:
                    set_left_motor_percent(-PIVOT_RECOVERY_THROTTLE_PERCENT)
                    set_right_motor_percent(PIVOT_RECOVERY_THROTTLE_PERCENT)
                elif last_significant_turn_direction == 1:
                    set_left_motor_percent(PIVOT_RECOVERY_THROTTLE_PERCENT)
                    set_right_motor_percent(-PIVOT_RECOVERY_THROTTLE_PERCENT)
                else:
                    set_left_motor_percent(PIVOT_RECOVERY_THROTTLE_PERCENT * 0.7)
                    set_right_motor_percent(-PIVOT_RECOVERY_THROTTLE_PERCENT * 0.7)
                time.sleep(0.05)
                continue

        elif s_middle == 0:
            last_line_seen_time_ms = time.ticks_ms()
            current_error_for_pid = pid_error_mapping_when_middle_on.get(sensor_state, 0)

            if not pid_active_previously:
                previous_error = current_error_for_pid
                integral = 0
            pid_active_previously = True

            if current_error_for_pid < -0.1:
                last_significant_turn_direction = -1
            elif current_error_for_pid > 0.1:
                last_significant_turn_direction = 1
            else:
                last_significant_turn_direction = 0

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

            pid_steering_output = P_term + I_term + D_term

            set_motor_speeds_from_steering(steering=pid_steering_output, throttle_percent=BASE_THROTTLE_PERCENT)

        elif s_middle == 1:
            last_line_seen_time_ms = time.ticks_ms()
            pid_active_previously = False
            integral = 0
            previous_error = 0

            if s_left == 1 and s_right == 0:
                set_left_motor_percent(0)
                set_right_motor_percent(TURN_THROTTLE_PERCENT)
                last_significant_turn_direction = -1
            elif s_left == 0 and s_right == 1:
                set_left_motor_percent(TURN_THROTTLE_PERCENT)
                set_right_motor_percent(0)
                last_significant_turn_direction = 1
            elif s_left == 0 and s_right == 0:
                if last_significant_turn_direction == -1:
                    set_left_motor_percent(0)
                    set_right_motor_percent(TURN_THROTTLE_PERCENT * 0.7)
                elif last_significant_turn_direction == 1:
                    set_left_motor_percent(TURN_THROTTLE_PERCENT * 0.7)
                    set_right_motor_percent(0)
                else:
                    set_motor_speeds_from_steering(steering=0, throttle_percent=BASE_THROTTLE_PERCENT * 0.5)
            else:
                stop_all_motors()

except KeyboardInterrupt:
    pass
finally:
    stop_all_motors()