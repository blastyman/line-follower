from machine import Pin, PWM
import time

# Motor control pins
IN1 = Pin(2, Pin.OUT)   # Left motor forward
IN2 = Pin(3, Pin.OUT)   # Left motor backward
IN3 = Pin(4, Pin.OUT)   # Right motor backward (reversed)
IN4 = Pin(5, Pin.OUT)   # Right motor forward (reversed)

ENA = PWM(Pin(1))       # Left motor speed
ENB = PWM(Pin(6))       # Right motor speed

ENA.freq(1000)
ENB.freq(1000)

# IR Sensor inputs
IR_L = Pin(9, Pin.IN)
IR_M = Pin(8, Pin.IN)
IR_R = Pin(7, Pin.IN)

# Extra-slow PWM speed
SPEED = 3000  # Adjust if motors don't move at this level

# Motor control helper functions
def left_motor_forward(speed):
    print(f"Left motor forward @ speed: {speed}")
    IN1.high()
    IN2.low()
    ENA.duty_u16(speed)

def left_motor_backward(speed):
    print(f"Left motor backward @ speed: {speed}")
    IN1.low()
    IN2.high()
    ENA.duty_u16(speed)

def left_motor_stop():
    print("Left motor stop")
    IN1.low()
    IN2.low()
    ENA.duty_u16(0)

def right_motor_forward(speed):
    print(f"Right motor forward @ speed: {speed}")
    IN3.low()
    IN4.high()
    ENB.duty_u16(speed)

def right_motor_backward(speed):
    print(f"Right motor backward @ speed: {speed}")
    IN3.high()
    IN4.low()
    ENB.duty_u16(speed)

def right_motor_stop():
    print("Right motor stop")
    IN3.low()
    IN4.low()
    ENB.duty_u16(0)

def stop():
    print("Stop all motors")
    left_motor_stop()
    right_motor_stop()

# Track the last known action
last_action = "stop"

while True:
    left = IR_L.value()
    middle = IR_M.value()
    right = IR_R.value()

    # Debug print for sensor output
    print(f"Sensors - Left: {left}, Middle: {middle}, Right: {right}")

    if middle == 0 and left == 1 and right == 1:
        left_motor_forward(SPEED)
        right_motor_forward(SPEED)
        last_action = "forward"
    elif left == 0:
        left_motor_forward(SPEED)
        right_motor_forward(SPEED)
        last_action = "left"
    elif right == 0:
        left_motor_forward(SPEED)
        right_motor_forward(SPEED)
        last_action = "right"
    else:
        print("Line lost — repeating last action")
        if last_action == "forward":
            left_motor_forward(SPEED)
            right_motor_forward(SPEED)
        elif last_action == "left":
            left_motor_forward(SPEED)
            right_motor_backward(SPEED)
        elif last_action == "right":
            left_motor_backward(SPEED)
            right_motor_forward(SPEED)
        else:
            stop()

    time.sleep(0.05)
