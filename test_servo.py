import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

# Setup I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize two PCA9685 boards with different addresses
pca1 = PCA9685(i2c, address=0x40)  # Shoulder
pca2 = PCA9685(i2c, address=0x41)  # Elbow + Wrist + Gripper

pca1.frequency = 60
pca2.frequency = 60

# Initialize ServoKit for each PCA board
kit1 = ServoKit(channels=16, address=0x40)  # Shoulder
kit2 = ServoKit(channels=16, address=0x41)  # Elbow + Wrist + Gripper

# Move shoulder (two mirrored servos on channels 0 and 1 of PCA1)
def move_shoulder_range(start, end, delay_sec):
    step = 1 if end > start else -1
    for angle in range(start, end + step, step):
        kit1.servo[0].angle = angle
        kit1.servo[1].angle = 180 - angle  # Mirror
        time.sleep(delay_sec)

# Move elbow (channel 0 of PCA2)
def move_elbow_range(start, end, delay_sec):
    step = 1 if end > start else -1
    for angle in range(start, end + step, step):
        kit2.servo[0].angle = angle
        time.sleep(delay_sec)

# Move wrist1 (channel 1 of PCA2)
def move_wrist1_range(start, end, delay_sec):
    step = 1 if end > start else -1
    for angle in range(start, end + step, step):
        kit2.servo[1].angle = angle
        time.sleep(delay_sec)

# Move wrist2 (channel 2 of PCA2)
def move_wrist2_range(start, end, delay_sec):
    step = 1 if end > start else -1
    for angle in range(start, end + step, step):
        kit2.servo[2].angle = angle
        time.sleep(delay_sec)

# Move gripper (end effector) on channel 3 of PCA2
def move_gripper(open_angle, close_angle, delay_sec):
    # Close
    for angle in range(open_angle, close_angle + 1):
        kit2.servo[3].angle = angle
        time.sleep(delay_sec)
    time.sleep(0.5)
    # Open
    for angle in range(close_angle, open_angle - 1, -1):
        kit2.servo[3].angle = angle
        time.sleep(delay_sec)

# Main motion loop
while True:
    move_shoulder_range(35, 80, 0.025)
    time.sleep(0.9)

    move_elbow_range(35, 90, 0.01)
    time.sleep(0.9)

    move_wrist1_range(45, 100, 0.01)
    time.sleep(0.9)

    move_wrist2_range(0, 180, 0.01)
    time.sleep(0.9)

    move_gripper(30, 90, 0.01)  # Example gripper open/close range

    move_wrist2_range(180, 0, 0.01)
    time.sleep(0.9)

    move_wrist1_range(100, 45, 0.01)
    time.sleep(0.9)

    move_elbow_range(90, 35, 0.01)
    time.sleep(0.9)

    move_shoulder_range(80, 35, 0.025)
    time.sleep(0.9)