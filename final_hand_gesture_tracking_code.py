import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

import cv2
import mediapipe as mp
import numpy as np
import math

# Setup I2C and ServoKit
i2c = busio.I2C(board.SCL, board.SDA)
pca1 = PCA9685(i2c, address=0x40)
pca2 = PCA9685(i2c, address=0x41)
pca1.frequency = 60
pca2.frequency = 60

kit1 = ServoKit(channels=16, address=0x40)
kit2 = ServoKit(channels=16, address=0x41)

# Servo angle ranges
SHOULDER_MIN, SHOULDER_MAX = 35, 80
WRIST1_MIN, WRIST1_MAX = 45, 100
WRIST2_MIN, WRIST2_MAX = 0, 180
GRIPPER_OPEN = 30
GRIPPER_CLOSE = 110  # Increased gripper close angle
ELBOW_ANGLE = 45     # Updated elbow angle for easy adjustment

# Mediapipe setup
cap = cv2.VideoCapture(0)
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.7,
                       min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Calculate tilt between wrist and base of fingers
def calculate_tilt_angle(landmarks):
    x1, y1 = landmarks[0][0], landmarks[0][1]
    x2, y2 = landmarks[9][0], landmarks[9][1]
    angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    return angle

# Check if palm is open using distances from wrist to fingertips
def is_palm_open(landmarks):
    tips_ids = [8, 12, 16, 20]
    open_count = 0
    for tip_id in tips_ids:
        dist = np.linalg.norm(np.array(landmarks[tip_id]) - np.array(landmarks[0]))
        if dist > 0.2:
            open_count += 1
    return open_count >= 3

# Linear mapping between ranges
def map_range(value, from_min, from_max, to_min, to_max):
    value = max(min(value, from_max), from_min)
    return to_min + (to_max - to_min) * (value - from_min) / (from_max - from_min)

# Main loop
while True:
    success, frame = cap.read()
    if not success:
        print("Camera not available")
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            landmark_list = [(lm.x, lm.y) for lm in hand_landmarks.landmark]
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            x = landmark_list[0][0]
            y = landmark_list[0][1]
            tilt = calculate_tilt_angle(landmark_list)
            palm_open = is_palm_open(landmark_list)

            # Mapped angles
            shoulder_angle = int(map_range(x, 0.0, 1.0, SHOULDER_MIN, SHOULDER_MAX))
            wrist1_angle = int(map_range(y, 0.5, 1.0, WRIST1_MIN, WRIST1_MAX))
            wrist2_angle = int(map_range(tilt, -130, -70, WRIST2_MIN, WRIST2_MAX))
            gripper_angle = GRIPPER_CLOSE if palm_open else GRIPPER_OPEN

            # Apply servo angles
            kit1.servo[0].angle = shoulder_angle
            kit1.servo[1].angle = 180 - shoulder_angle  # Possibly for shoulder symmetry
            kit2.servo[0].angle = ELBOW_ANGLE
            kit2.servo[1].angle = wrist1_angle
            kit2.servo[2].angle = wrist2_angle
            kit2.servo[3].angle = gripper_angle

            print(f"X={x:.2f} → Shoulder:{shoulder_angle}, "
                  f"Y={y:.2f} → Wrist1:{wrist1_angle}, "
                  f"Tilt={tilt:.1f}° → Wrist2:{wrist2_angle}, "
                  f"Palm={'Open' if palm_open else 'Closed'} → Gripper:{gripper_angle}")

    cv2.imshow("Hand Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
