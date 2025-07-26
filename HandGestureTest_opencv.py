import cv2
import mediapipe as mp
import numpy as np
import math

# Load stream from Mac webcam
cap = cv2.VideoCapture(0)

# Mediapipe hand detector setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.7,
                       min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

def calculate_tilt_angle(landmarks):
    # Palm tilt between wrist and middle finger base (landmark 0 and 9)
    x1, y1 = landmarks[0][0], landmarks[0][1]  # wrist
    x2, y2 = landmarks[9][0], landmarks[9][1]  # base of middle finger

    angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    return angle

def is_palm_open(landmarks):
    # Simple check: distance between fingertips and palm base
    tips_ids = [8, 12, 16, 20]
    open_count = 0
    for tip_id in tips_ids:
        dist = np.linalg.norm(np.array(landmarks[tip_id]) - np.array(landmarks[0]))
        if dist > 0.2:  # Threshold for "open"
            open_count += 1
    return open_count >= 3  # 3 or more fingers extended → open palm

while True:
    success, frame = cap.read()
    if not success:
        print("Stream not available")
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            landmark_list = []

            for lm in hand_landmarks.landmark:
                h, w, _ = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                landmark_list.append((lm.x, lm.y))

            # Draw hand landmarks
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Get wrist position (as base hand position)
            hand_x = landmark_list[0][0]
            hand_y = landmark_list[0][1]

            # Calculate tilt angle of palm
            tilt_angle = calculate_tilt_angle(landmark_list)

            # Check if palm is open
            palm_state = "Open" if is_palm_open(landmark_list) else "Closed"

            # Print the results
            print(f"Hand Position: X={hand_x:.2f}, Y={hand_y:.2f}, Tilt Angle: {tilt_angle:.2f}°, Palm: {palm_state}")

    cv2.imshow("Hand Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
