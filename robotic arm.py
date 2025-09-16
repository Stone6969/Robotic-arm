import serial
import cv2
import mediapipe as mp
import math

# Enable/Disable Arduino communication
enable_arduino = True

# Connect to Arduino
if enable_arduino:
    ser = serial.Serial('COM7', 115200)  # Replace with your Arduino COM port

# Servo angle limits
angle_limits = {
    "base": (0, 180),
    "shoulder": (30, 150),
    "elbow": (30, 150),
    "gripper": (0, 90),              # 0 = closed, 90 = open
    "wrist_left_right": (0, 180),    # 0 = spin one way, 180 = other way
    "wrist_up_down": (30, 150)       # Tilt movement
}

# Helper functions
def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def map_range(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def calculate_distance(p1, p2):
    return math.hypot(p1.x - p2.x, p1.y - p2.y)

# Convert hand landmarks to servo angles
def landmark_to_servo_angles(landmarks):
    wrist = landmarks.landmark[0]
    index_mcp = landmarks.landmark[5]
    thumb_tip = landmarks.landmark[4]
    index_tip = landmarks.landmark[8]
    pinky_tip = landmarks.landmark[20]

    # Mirrored axis handling: flip horizontal control
    base = map_range(wrist.x, 0.2, 0.8, angle_limits["base"][1], angle_limits["base"][0])
    shoulder = map_range(wrist.y, 0.3, 0.9, angle_limits["shoulder"][1], angle_limits["shoulder"][0])
    elbow = map_range(index_mcp.y, 0.3, 0.9, angle_limits["elbow"][1], angle_limits["elbow"][0])
    wrist_up_down = map_range(index_tip.y, 0.2, 0.8, angle_limits["wrist_up_down"][1], angle_limits["wrist_up_down"][0])

    # Gripper angle
    grip_dist = calculate_distance(thumb_tip, index_tip)
    gripper = map_range(grip_dist, 0.02, 0.15, angle_limits["gripper"][0], angle_limits["gripper"][1])
    gripper = clamp(gripper, *angle_limits["gripper"])

    # Wrist spin based on pinky horizontal position
    pinky_x = pinky_tip.x
    if pinky_x < 0.4:
        wrist_left_right = 180  # One direction
    elif pinky_x > 0.6:
        wrist_left_right = 0    # Other direction
    else:
        wrist_left_right = 90   # Neutral/Stop

    return [
        clamp(base, *angle_limits["base"]),
        clamp(shoulder, *angle_limits["shoulder"]),
        clamp(elbow, *angle_limits["elbow"]),
        gripper,
        wrist_left_right,
        clamp(wrist_up_down, *angle_limits["wrist_up_down"])
    ]

# MediaPipe setup
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
cap = cv2.VideoCapture(0)

with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.7, min_tracking_confidence=0.7) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            continue

        image = cv2.flip(image, 1)  # Flip for mirror view
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )

            hand_landmarks = results.multi_hand_landmarks[0]
            servo_angles = landmark_to_servo_angles(hand_landmarks)

            print("Servo Angles:", servo_angles)

            if enable_arduino:
                ser.write(bytearray(servo_angles))

        cv2.imshow('Gesture Control', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
