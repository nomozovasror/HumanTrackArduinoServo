from collections import deque

import cv2
import mediapipe as mp
import serial
import time

ARDUINO_PORT = '/dev/cu.usbmodem101'  #MAC OS
BAUD_RATE = 9600
DEAD_ZONE_BUFFER = 30
ANGLE_STEP = 5
HISTORY_LENGTH = 5

arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

mp_face = mp.solutions.face_detection
face_detector = mp_face.FaceDetection(
    min_detection_confidence=0.8,
    model_selection=1
)

# Kamera
cap = cv2.VideoCapture(0)
angle_history = deque(maxlen=HISTORY_LENGTH)
current_angle = 90
last_sent_angle = 90

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_detector.process(rgb)

    if results.detections:
        for detection in results.detections:
            bbox = detection.location_data.relative_bounding_box
            cx = int((bbox.xmin + bbox.width / 2) * w)

            cv2.line(frame, (w // 3 - DEAD_ZONE_BUFFER, 0),
                     (w // 3 - DEAD_ZONE_BUFFER, h), (100, 100, 255), 2)
            cv2.line(frame, (2 * w // 3 + DEAD_ZONE_BUFFER, 0),
                     (2 * w // 3 + DEAD_ZONE_BUFFER, h), (100, 100, 255), 2)

            if cx < (w // 3 - DEAD_ZONE_BUFFER):
                current_angle = min(180, current_angle + ANGLE_STEP)  # + o'zgartirildi
            elif cx > (2 * w // 3 + DEAD_ZONE_BUFFER):
                current_angle = max(0, current_angle - ANGLE_STEP)  # - o'zgartirildi

            angle_history.append(current_angle)
            break

    if angle_history:
        avg_angle = int(sum(angle_history) / len(angle_history))
    else:
        avg_angle = 90

    if abs(avg_angle - last_sent_angle) >= 5:
        arduino.write(f"{avg_angle}\n".encode())
        last_sent_angle = avg_angle
        print(f"Yuborilgan burchak: {avg_angle}")

    cv2.putText(frame, f"Angle: {avg_angle}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Control", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()