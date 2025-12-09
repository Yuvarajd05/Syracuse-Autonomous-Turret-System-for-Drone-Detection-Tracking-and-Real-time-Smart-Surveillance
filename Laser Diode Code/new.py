import cv2
from ultralytics import YOLO
import controller as cnt     # Your servo + laser control file

# Load YOLO 11m model
model = YOLO(r"D:\Project\Drone Detection\Test result\best (yolo11m).pt")

# Open camera
video = cv2.VideoCapture(0)

# Get frame center for servo tracking
FRAME_W = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
FRAME_H = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

# Sensitivity (tune later)
PAN_SENS = 0.15
TILT_SENS = 0.15

while True:
    ret, frame = video.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)

    results = model(frame, stream=True)

    drone_detected = False
    target_pos = None

    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            # drone class = 0 (change if your dataset has different)
            if cls_id == 0 and conf > 0.5:
                drone_detected = True

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                target_pos = (cx, cy)

                # Draw visuals
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.circle(frame, (cx, cy), 8, (0,0,255), -1)
                cv2.putText(frame, "Drone", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    # ===== LASER AND SERVO CONTROL =====
    if drone_detected and target_pos is not None:
        # Laser ON only during detection
        cnt.laser_on()

        cx, cy = target_pos
        error_x = cx - CENTER_X
        error_y = CENTER_Y - cy

        pan_angle = error_x * PAN_SENS
        tilt_angle = error_y * TILT_SENS

        cnt.move_servo(pan_angle, tilt_angle)

        cv2.putText(frame, "Laser: ON", (20, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    else:
        # Stop everything when no drone
        cnt.laser_off()
        cv2.putText(frame, "Laser: OFF", (20, 460),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

    cv2.imshow("Drone Tracker - YOLO11m", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video.release()
cv2.destroyAllWindows()
