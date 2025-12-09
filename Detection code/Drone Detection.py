# ================================================================
# Fix OpenMP conflicts (must be done before any imports)
# ================================================================
import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"

import time
import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

import torch
torch.set_num_threads(1)

from ultralytics import YOLO

import serial
import serial.tools.list_ports


# ================================================================
# Configuration (WITH SMOOTH + MOTION-AWARE TRACKING)
# ================================================================
@dataclass
class Config:
    X_MIN: int = 900
    X_MAX: int = 2100
    Y_MIN: int = 900
    Y_MAX: int = 2100
    X_HOME: int = 1500
    Y_HOME: int = 1500

    MAX_LOST_FRAMES: int = 30
    DETECTION_CONF: float = 0.5
    IOU_THRESHOLD: float = 0.7

    # ---------- PID Tuning ----------
    PID_X_KP: float = 0.9
    PID_X_KI: float = 0.02
    PID_X_KD: float = 0.4

    PID_Y_KP: float = 1.0
    PID_Y_KI: float = 0.02
    PID_Y_KD: float = 0.45

    # ---------- Search Pattern ----------
    SCAN_SPEED: float = 0.05
    SCAN_RADIUS_BASE: float = 120
    SCAN_RADIUS_VAR: float = 60

    MODEL_PATH: str = "C:/Users/vaibh/Downloads/Test2/Test2/best.pt"
    BAUD_RATE: int = 115200


# ================================================================
# Arduino Controller
# ================================================================
class ArduinoController:
    def __init__(self, config: Config):
        self.config = config
        self.ser = None
        self.current_x = config.X_HOME
        self.current_y = config.Y_HOME
        self._connect()

    def _connect(self):
        port = self._find_arduino()
        if not port:
            raise Exception("Arduino not found!")

        self.ser = serial.Serial(port, self.config.BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"✓ Connected to Arduino on {port}")

        self.send_command(self.config.X_HOME, self.config.Y_HOME, False)

    @staticmethod
    def _find_arduino() -> Optional[str]:
        keywords = ["Arduino", "CH340", "USB-SERIAL", "CP210"]
        for p in serial.tools.list_ports.comports():
            if any(k in p.description for k in keywords):
                return p.device
        return None

    def send_command(self, x, y, laser):
        x = int(np.clip(x, self.config.X_MIN, self.config.X_MAX))
        y = int(np.clip(y, self.config.Y_MIN, self.config.Y_MAX))

        self.current_x = x
        self.current_y = y

        cmd = f"X{x} Y{y} L{1 if laser else 0}\n"
        try:
            self.ser.write(cmd.encode())
        except:
            pass

    def home(self):
        self.send_command(self.config.X_HOME, self.config.Y_HOME, False)

    def close(self):
        try:
            self.home()
            self.ser.close()
        except:
            pass


# ================================================================
# PID Controller
# ================================================================
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()

    def update(self, measurement):
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            dt = 0.01

        error = self.setpoint - measurement
        self.integral += error * dt
        self.integral = np.clip(self.integral, -300, 300)
        derivative = (error - self.prev_error) / dt

        output = (
            self.Kp * error +
            self.Ki * self.integral +
            self.Kd * derivative
        )

        self.prev_error = error
        self.prev_time = now
        return output

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()


# ================================================================
# Drone Tracker (FULL VERSION)
# ================================================================
class DroneTracker:
    def __init__(self, config: Config):
        self.config = config
        self.arduino = ArduinoController(config)

        print("Loading YOLO model...")
        self.model = YOLO(config.MODEL_PATH)

        if torch.cuda.is_available():
            print("✓ GPU detected")
            self.device = "cuda"
            self.model.to("cuda")
        else:
            print("⚠ CPU Mode")
            self.device = "cpu"

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise Exception("Camera error!")

        self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.cx = self.w // 2
        self.cy = self.h // 2

        self.pid_x = PIDController(config.PID_X_KP, config.PID_X_KI, config.PID_X_KD, self.cx)
        self.pid_y = PIDController(config.PID_Y_KP, config.PID_Y_KI, config.PID_Y_KD, self.cy)

        self.lost_frames = 0
        self.tracked_id = None
        self.laser_on = False
        self.scan_angle = 0.0

        self.prev_centers = []
        self.prev_track_center = None


    # ============================================================
    # YOLO Detection (with smoothing)
    # ============================================================
    def _detect(self, frame):
        results = self.model.track(
            frame,
            persist=True,
            conf=self.config.DETECTION_CONF,
            iou=self.config.IOU_THRESHOLD,
            tracker="bytetrack.yaml",
            device=self.device,
            half=(self.device == "cuda"),
            verbose=False
        )[0]

        dets = []

        if results.boxes.id is None:
            return dets

        for (box, conf, tid) in zip(
            results.boxes.xyxy.cpu().numpy(),
            results.boxes.conf.cpu().numpy(),
            results.boxes.id.cpu().numpy().astype(int)
        ):
            if conf < self.config.DETECTION_CONF:
                continue

            x1, y1, x2, y2 = map(int, box)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            self.prev_centers.append((cx, cy))
            if len(self.prev_centers) > 4:
                self.prev_centers.pop(0)

            cx = int(np.mean([p[0] for p in self.prev_centers]))
            cy = int(np.mean([p[1] for p in self.prev_centers]))

            dets.append({
                "id": tid,
                "box": (x1, y1, x2, y2),
                "center": (cx, cy),
                "area": (x2 - x1) * (y2 - y1),
                "conf": conf
            })

        return dets


    # ============================================================
    # Target Selection
    # ============================================================
    def _select(self, dets):
        if not dets:
            return None

        if self.tracked_id is not None:
            for d in dets:
                if d["id"] == self.tracked_id:
                    return d

        return max(dets, key=lambda d: d["area"])


    # ============================================================
    # Tracking Mode (FIXED SYNTAX)
    # ============================================================
    def _tracking(self, target):
        self.lost_frames = 0
        self.tracked_id = target["id"]
        self.laser_on = True

        cx, cy = target["center"]

        if self.prev_track_center is None:
            self.prev_track_center = (cx, cy)

        vx = cx - self.prev_track_center[0]
        vy = cy - self.prev_track_center[1]
        self.prev_track_center = (cx, cy)

        speed = math.hypot(vx, vy)

        pred_x = int(cx + vx * 0.8)
        pred_y = int(cy + vy * 0.8)

        if speed < 10:
            px = self.pid_x.update(cx)
            py = self.pid_y.update(cy)
        else:
            px = self.pid_x.update(pred_x)
            py = self.pid_y.update(pred_y)

        if abs(cx - self.cx) < 25:
            px = 0
        if abs(cy - self.cy) < 25:
            py = 0

        px = np.clip(px, -12, 12)
        py = np.clip(py, -12, 12)

        alpha = 0.20
        self.arduino.current_x = (1 - alpha) * self.arduino.current_x + alpha * (self.arduino.current_x + px)
        self.arduino.current_y = (1 - alpha) * self.arduino.current_y + alpha * (self.arduino.current_y + py)

        self.arduino.current_x = np.clip(self.arduino.current_x, self.config.X_MIN, self.config.X_MAX)
        self.arduino.current_y = np.clip(self.arduino.current_y, self.config.Y_MIN, self.config.Y_MAX)


    # ============================================================
    # Search Mode
    # ============================================================
    def _search(self):
        self.lost_frames += 1

        if self.lost_frames > 5:
            self.laser_on = False

        if self.lost_frames > self.config.MAX_LOST_FRAMES:
            self.tracked_id = None
            self.pid_x.reset()
            self.pid_y.reset()

        self.scan_angle += self.config.SCAN_SPEED
        r = self.config.SCAN_RADIUS_BASE + self.config.SCAN_RADIUS_VAR * math.sin(self.scan_angle * 0.3)

        self.arduino.current_x = self.config.X_HOME + r * math.cos(self.scan_angle * 1.2)
        self.arduino.current_y = self.config.Y_HOME + r * math.sin(self.scan_angle * 0.8) * 0.6


    # ============================================================
    # Draw UI
    # ============================================================
    def _draw(self, frame, target, fps):
        cv2.drawMarker(frame, (self.cx, self.cy),
                       (255, 255, 255),
                       cv2.MARKER_CROSS, 30, 2)

        if target:
            x1, y1, x2, y2 = target["box"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(frame, "TRACKING", (10, 50),
                        cv2.FONT_HERSHEY_DUPLEX, 1.3, (0, 255, 0), 3)
        else:
            cv2.putText(frame, "SEARCHING", (10, 50),
                        cv2.FONT_HERSHEY_DUPLEX, 1.3, (0, 0, 255), 3)

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, self.h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


    # ============================================================
    # Main Loop
    # ============================================================
    def run(self):
        print("\n=== Drone Tracker Started ===")

        while True:
            t0 = time.time()
            ret, frame = self.cap.read()
            if not ret:
                break

            dets = self._detect(frame)
            target = self._select(dets)

            if target:
                self._tracking(target)
            else:
                self._search()

            self.arduino.send_command(
                self.arduino.current_x,
                self.arduino.current_y,
                self.laser_on
            )

            fps = 1.0 / max(time.time() - t0, 1e-6)
            self._draw(frame, target, fps)

            cv2.imshow("Drone Tracker", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

            if cv2.waitKey(1) & 0xFF == ord('r'):
                self.arduino.home()

        self.cleanup()


    def cleanup(self):
        print("\nShutting down...")
        self.arduino.close()
        self.cap.release()
        cv2.destroyAllWindows()


# ================================================================
# Main Entry Point
# ================================================================
def main():
    tracker = DroneTracker(Config())
    tracker.run()


if __name__ == "__main__":
    main()
