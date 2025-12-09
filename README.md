Syracuse: Autonomous Turret System for Drone
Detection & Tracking and Real-time Smart
Surveillance

📌 Project Overview

This project implements a real-time vision-based drone detection and tracking system using deep learning and embedded systems. The system detects drones from a live camera feed using the YOLOv11-m model and continuously tracks the target using a servo-controlled pan–tilt mechanism driven by an Arduino microcontroller. A laser module and on-screen indicator are used to simulate target engagement in a safe and legally compliant manner.

🎯 Objectives

Detect drones in real time using deep learning

Track drone movement smoothly using pan–tilt control

Implement motion-aware tracking with PID control

Simulate target engagement using laser and UI indicators

Develop a low-cost and modular surveillance prototype

🛠️ Hardware Requirements

Arduino UNO

Camera Module / USB Webcam

MG995 Servo Motors (Pan & Tilt)

Servo Bracket Mount

Low-Power Laser Module

E88 PRO RC Drone (for testing)

Jumper Wires, Breadboard

Laptop / PC

💻 Software Requirements

Python 3.x

OpenCV

Ultralytics YOLO (YOLOv11-m)

PyTorch

NumPy

Arduino IDE

⚙️ System Architecture

Camera captures live video

YOLOv11-m detects the drone

Tracking logic computes drone position

PID controller generates movement commands

Commands sent to Arduino via Serial communication

Arduino controls servos and laser module

🧠 Algorithms Used

YOLOv11-m for object detection

Motion-aware tracking

PID control for smooth pan–tilt movement

Serial communication protocol for hardware control

▶️ How to Run the Project
1️⃣ Clone the Repository
git clone https://github.com/your-username/drone-detection-tracking.git
cd drone-detection-tracking

2️⃣ Install Dependencies
pip install opencv-python numpy torch ultralytics pyserial

3️⃣ Upload Arduino Code

Open Arduino IDE

Upload the Arduino sketch to Arduino UNO

4️⃣ Run Python Script
python main.py

✅ Features

Real-time drone detection

Smooth servo tracking

Automatic search mode

FPS display and UI overlay

Safe simulated engagement

⚠️ Limitations

Works best in good lighting conditions

Optimized for single-drone tracking

Performance depends on camera quality

🔮 Future Enhancements

Multi-drone tracking

Integration of thermal or radar sensors

Deployment on embedded edge devices

Improved detection in low-light conditions

📚 Applications

Smart surveillance systems

Defense training simulations

Research and education

Computer vision demonstrations

👨‍💻 Authors

Yuvaraj D

Final Year Project – Department of Computer Science and Design

Canara Engineering College, Mangaluru

📜 License

This project is developed for educational and research purposes only.
No physical drone-disabling mechanisms are included.
