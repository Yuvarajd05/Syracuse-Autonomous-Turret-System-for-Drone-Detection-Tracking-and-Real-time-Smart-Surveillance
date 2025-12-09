🚀🛡️ SYRACUSE: Autonomous Turret System for Drone Detection & Tracking and Real-Time Smart Surveillance 🤖📡


🌟 Project Overview

This project presents a real-time, vision-based drone detection and tracking system powered by deep learning and embedded systems. Using the YOLOv11-m object detection model, the system detects drones from a live camera feed and continuously tracks them using a servo-controlled pan–tilt mechanism operated by an Arduino UNO.

To ensure safety and legal compliance, the system uses a low-power laser and on-screen UI indicators to simulate target engagement.
❌ No physical firing or drone-disabling mechanism is involved.



🎯 Key Objectives

✅ Detect drones in real time using deep learning

✅ Track drone movement smoothly using pan–tilt control

✅ Apply PID control for stable motion tracking

✅ Simulate target engagement using laser & UI

✅ Build a low-cost, modular surveillance prototype




🧰 Hardware Requirements

🔧 Arduino UNO

📷 USB Camera / Webcam

⚙️ MG995 Servo Motors (Pan & Tilt)

🧱 Servo Bracket Mount

🔴 Low-Power Laser Module

🚁 E88 PRO RC Drone (for testing)

🔌 Jumper Wires & Breadboard

💻 Laptop / PC




💻 Software Requirements

🐍 Python 3.x

📷 OpenCV

🔥 Ultralytics YOLO (YOLOv11-m)

🧠 PyTorch

📊 NumPy

🔌 PySerial

⚙️ Arduino IDE




🏗️ System Architecture

📸 Camera captures live video

🧠 YOLOv11-m detects the drone

🎯 Tracking logic calculates drone position

⚙️ PID controller generates control signals

🔌 Commands sent to Arduino via Serial

🤖 Arduino moves servos & controls laser



🧠 Algorithms Used

🟢 YOLOv11-m – Real-time object detection

🟡 Motion-Aware Tracking – Smooth target following

🔵 PID Control – Stable pan–tilt movement

🔴 Serial Communication – Software ↔ Hardware




▶️ How to Run the Project
1️⃣ Clone the Repository
git clone https://github.com/your-username/drone-detection-tracking.git
cd drone-detection-tracking

2️⃣ Install Dependencies
pip install opencv-python numpy torch ultralytics pyserial

3️⃣ Upload Arduino Code

Open Arduino IDE

Upload the provided .ino file to Arduino UNO

4️⃣ Run the Python Program
python main.py




📌 Applications

🛡️ Smart surveillance systems

🎯 Defense training simulations

🎓 Academic research & learning

🤖 Computer vision demonstrations




📜 License & Disclaimer

📖 Developed strictly for educational and research purposes
🚫 No physical drone-neutralization mechanisms included
