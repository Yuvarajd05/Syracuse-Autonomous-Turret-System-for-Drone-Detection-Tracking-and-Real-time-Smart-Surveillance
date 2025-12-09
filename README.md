🚀🛡️ SYRACUSE: Autonomous Turret System for Drone Detection & Tracking and Real-Time Smart Surveillance 🤖📡

🌟 Project Overview

This project presents a real-time vision-based drone detection and tracking system powered by deep learning and embedded systems. Using the YOLOv11-m object detection model, the system detects drones from a live camera feed and continuously tracks them using a servo-controlled pan–tilt mechanism operated by an Arduino UNO.

To ensure safety and legality, the project uses a low-power laser and on-screen indicators to simulate target engagement—no physical firing or drone disabling is involved.

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
🚁 E88 PRO RC Drone (Testing Purpose)
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
➡️ YOLOv11-m detects the drone
➡️ Tracking logic calculates drone position
➡️ PID controller generates control signals
➡️ Commands sent to Arduino via Serial
➡️ Arduino moves servos & controls laser

🧠 Algorithms Used

🟢 YOLOv11-m – Real-time object detection
🟡 Motion-Aware Tracking – Smooth target following
🔵 PID Control – Stable pan–tilt movement
🔴 Serial Communication – Software ↔ Hardware

▶️ How to Run the Project
1️⃣ Clone the Repository
git clone https://github.com/your-username/drone-detection-tracking.git
cd drone-detection-tracking

2️⃣ Install Python Dependencies
pip install opencv-python numpy torch ultralytics pyserial

3️⃣ Upload Arduino Code

🔹 Open Arduino IDE
🔹 Upload the provided .ino file to Arduino UNO

4️⃣ Run the Python Program
python main.py

✨ Key Features

🚀 Real-time drone detection
🎯 Smooth and accurate tracking
🔄 Automatic search mode
📊 FPS display & UI overlays
🔴 Safe laser-based target indication

⚠️ Limitations

⚡ Works best in good lighting conditions
🚁 Optimized for single-drone tracking
📷 Performance depends on camera quality

🔮 Future Enhancements

🔹 Multi-drone tracking support
🌡️ Integration of thermal / radar sensors
📦 Deployment on edge devices (Jetson, Raspberry Pi)
🌙 Improved low-light detection

📌 Applications

🛡️ Smart surveillance systems
🎯 Defense training simulations
🎓 Academic research & learning
🤖 Computer vision demos

👨‍💻 Author

👤 Yuvaraj D
🎓 Final Year Project
🏫 Department of Computer Science & Design
🏛️ Canara Engineering College, Mangaluru

📜 License & Disclaimer

📖 This project is developed strictly for educational & research purposes.
🚫 No physical drone-neutralization mechanisms are included.
