# 🚀🛡️ SYRACUSE
## 🤖 Autonomous Turret System for Drone Detection, Tracking & Real-Time Smart Surveillance

## 🌟 Project Overview

• Real-time vision-based drone detection and tracking system  
• Powered by deep learning and embedded systems  
• Uses YOLOv11-m object detection model  
• Detects drones from a live camera feed  
• Tracks detected drones using a servo-controlled pan–tilt mechanism  
• Pan–tilt system operated using Arduino UNO  
• Uses a low-power laser and on-screen UI indicators to simulate target engagement  
• Designed with safety and legal compliance in mind  
• No physical firing or drone-disabling mechanism involved 


## 🎯 Key Objectives

• Detect drones in real time using deep learning  
• Track drone movement smoothly using pan–tilt control  
• Apply PID control for stable motion tracking  
• Simulate target engagement using laser and UI  
• Build a low-cost and modular surveillance prototype  


## 🧰 Hardware Requirements

• Arduino UNO  
• USB Camera / Webcam  
• MG995 Servo Motors (Pan & Tilt)  
• Servo Bracket Mount  
• Low-Power Laser Module  
• E88 PRO RC Drone (for testing)  
• Jumper Wires & Breadboard  
• Laptop / PC  (With Graphics Card )


## 💻 Software Requirements

• Python 3.x  
• OpenCV  
• Ultralytics YOLO (YOLOv11-m)  
• PyTorch  
• NumPy  
• PySerial  
• Arduino IDE 


## 🏗️ System Architecture

• Camera captures live video feed  
• YOLOv11-m detects drone in each frame  
• Tracking logic calculates drone position  
• PID controller generates servo control signals  
• Commands sent to Arduino via serial communication  
• Arduino controls pan–tilt servos and laser module  


## 🧠 Algorithms Used

• YOLOv11-m for real-time object detection  
• Motion-aware tracking for smooth target following  
• PID control for stable pan–tilt movement  
• Serial communication for software–hardware integration  


## ▶️ How to Run the Project

• Clone the repository  

git clone https://github.com/your-username/drone-detection-tracking.git
cd drone-detection-tracking 

• Install required dependencies

pip install opencv-python numpy torch ultralytics pyserial

• Upload Arduino code using Arduino IDE
• Connect Arduino UNO to system
• Run the Python application

python main.py


📌 Applications

• Smart surveillance systems
• Defense training simulations
• Academic research and learning
• Computer vision demonstrations

📜 License & Disclaimer

• Developed strictly for educational and research purposes
• No physical drone-neutralization or weaponized system included

