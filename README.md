# RobotProject

An autonomous mobile robot that detects, classifies, approaches, and sorts scattered toys using an OAK-D camera, Raspberry Pi 5, Arduino Uno, and a robotic arm.

---

## Overview

RobotProject is my graduation project. The robot navigates through a room, detects toys using computer vision, estimates their distance using stereo depth, classifies them into predefined categories, and autonomously picks them up with a robotic arm.

The system is designed with a modular architecture that separates perception, decision making, and hardware control.

---

## Features

- Live RGB camera streaming
- Stereo depth estimation
- YOLO object detection
- Custom toy classification
- Distance estimation
- Web dashboard
- Autonomous navigation
- Robotic arm control

---

## Hardware

- Raspberry Pi 5
- Luxonis OAK-D Camera
- Arduino Uno
- Mobile robot chassis
- Robotic arm
- Servo motors
- DC motors

---

## Software Stack

- Python
- DepthAI
- OpenCV
- Flask
- NumPy
- YOLO

---

## System Architecture

```

                 OAK-D Camera
                       │
        RGB + Depth + YOLO Inference
                       │
                       ▼
                Raspberry Pi 5
          High-Level Decision Making
                       │
                Serial Communication
                       │
                       ▼
                 Arduino Uno
        Low-Level Motor Control
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
     Drive Motors            Robotic Arm

```

---

## Project Structure

```

RobotProject/

robot_project/
camera/
web/

models/

main.py

requirements.txt

README.md

```

---

## Current Status

✅ Raspberry Pi configured

✅ OAK-D RGB streaming

✅ Flask web interface

🔄 Stereo depth implementation

⬜ YOLO integration

⬜ Custom toy dataset

⬜ Training

⬜ Autonomous navigation

⬜ Robotic arm integration

---

## Running the Project

Activate the virtual environment:

```bash
source venv/bin/activate
```

Run:

```bash
python main.py
```

Open:

```
http://<RASPBERRY_PI_IP>:5000
```

---

## Author

Lana Hawash

Graduation Project