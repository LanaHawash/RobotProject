# RobotProject

A mobile robot that detects, classifies, and sorts scattered toys using an OAK-D camera, Raspberry Pi 5, Arduino Uno, and a robotic arm.

---

## Overview

RobotProject is my graduation project. The robot uses computer vision and stereo depth to detect objects, estimate their distance, and prepare them for robotic manipulation.

The software is designed with a modular architecture that separates perception, decision making, and hardware control. The current implementation focuses on building a reliable perception pipeline before integrating navigation, communication, and robotic manipulation.

---

## Features

### Implemented

- Live RGB camera streaming
- Stereo depth estimation
- Depth map visualization
- YOLO object detection
- Per-object distance estimation
- Flask web interface

### Planned

- Custom YOLO model for toy detection
- Object management and tracking
- Raspberry Pi to Arduino communication
- Robot navigation
- Robotic arm control
- Toy sorting

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
- Ultralytics YOLO

---

## System Architecture

```
                 OAK-D Camera
                       │
          RGB Stream + Stereo Depth
                       │
                       ▼
                Raspberry Pi 5
        Object Detection (YOLO)
        Distance Estimation
        Decision Making
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
│
├── robot_project/
│   ├── camera/
│   ├── detection/
│   └── web/
│
├── main.py
├── requirements.txt
└── README.md
```

---

## Current Status

### Completed

- Raspberry Pi 5 configuration
- OAK-D RGB streaming
- Flask web interface
- Stereo depth estimation
- Depth map visualization
- YOLO object detection
- Per-object distance estimation

### In Progress

- Detection module refactoring
- Object management system

### Planned

- Custom YOLO model training
- Toy classification
- Raspberry Pi to Arduino communication
- Robot navigation
- Robotic arm integration

---

## Running the Project

Activate the virtual environment:

```bash
source venv/bin/activate
```

Run the application:

```bash
python main.py
```

Open the web interface:

```
http://<RASPBERRY_PI_IP>:5000
```

---

## Future Development

The next development stages include:

1. Object management and tracking
2. Custom YOLO model training for toy categories
3. Decision-making module
4. Raspberry Pi to Arduino communication
5. Robot navigation
6. Robotic arm control
7. Complete toy sorting workflow

---

## Author

Lana Hawash

Graduation Project