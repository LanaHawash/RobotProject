# RobotProject

A mobile robot that detects, classifies, approaches, and sorts scattered toys using an OAK-D camera, Raspberry Pi 5, Arduino Uno, mobile platform, and robotic arm.

---

## Overview

RobotProject is my graduation project.

The robot uses computer vision, stereo depth, and embedded control to detect toys, classify them, estimate their distance, navigate toward them, and prepare them for robotic manipulation and sorting.

The software follows a modular architecture that separates:

* Camera processing
* Object detection
* Depth estimation
* Object management
* Decision making
* Hardware communication
* Motor and robotic arm control

The current system includes a working OAK-D perception pipeline and verified USB serial communication between the Raspberry Pi and Arduino Uno.

---

## Features

### Implemented

* Live RGB camera streaming
* Stereo depth estimation
* Depth map visualization
* Custom YOLO toy detection
* Toy classification
* Per-object distance estimation
* Object selection and management
* Flask web interface
* Raspberry Pi to Arduino USB serial communication
* Arduino connection test using `PING` and `PONG`
* Modular Python project architecture

### In Progress

* Motor command integration
* Robot movement control
* Object tracking improvements
* Target approach logic
* Arduino motor-control firmware

### Planned

* Autonomous navigation
* Obstacle avoidance
* Robotic arm control
* Toy pickup sequence
* Toy sorting by category
* Full autonomous robot workflow

---

## Toy Classes

The custom YOLO model currently detects the following categories:

* `toy_car`
* `animal`
* `building_block`

---

## Hardware

* Raspberry Pi 5
* Luxonis OAK-D camera
* Arduino Uno
* L298N motor driver
* PCA9685 16-channel servo driver
* MPU-6050 IMU
* Four DC TT motors
* Four-wheel mobile robot chassis
* Robotic arm
* Servo motors
* External motor and servo power supplies

---

## Software Stack

* Python 3
* DepthAI
* OpenCV
* Flask
* NumPy
* Ultralytics YOLO
* PySerial
* Arduino C++
* Git and GitHub

---

## System Architecture

```text
                 OAK-D Camera
                       │
          RGB Stream + Stereo Depth
                       │
                       ▼
                Raspberry Pi 5
        ┌──────────────────────────┐
        │ Custom YOLO Detection    │
        │ Toy Classification       │
        │ Distance Estimation      │
        │ Object Management        │
        │ Decision Making          │
        └──────────────────────────┘
                       │
              USB Serial Commands
                       │
                       ▼
                 Arduino Uno
        ┌──────────────────────────┐
        │ Low-Level Motor Control  │
        │ Servo Control            │
        │ Sensor Communication     │
        └──────────────────────────┘
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
     L298N Driver             PCA9685 Driver
          │                         │
          ▼                         ▼
     Drive Motors            Robotic Arm
```

---

## Project Structure

```text
RobotProject/
│
├── robot_project/
│   ├── camera/
│   │   └── Camera pipeline and depth processing
│   │
│   ├── detection/
│   │   └── YOLO detection and object processing
│   │
│   ├── hardware/
│   │   └── Raspberry Pi and Arduino communication
│   │
│   ├── web/
│   │   └── Flask web interface and video streaming
│   │
│   ├── world/
│   │   └── Object management and world state
│   │
│   └── config.py
│
├── models/
│   └── oak/
│       └── best.pt
│
├── main.py
├── requirements.txt
├── .gitignore
└── README.md
```

---

## Current Status

### Completed

* Raspberry Pi 5 setup and configuration
* Python virtual environment
* OAK-D device configuration
* Live RGB streaming
* Flask browser interface
* Stereo depth generation
* RGB and depth alignment
* Depth map visualization
* Custom YOLO model training
* YOLO model integration
* Toy classification
* Bounding-box visualization
* Per-object distance estimation
* Object management implementation
* Raspberry Pi to Arduino USB connection
* Bidirectional serial communication test

The Raspberry Pi successfully sends:

```text
PING
```

The Arduino responds with:

```text
PONG
```

The Arduino is detected on the Raspberry Pi as:

```text
/dev/ttyACM0
```

The communication baud rate is:

```text
115200
```

### In Progress

* Connecting the Arduino to the L298N motor driver
* Implementing movement commands
* Testing forward, backward, left, right, and stop operations
* Improving target tracking
* Integrating robot approach behavior

### Next Milestone

The next milestone is:

```text
Raspberry Pi decision logic
        ↓
USB serial motor command
        ↓
Arduino Uno
        ↓
L298N motor driver
        ↓
Robot movement
```

---

## Installation

Clone the repository:

```bash
git clone https://github.com/LanaHawash/RobotProject.git
cd RobotProject
```

Create a virtual environment:

```bash
python3 -m venv venv
```

Activate the virtual environment:

```bash
source venv/bin/activate
```

Install the required Python packages:

```bash
pip install -r requirements.txt
```

---

## Model Setup

The custom YOLO model must be placed at:

```text
models/oak/best.pt
```

The model file may not be included in the Git repository because trained weight files can be large.

---

## Running the Vision System

Activate the virtual environment:

```bash
source venv/bin/activate
```

Run the application:

```bash
python main.py
```

Open the web interface:

```text
http://<RASPBERRY_PI_IP>:5000
```

Example:

```text
http://192.168.68.101:5000
```

---

## Testing Arduino Communication

Connect the Arduino Uno to the Raspberry Pi using a USB cable.

Check that the Arduino is detected:

```bash
ls /dev/ttyACM*
```

Expected output:

```text
/dev/ttyACM0
```

Run the serial communication test:

```bash
python robot_project/hardware/test_arduino_serial.py
```

Expected output:

```text
Connected to Arduino on /dev/ttyACM0
Arduino startup: ARDUINO_READY
Arduino response: PONG
Raspberry Pi <-> Arduino communication works.
```

---

## Arduino Test Firmware

The current Arduino firmware verifies bidirectional communication:

```cpp
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ARDUINO_READY");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "PING") {
      Serial.println("PONG");
    } else {
      Serial.println("UNKNOWN_COMMAND");
    }
  }
}
```

---

## Development Roadmap

1. Connect Arduino Uno to the L298N motor driver
2. Implement serial motor commands
3. Test manual robot movement
4. Integrate the MPU-6050 IMU
5. Improve object tracking
6. Implement target approach logic
7. Integrate the robotic arm
8. Implement toy pickup
9. Implement toy category sorting
10. Complete the autonomous workflow

---

## Safety Notes

* The Raspberry Pi uses a separate power supply.
* Motors are powered through the L298N motor driver.
* Servos use a separate regulated power supply.
* The Arduino communicates with the Raspberry Pi through USB.
* Motor and servo power must not be supplied directly from the Raspberry Pi.
* All control-system grounds must share a common ground where required.

---

## Author

**Lana Hawash**

Graduation Project
