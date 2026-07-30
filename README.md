# RobotProject

RobotProject is an autonomous mobile sorting robot developed as a graduation project. The robot detects scattered toy objects, selects and approaches a target, positions itself for pickup, grabs the object with a robotic arm, returns to its starting position, and prepares to deliver the object to its assigned destination bin.

The system combines an OAK-D depth camera, a Raspberry Pi 5, an Arduino Uno, an MPU-6050 IMU, an ultrasonic sensor, a four-wheel drive platform, and a servo-controlled robotic arm.

## Project Objectives

The complete robot is intended to:

1. Detect and classify a toy object.
2. Confirm that the same object remains visible across multiple frames.
3. Save and lock the object's assigned destination.
4. Align with and approach the selected object.
5. Stop at a safe pickup distance.
6. Pick up and continue holding the object.
7. Return to the starting position using recorded movement history.
8. Turn toward the destination-bin area.
9. Find and approach the correct bin.
10. Release the object only after reaching that bin.
11. Move away from the bin, reset its cycle state, and repeat.

## Current Project Status

The project currently implements the complete object-detection, object-approach, pickup, and return-home portion of the cycle.

The robot can currently:

- Stream RGB and stereo-depth data from the OAK-D camera.
- Detect supported toy classes using a custom Ultralytics YOLO model.
- Track detections between frames using ByteTrack.
- Estimate object distance using aligned depth data.
- Confirm a stable target over several consecutive frames.
- Align the robot with the selected target.
- Approach the target using camera guidance and ultrasonic measurements.
- Perform final pickup positioning.
- Close the robotic arm and hold the object.
- Record outbound movement and turn history.
- Calculate and execute a return route to the starting position.
- Turn from the starting position to face the destination-bin area.
- Expose live camera, detection, navigation, and hardware status through a Flask web interface.

The current autonomous sequence is effectively:

```text
Detect target
    -> confirm target
    -> align with target
    -> approach target
    -> position for pickup
    -> grab object
    -> return to start
    -> face destination bins
```

The robot does not yet navigate to the assigned destination bin. The existing release command must not be used immediately after the robot faces the bins.

## Current Development Stage

The current development priority is destination propagation and destination locking.

The `ObjectSelector` already assigns a destination when it confirms a target. However, the live dictionary passed to `TargetNavigator` currently contains navigation measurements only:

```python
{
    "label": ...,
    "track_id": ...,
    "confidence": ...,
    "center_x": ...,
    "center_y": ...,
    "box_width": ...,
    "box_height": ...,
    "box_area": ...,
    "box_occupancy": ...,
}
```

It must also contain:

```python
"destination": ...
```

After the destination reaches `TargetNavigator`, the navigator must save and lock it before pickup begins. A later camera detection must not replace the destination of the object already being carried.

The intended locked state is:

```python
self.carried_object_label = target["label"]
self.destination_bin = target["destination"]
self.destination_locked = True
```

This state must remain active through pickup, return-home movement, bin search, bin alignment, and bin approach. It should only be cleared after a successful release and final cycle reset.

## Supported Toy Classes

The custom YOLO model currently supports:

- `toy_car`
- `animal`
- `building_block`

The current object selector assigns the detected class as the normal destination. A low-confidence target that requires additional confirmation may be assigned to `discharge`.

A future explicit mapping may use structures such as:

```python
DESTINATION_BINS = {
    "animal": {
        "destination": "animal_bin",
        "color": "red",
    },
    "toy_car": {
        "destination": "toy_car_bin",
        "color": "blue",
    },
    "building_block": {
        "destination": "building_block_bin",
        "color": "green",
    },
}
```

The physical bin colors and HSV ranges must be calibrated using the real environment and lighting conditions.

## System Architecture

The project uses a two-controller architecture.

- The Raspberry Pi performs high-level perception, target selection, navigation decisions, movement-history processing, cycle control, and web-interface tasks.
- The Arduino performs low-level motor, sensor, turning, and robotic-arm operations in response to serial commands.

```text
                           OAK-D Camera
                    RGB frames + stereo depth
                                  |
                                  v
                         Raspberry Pi 5
        +--------------------------------------------------+
        | Camera pipeline and frame synchronization         |
        | YOLO object detection and ByteTrack tracking      |
        | Depth estimation and target confirmation          |
        | Camera-guided target alignment                    |
        | Ultrasonic-assisted approach decisions            |
        | Movement-history recording and return planning    |
        | Destination state and sorting-cycle coordination  |
        | Flask monitoring and control interface            |
        +--------------------------------------------------+
                                  |
                         USB serial commands
                                  |
                                  v
                            Arduino Uno
        +--------------------------------------------------+
        | Timed forward and backward movement               |
        | Continuous-forward safety control                 |
        | IMU-assisted left and right turns                 |
        | Ultrasonic distance measurements                  |
        | Pickup positioning sequence                       |
        | Robotic-arm grab and release sequences            |
        +--------------------------------------------------+
                   |               |               |
                   v               v               v
              L298N driver     MPU-6050       PCA9685 driver
                   |          + ultrasonic          |
                   v                               v
              Drive motors                    Robotic arm
```

## Software Architecture

### Camera Layer

`robot_project/camera/`

Responsible for:

- OAK-D device creation and configuration.
- RGB and stereo-depth pipelines.
- Frame-rate calculation.
- Depth-frame processing and alignment.

### Detection Layer

`robot_project/detection/`

Responsible for:

- Loading the custom YOLO model.
- Running object detection and ByteTrack tracking.
- Processing detection metadata.
- Confirming a stable object over multiple frames.
- Assigning the selected object's destination.

Important files:

- `yolo.py`: YOLO loading, inference, and tracking.
- `detector.py`: detection and depth-related processing.
- `object_selector.py`: target confirmation and destination assignment.

### Navigation Layer

`robot_project/navigation/`

Responsible for:

- Reading the current camera-guided navigation target.
- Aligning the robot with the object.
- Managing continuous and timed forward movement.
- Switching to ultrasonic control near the object.
- Completing final pickup positioning.
- Recording linear and rotational movement history.
- Calculating the inverse return route.
- Returning to the starting point.
- Turning to face the destination-bin area.

Important files:

- `target_navigator.py`: current object navigation, pickup, return, and post-return sequence.
- `movement_history.py`: movement recording, simplification, pose estimation, and inverse-route generation.

Future navigation modules may separate bin navigation and full-cycle control from object navigation.

### Hardware Layer

`robot_project/hardware/`

Responsible for:

- Opening and maintaining the Raspberry Pi-to-Arduino serial connection.
- Sending commands and validating Arduino responses.
- Exposing high-level Python methods for movement, turning, distance measurement, pickup, and release.

Important files:

- `serial_controller.py`: base serial connection management.
- `arduino_controller.py`: command-level Raspberry Pi interface.
- `test_arduino_serial.py`: serial communication test.
- `test_movement_imu.py`: movement and IMU test utilities.

### Arduino Firmware

`arduino/robot_controller/robot_controller.ino`

Responsible for:

- Controlling the L298N motor-driver inputs.
- Executing timed movement commands.
- Enforcing continuous-forward safety behavior.
- Reading the ultrasonic sensor.
- Reading and calibrating the MPU-6050.
- Performing angle-controlled turns.
- Controlling the robotic arm through the PCA9685.
- Executing pickup-positioning, grab, and release sequences.

### Web Layer

`robot_project/web/`

Responsible for:

- Running the Flask application.
- Processing frames continuously.
- Publishing raw, annotated, and depth streams.
- Maintaining shared detection and target state.
- Providing navigation and Arduino control endpoints.
- Displaying the robot's current status in the browser.

The web layer currently creates `current_navigation_target` and passes it to `TargetNavigator` through `get_navigation_target()`.

### World Layer

`robot_project/world/`

Contains object and world-state abstractions used to represent detected objects and manage scene information.

## Data Flow

The main runtime data flow is:

```text
OAK-D frame
    -> YOLO detection and ByteTrack tracking
    -> depth and bounding-box measurements
    -> ObjectSelector candidate update
    -> confirmed target and assigned destination
    -> current_navigation_target
    -> TargetNavigator
    -> ArduinoController command
    -> Arduino firmware
    -> motors, sensors, and robotic arm
```

The current architecture needs one correction in this flow:

```text
confirmed destination
    -> current_navigation_target["destination"]
    -> TargetNavigator destination lock
```

Without this correction, the navigator can steer toward the object but cannot preserve the destination required for the later sorting phase.

## Hardware

- Raspberry Pi 5
- Luxonis OAK-D camera
- Arduino Uno
- L298N dual H-bridge motor driver
- PCA9685 16-channel servo driver
- MPU-6050 accelerometer and gyroscope
- Ultrasonic distance sensor
- Four DC TT motors
- Four-wheel mobile chassis
- Multi-servo robotic arm and gripper
- Separate regulated power systems for computing, drive motors, and servos

## Software Stack

- Python 3
- DepthAI
- OpenCV
- Flask
- NumPy
- Ultralytics YOLO
- ByteTrack
- PySerial
- Arduino C++
- Git and GitHub

## Project Structure

```text
RobotProject/
├── arduino/
│   ├── communication_test/
│   │   └── communication_test.ino
│   └── robot_controller/
│       └── robot_controller.ino
├── config/
│   └── trackers/
│       └── bytetrack_robot.yaml
├── robot_project/
│   ├── camera/
│   │   ├── depth.py
│   │   ├── device.py
│   │   ├── fps.py
│   │   └── pipeline.py
│   ├── detection/
│   │   ├── detector.py
│   │   ├── object_selector.py
│   │   └── yolo.py
│   ├── hardware/
│   │   ├── arduino_controller.py
│   │   ├── serial_controller.py
│   │   ├── test_arduino_serial.py
│   │   └── test_movement_imu.py
│   ├── navigation/
│   │   ├── movement_history.py
│   │   └── target_navigator.py
│   ├── web/
│   │   ├── app.py
│   │   └── capture.py
│   ├── world/
│   │   ├── manager.py
│   │   └── object.py
│   └── config.py
├── tools/
│   ├── capture_dataset.py
│   └── split_dataset.py
├── main.py
├── requirements.txt
├── .gitignore
└── README.md
```

The trained model is expected at:

```text
models/oak/best.pt
```

Model weights may be excluded from Git because of their size.

## Raspberry Pi and Arduino Responsibilities

### Raspberry Pi

The Raspberry Pi is responsible for decisions that require perception or application state:

- Camera processing.
- Object detection and tracking.
- Target confirmation.
- Destination assignment and locking.
- Alignment decisions.
- Approach-state transitions.
- Movement-history storage.
- Return-route calculation.
- Sorting-cycle state management.
- Web monitoring and manual controls.

### Arduino

The Arduino is responsible for deterministic hardware actions:

- Motor pin control.
- Timed movement execution.
- IMU-based turns.
- Ultrasonic measurements.
- Servo positioning.
- Pickup-positioning sequences.
- Gripper closing and opening.
- Motor safety stops and command acknowledgements.

This separation keeps computer vision and high-level behavior on Linux while maintaining direct and predictable hardware control on the microcontroller.

## Current Robot Workflow

### Detection and Confirmation

1. The OAK-D provides an RGB frame and aligned depth information.
2. YOLO detects supported toy classes.
3. ByteTrack preserves detection identities between frames.
4. The selector chooses the closest valid detection.
5. The same candidate must remain sufficiently stable for a configured number of frames.
6. The selector creates a confirmed target and assigns its destination.

### Object Navigation

1. `TargetNavigator` reads the latest live target.
2. It calculates horizontal error relative to the image center.
3. It performs small IMU-controlled turns until the object is centered.
4. It begins forward motion while continuing to monitor alignment.
5. Near the object, it uses ultrasonic distance rather than camera depth for final approach control.
6. It stops and starts the pickup-positioning sequence at the configured threshold.

### Pickup and Return

1. The Arduino completes the final physical pickup positioning.
2. The arm closes and the object is held.
3. The Raspberry Pi calculates the robot's estimated outbound pose from movement history.
4. It generates and executes a return route.
5. The robot stops at the starting area.
6. The robot turns to face the destination bins.

### Sorting Phase

The sorting phase is the next major implementation area. It will require:

1. Locked destination state in `TargetNavigator`.
2. Destination-to-bin mapping.
3. Colored-bin recognition using OpenCV HSV masks.
4. Required-bin alignment.
5. Ultrasonic-assisted bin approach.
6. Release only after the state reaches `BIN_REACHED`.
7. Back-away and 180-degree return-to-search behavior.
8. Cycle-state clearing and IMU recalibration.
9. Automatic repetition.

## Development Roadmap

### Stage 1: Destination Propagation and Locking

Current stage.

- Add `destination` to `current_navigation_target`.
- Verify that `get_navigation_target()` returns it.
- Add carried-object and destination state to `TargetNavigator`.
- Save and lock the destination before pickup.
- Verify that later camera updates cannot overwrite the locked destination.

### Stage 2: Hold the Object After Facing the Bins

- Remove the immediate release after `_face_bins()`.
- Stop in a bin-search state.
- Confirm physically that the gripper remains closed.
- Preserve movement history and destination state.

### Stage 3: Stationary Bin Detection

- Add an HSV color detector for the physical bins.
- Detect only the color required by the locked destination.
- Validate contour area, center position, and stability.
- Test this stage without moving the robot.

### Stage 4: Bin Alignment

- Add small left and right alignment corrections.
- Require repeated centered detections before approaching.
- Stop safely whenever the required bin is lost.

### Stage 5: Bin Approach and Release

- Use separate bin-approach calibration values.
- Monitor ultrasonic distance while approaching.
- Stop at the calibrated release distance.
- Release only after confirming `BIN_REACHED`.

### Stage 6: Cycle Completion

- Move backward from the bin.
- Turn 180 degrees toward the object-search area.
- Clear movement history and selected-target state.
- Clear carried-object and destination state.
- Recalibrate the IMU while stationary.
- Start the next sorting cycle automatically.

## Installation

Clone the repository:

```bash
git clone https://github.com/LanaHawash/RobotProject.git
cd RobotProject
```

Create and activate a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

Install the required packages:

```bash
pip install -r requirements.txt
```

Place the trained YOLO model at:

```text
models/oak/best.pt
```

## Running the Application

Activate the virtual environment and start the application:

```bash
source venv/bin/activate
python main.py
```

Open the Flask interface from a browser on the same network:

```text
http://<RASPBERRY_PI_IP>:5000
```

## Arduino Setup

Upload the main firmware from:

```text
arduino/robot_controller/robot_controller.ino
```

Connect the Arduino Uno to the Raspberry Pi through USB. The default serial settings in the project use:

```text
Port: /dev/ttyACM0
Baud rate: 115200
```

Confirm that the device exists:

```bash
ls /dev/ttyACM*
```

Run the serial test:

```bash
python robot_project/hardware/test_arduino_serial.py
```

## Safety and Calibration

- Keep Raspberry Pi power separate from motor and high-current servo power.
- Power the drive motors through the L298N driver.
- Power the servos through a suitable regulated supply and the PCA9685 power input.
- Do not power drive motors or the servo power rail directly from the Raspberry Pi.
- Ensure required grounds are common between the connected control systems.
- Stop the robot before relying on a single uncertain sensor reading.
- Keep the gripper closed if an error occurs while carrying an object.
- Do not clear destination or movement history automatically while an object is still held.
- Recalibrate movement duration, return scaling, pickup distance, and bin distance on the physical robot.
- Keep the robot completely still during IMU calibration.

## Known Limitations

- The destination is assigned by `ObjectSelector` but is not yet included in `current_navigation_target`.
- `TargetNavigator` does not yet save or lock the carried object's destination.
- The robot currently has no colored-bin detector.
- Bin alignment and bin approach are not implemented.
- The current complete cycle does not yet reset and repeat automatically.
- Dead-reckoning return accuracy depends on physical calibration and wheel slip.
- HSV bin ranges will depend on lighting and physical bin material.

## Author

Lana Hawash

Graduation Project
