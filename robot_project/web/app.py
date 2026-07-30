import threading
import time

import cv2

from robot_project.hardware.arduino_controller import (
    ArduinoController,
)
from robot_project.navigation.target_navigator import (
    TargetNavigator,
)
from flask import Flask, Response

from robot_project.camera.fps import FPS
from robot_project.camera.pipeline import CameraPipeline
from robot_project.detection.detector import Detector
from robot_project.detection.object_selector import ObjectSelector
from robot_project.web.capture import image_count, save_image
from robot_project.world.manager import ObjectManager

from robot_project.detection.bin_color_detector import (
    BinColorDetector,
)


app = Flask(__name__)

camera = CameraPipeline()
detector = Detector()
manager = ObjectManager()
selector = ObjectSelector()

fps_counter = FPS()
arduino = ArduinoController()
arduino_error = None

camera.create_rgb()
camera.create_depth()
camera.start()

try:
    arduino.connect()

    if not arduino.ping():
        raise RuntimeError(
            "Arduino did not respond to PING."
        )

except Exception as error:
    arduino_error = str(error)
    print(f"Arduino connection error: {error}")

# Protect shared frames because the processing thread writes them
# while Flask routes read them.
frame_lock = threading.Lock()

latest_raw_frame = None
latest_annotated_frame = None
latest_depth_frame = None

current_fps = 0.0
current_detection_count = 0
current_valid_depth_count = 0

current_closest_object = None
current_navigation_target = None

latest_candidate_status = {
    "label": None,
    "frames_seen": 0,
    "average_confidence": None,
    "average_distance_mm": None,
}

latest_selected_target = None

processing_error = None
def get_navigation_target():
    """
    Return the latest live target used by camera-guided navigation.

    The navigation target is based on bounding-box occupancy rather
    than OAK-D depth.
    """

    with frame_lock:
        if current_navigation_target is None:
            return None

        return current_navigation_target.copy()





def get_raw_frame():
    with frame_lock:
        if latest_raw_frame is None:
            return None

        return latest_raw_frame.copy()

navigator = TargetNavigator(
    arduino=arduino,
    get_target=get_navigation_target,
    get_raw_frame=get_raw_frame,
)

def create_depth_visualization(depth_frame):
    """
    Convert the raw millimeter depth frame into an image suitable
    for viewing in the browser.
    """

    if depth_frame is None:
        return None

    depth_visual = cv2.normalize(
        depth_frame,
        None,
        0,
        255,
        cv2.NORM_MINMAX,
    )

    depth_visual = depth_visual.astype("uint8")
    depth_visual = cv2.medianBlur(depth_visual, 5)

    return depth_visual


def draw_system_information(
    frame,
    fps,
    detection_count,
    valid_depth_count,
):
    """
    Draw basic runtime information on the annotated RGB frame.
    """

    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )

    cv2.putText(
        frame,
        f"Detections: {detection_count}",
        (10, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )

    cv2.putText(
        frame,
        f"Valid depth: {valid_depth_count}",
        (10, 79),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )


def camera_processing_loop():
    """
    This is the only function allowed to consume RGB and depth
    frames from the OAK-D queues.
    """

    global latest_raw_frame
    global latest_annotated_frame
    global latest_depth_frame

    global current_fps
    global current_detection_count
    global current_valid_depth_count

    global current_closest_object
    global current_navigation_target
    global latest_candidate_status
    global latest_selected_target

    global processing_error

    print("Camera processing thread started.")

    last_reported_target = None

    while camera.is_running():
        try:
            rgb_message = camera.rgb_queue.get()
            depth_message = camera.depth_queue.get()

            raw_frame = rgb_message.getCvFrame()
            depth_frame = depth_message.getCvFrame()

            detections, annotated_frame = detector.detect(
                raw_frame,
                depth_frame,
            )

            manager.update(detections)

            closest = manager.get_closest_object()

            selected_target = selector.update(detections)
            candidate_status = selector.get_candidate_status()

            # Build a live navigation target using bounding-box size.
            # Depth remains available for display/debugging, but it is
            # no longer required by the navigation target returned here.
            navigation_target_data = None

            if detections:
                live_target = max(
                    detections,
                    key=lambda detection: (
                        detection["bounding_box"][2]
                        - detection["bounding_box"][0]
                    )
                    * (
                        detection["bounding_box"][3]
                        - detection["bounding_box"][1]
                    ),
                )

                x1, y1, x2, y2 = live_target["bounding_box"]

                box_width = max(0, x2 - x1)
                box_height = max(0, y2 - y1)
                box_area = box_width * box_height

                frame_height, frame_width = raw_frame.shape[:2]
                frame_area = frame_width * frame_height

                box_occupancy = (
                    box_area / frame_area
                    if frame_area > 0
                    else 0.0
                )

                navigation_target_data = {
                    "label": live_target["class"],
                    "track_id": live_target.get("track_id"),
                    "confidence": round(
                        live_target["confidence"],
                        3,
                    ),
                    "center_x": live_target["center"][0],
                    "center_y": live_target["center"][1],
                    "box_width": box_width,
                    "box_height": box_height,
                    "box_area": box_area,
                    "box_occupancy": round(
                        box_occupancy,
                        4,
                    ),
                }

            if selected_target is None:
                selected_target_data = None
                last_reported_target = None
            else:
                selected_target_data = {
                    "label": selected_target.label,
                    "average_confidence": round(
                        selected_target.average_confidence,
                        3,
                    ),
                    "average_distance_mm":
                        selected_target.average_distance_mm,
                    "center_x": selected_target.center_x,
                    "center_y": selected_target.center_y,
                    "destination": selected_target.destination,
                    "confirmation_frames":
                        selected_target.confirmation_frames,
                }

                target_signature = (
                    selected_target.label,
                    selected_target.destination,
                )

                if target_signature != last_reported_target:
                    print(
                        "TARGET CONFIRMED: "
                        f"{selected_target.label} | "
                        f"confidence="
                        f"{selected_target.average_confidence:.2f} | "
                        f"distance="
                        f"{selected_target.average_distance_mm} mm | "
                        f"destination="
                        f"{selected_target.destination}"
                    )

                    last_reported_target = target_signature

            if closest is None:
                closest_data = None
            else:
                closest_data = {
                    "label": closest.label,
                    "confidence": round(closest.confidence, 3),
                    "distance_mm": closest.distance_mm,
                    "center_x": closest.center_x,
                    "center_y": closest.center_y,
                }

            fps = fps_counter.update()

            valid_depth_count = sum(
                1
                for detection in detections
                if detection["distance_mm"] is not None
            )

            depth_visual = create_depth_visualization(depth_frame)

            draw_system_information(
                annotated_frame,
                fps,
                len(detections),
                valid_depth_count,
            )

            with frame_lock:
                latest_raw_frame = raw_frame.copy()
                latest_annotated_frame = annotated_frame.copy()

                if depth_visual is not None:
                    latest_depth_frame = depth_visual.copy()

                current_fps = fps
                current_detection_count = len(detections)
                current_valid_depth_count = valid_depth_count

                current_closest_object = closest_data
                current_navigation_target = (
                    navigation_target_data.copy()
                    if navigation_target_data is not None
                    else None
                )
                latest_candidate_status = candidate_status.copy()
                latest_selected_target = selected_target_data

                processing_error = None

        except Exception as error:
            processing_error = str(error)
            print(f"Camera processing error: {error}")
            time.sleep(0.1)

    print("Camera processing thread stopped.")


def generate_stream(frame_type):
    """
    Send the most recently stored frame to a browser.

    frame_type can be:
        annotated
        raw
        depth
    """

    while True:
        with frame_lock:
            if frame_type == "annotated":
                frame = (
                    latest_annotated_frame.copy()
                    if latest_annotated_frame is not None
                    else None
                )

            elif frame_type == "raw":
                frame = (
                    latest_raw_frame.copy()
                    if latest_raw_frame is not None
                    else None
                )

            elif frame_type == "depth":
                frame = (
                    latest_depth_frame.copy()
                    if latest_depth_frame is not None
                    else None
                )

            else:
                frame = None

        if frame is None:
            time.sleep(0.05)
            continue

        success, buffer = cv2.imencode(".jpg", frame)

        if not success:
            time.sleep(0.01)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buffer.tobytes()
            + b"\r\n"
        )

        # Prevent unnecessary CPU usage if the browser refresh rate
        # is faster than the camera-processing rate.
        time.sleep(0.01)





@app.route("/")
def index():
    return """
    <html>
        <head>
            <title>RobotProject Camera</title>
        </head>

        <body>
            <h2>RobotProject - OAK-D Live Detection</h2>

            <img src="/video" width="640">

            <br><br>

            <a href="/navigation/start">
                <button
                    style="
                        font-size:22px;
                        padding:14px;
                        background:#4CAF50;
                    "
                >
                    Start Target Approach
                </button>
            </a>

            <a href="/navigation/stop">
                <button
                    style="
                        font-size:22px;
                        padding:14px;
                        background:#f44336;
                    "
                >
                    Emergency Stop
                </button>
            </a>

            <p>
                <a href="/navigation/status">
                    Open navigation status
                </a>
            </p>

            <p>
                <a href="/depth">Open depth stream</a>
            </p>

            <p>
                <a href="/capture">Open dataset capture</a>
            </p>

            <p>
                <a href="/status">Open system status</a>
            </p>
        </body>
    </html>
    """  


@app.route("/video")
def video():
    return Response(
        generate_stream("annotated"),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/depth")
def depth():
    return Response(
        generate_stream("depth"),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/capture")
def capture():
    return f"""
    <html>
        <head>
            <title>Dataset Capture</title>
        </head>

        <body>
            <h2>Dataset Capture</h2>

            <p>Images saved: {image_count()}</p>

            <img src="/capture_video" width="640">

            <br><br>

            <a href="/save">
                <button style="font-size:24px;padding:15px;">
                    Save Image
                </button>
            </a>

            <p>
                <a href="/">Back to detection stream</a>
            </p>
        </body>
    </html>
    """


@app.route("/capture_video")
def capture_video():
    return Response(
        generate_stream("raw"),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/save")
def save():
    with frame_lock:
        frame = (
            latest_raw_frame.copy()
            if latest_raw_frame is not None
            else None
        )

    if frame is None:
        return "No camera frame is available yet."

    filename = save_image(frame)

    return f"""
    <html>
        <head>
            <meta http-equiv="refresh" content="1; url=/capture">
        </head>

        <body>
            <h3>Saved {filename}</h3>
        </body>
    </html>
    """


@app.route("/status")
def status():
    with frame_lock:
        fps = current_fps
        detection_count = current_detection_count
        valid_depth_count = current_valid_depth_count

        closest_object = (
            current_closest_object.copy()
            if current_closest_object is not None
            else None
        )

        candidate_status = latest_candidate_status.copy()

        selected_target = (
            latest_selected_target.copy()
            if latest_selected_target is not None
            else None
        )

        navigation_target = (
            current_navigation_target.copy()
            if current_navigation_target is not None
            else None
        )

        error = processing_error

    return {
        "camera_running": camera.is_running(),
        "fps": round(fps, 2),
        "detections": detection_count,
        "detections_with_valid_depth": valid_depth_count,
        "closest_object": closest_object,
        "candidate": candidate_status,
        "selected_target": selected_target,
        "target_confirmed": selected_target is not None,
        "navigation_target": navigation_target,
        "arduino_connected": arduino.is_connected(),
        "arduino_error": arduino_error,
        "navigation": {
            "running": navigator.is_running(),
            "returning": navigator.is_returning(),
            "status": navigator.status,
            "last_action": navigator.last_action,
            "error": navigator.error,
        },
        "processing_error": error,
        "locked_object_class": navigator.locked_object_class,
        "locked_destination": navigator.locked_destination,
        "locked_bin_color": navigator.locked_bin_color,
    
    }

@app.route("/navigation/start")
def start_navigation():
    if not arduino.is_connected():
        return {
            "started": False,
            "error": (
                arduino_error
                or "Arduino is not connected."
            ),
        }, 503

    with frame_lock:
        navigation_target_available = (
            current_navigation_target is not None
        )

        selected_target = (
            latest_selected_target.copy()
            if latest_selected_target is not None
            else None
        )

    if (
        not navigation_target_available
        or selected_target is None
    ):
        return {
            "started": False,
            "error": (
                "No confirmed target is available."
            ),
        }, 409

    if navigator.is_returning():
        return {
            "started": False,
            "error": "The robot is currently returning.",
        }, 409

    try:
        navigator.start(selected_target)
    except RuntimeError as error:
        return {
            "started": False,
            "error": str(error),
        }, 409

    return {
        "started": True,
        "navigation_status": navigator.status,
    }


@app.route("/navigation/stop")
def stop_navigation():
    navigator.stop()

    return {
        "stopped": True,
        "navigation_status": navigator.status,
    }


@app.route("/navigation/return")
def return_to_start():
    if not arduino.is_connected():
        return {
            "started": False,
            "error": (
                arduino_error
                or "Arduino is not connected."
            ),
        }, 503

    if navigator.is_running():
        return {
            "started": False,
            "error": (
                "Target navigation is still running."
            ),
        }, 409

    try:
        navigator.start_return()
    except RuntimeError as error:
        return {
            "started": False,
            "error": str(error),
        }, 409

    return {
        "started": True,
        "navigation_status": navigator.status,
        "return_route": navigator.return_route(),
    }


@app.route("/navigation/status")
def navigation_status():
    with frame_lock:
        navigation_target = (
            current_navigation_target.copy()
            if current_navigation_target is not None
            else None
            
        )
        raw_frame = (
            latest_raw_frame.copy()
            if latest_raw_frame is not None
            else None
        )
        

    return {
        "running": navigator.is_running(),
        "returning": navigator.is_returning(),
        "status": navigator.status,
        "last_action": navigator.last_action,
        "error": navigator.error,
        "arduino_connected": arduino.is_connected(),
        "arduino_error": arduino_error,
        "navigation_target": navigation_target,
        "pickup_distance_cm": navigator.pickup_distance_cm,
        "pickup_pulses": navigator.pickup_pulses,
        "movement_history": navigator.movement_history(),
        "simplified_history": navigator.simplified_history(),
        "return_route": navigator.return_route(),
        "locked_object_class": navigator.locked_object_class,
        "locked_destination": navigator.locked_destination,
        "locked_bin_color": navigator.locked_bin_color,
        "current_bin_target": (
            navigator.current_bin_target.copy()
            if navigator.current_bin_target is not None
            else None
        ),
        
    }

processing_thread = threading.Thread(
    target=camera_processing_loop,
    daemon=True,
)

processing_thread.start()