import threading
import time
import atexit
import signal

import cv2
from flask import Flask, Response
from flask_sock import Sock
from robot_project.camera.fps import FPS
from robot_project.camera.pipeline import CameraPipeline
from robot_project.detection.detector import Detector
from robot_project.detection.object_selector import ObjectSelector
from robot_project.hardware.arduino_controller import (
    ArduinoController,
)
from robot_project.navigation.target_navigator import (
    TargetNavigator,
)
from robot_project.navigation.deep_cleaning_target_navigator import (
    DeepCleaningTargetNavigator,
)

from robot_project.navigation.deep_cleaning_navigator import (
    DeepCleaningNavigator,
)
from robot_project.web.capture import image_count, save_image
from robot_project.world.manager import ObjectManager

from robot_project.audio.microphone import Microphone
from robot_project.audio.audio_service import AudioService
from robot_project.audio.speaker import Speaker

from pathlib import Path

import firebase_admin
from firebase_admin import credentials, messaging

app = Flask(__name__)

camera = CameraPipeline()
detector = Detector()
manager = ObjectManager()
selector = ObjectSelector()

fps_counter = FPS()
arduino = ArduinoController()
arduino_error = None

microphone = Microphone()
audio_service = AudioService()
speaker = Speaker()
sock = Sock(app)
firebase_ready = False
FCM_DEVICE_TOKEN = "fV05MFkRTNOhf0hvodcrSJ:APA91bFGGdlKzGEgYT-Cdw8A65EXS6Y3BD_vuFCqkC9SRJYUrmHyzr8j-MC4sFXb5hYw-t3Tkr2NHmhQoy8dHU1E6hXokSZnsqT-9odPjz31sAc-XRFJquk"

audio_thread = None

processing_thread = None

system_start_lock = threading.Lock()
system_started = False

# Protect shared frames because the processing thread writes them
# while Flask routes read them.
frame_lock = threading.Lock()
shutdown_lock = threading.Lock()
shutdown_complete = False

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

# Camera-derived navigation data must be recent.
# One second tolerates an isolated slow frame while preventing
# navigation from continuing with an old image after a camera stall.
CAMERA_DATA_MAX_AGE_SECONDS = 1.0
# ByteTrack can occasionally assign a new ID to the same
# physical object after a temporary detection loss.
TRACK_RECOVERY_CONFIRMATION_FRAMES = 2
TRACK_RECOVERY_MAX_CENTER_SHIFT_PX = 120
TRACK_RECOVERY_MAX_AREA_RATIO = 2.0
latest_camera_update_time = None


def _camera_data_is_fresh_locked() -> bool:
    if latest_camera_update_time is None:
        return False

    camera_data_age = (
        time.monotonic()
        - latest_camera_update_time
    )

    return (
        camera_data_age
        <= CAMERA_DATA_MAX_AGE_SECONDS
    )


def get_navigation_target():
    """
    Return the latest fresh target used by camera-guided
    navigation.

    A previously detected target is rejected if camera
    processing has not produced a recent frame.
    """
    with frame_lock:
        if (
            current_navigation_target is None
            or not _camera_data_is_fresh_locked()
        ):
            return None

        return current_navigation_target.copy()


def get_confirmed_target():
    with frame_lock:
        if (
            latest_selected_target is None
            or not _camera_data_is_fresh_locked()
        ):
            return None

        return latest_selected_target.copy()


def clear_confirmed_target():
    global latest_selected_target
    global current_navigation_target
    global latest_candidate_status

    selector.clear_target()

    with frame_lock:
        latest_selected_target = None
        current_navigation_target = None

        latest_candidate_status = {
           
            "label": None,
            "frames_seen": 0,
            "average_confidence": None,
            "average_distance_mm": None,
        }


def get_raw_frame():
    with frame_lock:
        if (
            latest_raw_frame is None
            or not _camera_data_is_fresh_locked()
        ):
            return None

        return latest_raw_frame.copy()


# IMPORTANT:
# `navigator` remains the ORIGINAL TargetNavigator.
# All /navigation/* behavior continues to use this object unchanged.
navigator = TargetNavigator(
    arduino=arduino,
    get_target=get_navigation_target,
    get_raw_frame=get_raw_frame,
    get_confirmed_target=get_confirmed_target,
    clear_confirmed_target=clear_confirmed_target,
)

# Separate navigator used ONLY when deep cleaning is interrupted
# by a confirmed object. It never replaces `navigator`.
deep_cleaning_sorter = DeepCleaningTargetNavigator(
    arduino=arduino,
    get_target=get_navigation_target,
    get_raw_frame=get_raw_frame,
    get_confirmed_target=get_confirmed_target,
    clear_confirmed_target=clear_confirmed_target,
)

deep_cleaning = DeepCleaningNavigator(
    arduino=arduino,
    get_confirmed_target=get_confirmed_target,
    clear_confirmed_target=clear_confirmed_target,
    sort_object=deep_cleaning_sorter.sort_for_deep_cleaning,
    stop_sorting=deep_cleaning_sorter.stop,
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
    global latest_camera_update_time

    print("Camera processing thread started.")

    last_reported_target = None
    last_reference_track_id = None
    last_reference_center = None
    last_reference_box_area = None

    recovery_candidate_track_id = None
    recovery_candidate_frames = 0

    while (
        not shutdown_complete
        and camera.is_running()
    ):
        try:
            rgb_message = camera.rgb_queue.get()
            depth_message = camera.depth_queue.get()

            frame_received_time = time.monotonic()

            with frame_lock:
                previous_camera_update_time = (
                    latest_camera_update_time
                )

            if (
                previous_camera_update_time is not None
                and (
                    frame_received_time
                    - previous_camera_update_time
                )
                > CAMERA_DATA_MAX_AGE_SECONDS
            ):
                # A long camera gap invalidates the previous
                # confirmation history. Require fresh frames
                # before confirming another target.
                selector.clear_target()
                last_reported_target = None

            raw_frame = rgb_message.getCvFrame()
            depth_frame = depth_message.getCvFrame()

            if shutdown_complete or not camera.is_running():
                break

            detections, annotated_frame = detector.detect(
                raw_frame,
                depth_frame,
            )

            if shutdown_complete:
                break

            manager.update(detections)

            closest = manager.get_closest_object()

            selected_target = selector.update(detections)
            candidate_status = selector.get_candidate_status()

            # Build a live navigation target using bounding-box size.
            # Depth remains available for display/debugging, but it is
            # no longer required by the navigation target returned here.
            navigation_target_data = None

            # While navigation is running, follow only the exact
            # ByteTrack object locked by TargetNavigator.
            #
            # Before navigation starts, follow the exact object
            # confirmed by ObjectSelector so the Start route receives
            # matching selected-target and navigation-target data.
            eligible_detections = detections

            # Preserve the original standalone navigation behavior:
            # when `navigator` is running, follow its locked class exactly
            # as before. During a deep-cleaning object interruption, the
            # separate deep_cleaning_sorter gets the same locked-class
            # filtering.
            active_locked_object_class = None

            if (
                navigator.is_running()
                and navigator.locked_object_class
            ):
                active_locked_object_class = (
                    navigator.locked_object_class
                )
            elif (
                deep_cleaning_sorter.is_running()
                and deep_cleaning_sorter.locked_object_class
            ):
                active_locked_object_class = (
                    deep_cleaning_sorter.locked_object_class
                )

            if active_locked_object_class:
                eligible_detections = [
                    detection
                    for detection in detections
                    if (
                        detection["class"]
                        == active_locked_object_class
                    )
                ]


            if eligible_detections:
                live_target = max(
                    eligible_detections,
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
                    #"track_id": live_target.get("track_id"),
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

            frame_processed_time = time.monotonic()

            with frame_lock:
                latest_raw_frame = raw_frame.copy()
                latest_annotated_frame = annotated_frame.copy()

                latest_depth_frame = (
                    depth_visual.copy()
                    if depth_visual is not None
                    else None
                )

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

                latest_camera_update_time = (
                    frame_processed_time
                )

                processing_error = None

        except Exception as error:
            # Closing the camera interrupts blocked queue reads.
            # This is expected during application shutdown.
            if (
                shutdown_complete
                or not camera.is_running()
            ):
                break

            # Preserve the last successful camera result briefly.
            # If processing does not recover, the freshness timeout
            # automatically prevents navigation from using it.
            with frame_lock:
                processing_error = str(error)

            print(
                f"Camera processing error: {error}"
            )

            time.sleep(0.1)

    print("Camera processing thread stopped.")


def initialize_firebase():
    global firebase_ready

    if firebase_ready:
        return

    service_account_path = (
        Path(__file__).resolve().parents[2]
        / "config"
        / "firebase-service-account.json"
    )

    cred = credentials.Certificate(
        str(service_account_path)
    )

    firebase_admin.initialize_app(cred)

    firebase_ready = True

    print("Firebase notification service ready.")

def send_baby_cry_notification():
    if not firebase_ready:
        print(
            "Baby cry notification skipped: "
            "Firebase is not ready."
        )
        return

    try:
        message = messaging.Message(
            notification=messaging.Notification(
                title="Baby Cry Detected",
                body=(
                    "RoboCare detected possible crying."
                ),
            ),
            data={
                "type": "BABY_CRY_DETECTED",
                "mode": "AUTO_LULLABY",
            },
            token=FCM_DEVICE_TOKEN,
        )

        response = messaging.send(message)

        print(
            "Baby cry notification sent:",
            response,
        )

    except Exception as error:
        print(
            "Firebase notification error:",
            error,
        )

def send_baby_cry_notification_async():
    threading.Thread(
        target=send_baby_cry_notification,
        daemon=True,
    ).start()

def audio_processing_loop():
    print("Audio processing thread started.")

    while not shutdown_complete:
        try:
            if not microphone.is_running():
                time.sleep(0.1)
                continue

            stereo_audio = microphone.read()
            mono_audio = microphone.to_mono(stereo_audio)

            events = audio_service.process_audio(mono_audio)

            for event in events:
                print("AUDIO EVENT:", event)

                # -----------------------------
                # Wake-word acknowledgement
                # -----------------------------
                if event["type"] == "wake_word":
                    print(
                        "Hey Robo detected. "
                        "Playing acknowledgement."
                    )

                    # Prevent the microphones from hearing
                    # the robot's own voice response.
                    microphone.stop()

                    try:
                        speaker.speak("How can I help?")

                    except Exception as error:
                        print(
                            f"Wake-word response error: {error}"
                        )

                    finally:
                        if not shutdown_complete:
                            try:
                                microphone.start()

                                print(
                                    "Wake-word response finished. "
                                    "Listening for command."
                                )

                            except Exception as error:
                                print(
                                    "Microphone restart error "
                                    f"after wake-word response: {error}"
                                )

                    continue

                # -----------------------------
                # Voice commands
                # -----------------------------
                if event["type"] == "command":
                    command = event["command"]

                    print(
                        f"Voice command received: {command}"
                    )

                    try:
                        if command == "start navigation":
                            result = start_navigation()

                        elif command == "start deep cleaning":
                            result = start_deep_cleaning()

                        elif command == "stop":
                            result = emergency_stop()

                        else:
                            print(
                                f"Unknown voice command: {command}"
                            )
                            continue

                        print(
                            "Voice command result:",
                            result,
                        )

                    except Exception as error:
                        print(
                            f"Voice command error: {error}"
                        )

                    continue

                # -----------------------------
                # Baby cry
                # -----------------------------
                if event["type"] != "baby_cry":
                    continue

                print("Baby cry detected.")

                # Send notification in parallel so network latency
                # does not delay lullaby playback.
                send_baby_cry_notification_async()

                print(
                    "Stopping microphone before lullaby."
                )

                microphone.stop()

                try:
                    speaker.play_baby_lullaby()

                except Exception as error:
                    print(
                        f"Baby song playback error: {error}"
                    )

                finally:
                    try:
                        audio_service.reset()
                    except Exception as error:
                        print(
                            f"Audio reset error: {error}"
                        )

                    if not shutdown_complete:
                        try:
                            microphone.start()

                            print(
                                "Baby song finished. "
                                "Microphone restarted."
                            )

                        except Exception as error:
                            print(
                                f"Microphone restart error: {error}"
                            )


                         

        except Exception as error:
            if shutdown_complete:
                break

            print(
                f"Audio processing error: {error}"
            )

            time.sleep(0.1)

    print("Audio processing thread stopped.")

def generate_stream(frame_type):
    """
    Send the most recently stored frame to a browser.

    frame_type can be:
        annotated
        raw
        depth
    """

    while (
        not shutdown_complete
        and camera.is_running()
    ):
        with frame_lock:
            if not _camera_data_is_fresh_locked():
                frame = None

            elif frame_type == "annotated":
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

            <a href="/emergency-stop">
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
            
            <a href="/deep-cleaning/start">
                <button
                    style="
                        font-size:22px;
                        padding:14px;
                        background:#2196F3;
                    "
                >
                    Deep Cleaning
                </button>
            </a>

            <a href="/deep-cleaning/stop">
                <button
                    style="
                        font-size:22px;
                        padding:14px;
                        background:#FF9800;
                    "
                >
                    Stop Deep Cleaning
                </button>
            </a>

            <p>
                <a href="/deep-cleaning/status">
                    Open deep-cleaning status
                </a>
            </p>

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

@sock.route("/audio/talk")
def audio_talk(ws):
    print("Live talk connected.")

    microphone_was_running = microphone.is_running()

    try:
        if microphone_was_running:
            print("Stopping robot microphone for live talk.")
            microphone.stop()

        speaker.start_live_talk()

        while True:
            audio_data = ws.receive()

            if audio_data is None:
                break

            if isinstance(audio_data, bytes):
                speaker.write_live_audio(audio_data)

    except Exception as error:
        print(f"Live talk error: {error}")

    finally:
        speaker.stop_live_talk()

        try:
            audio_service.reset()
        except Exception as error:
            print(f"Audio reset error after talk: {error}")

        if microphone_was_running and not shutdown_complete:
            try:
                microphone.start()
                print("Live talk ended. Robot microphone restarted.")
            except Exception as error:
                print(
                    f"Microphone restart error after live talk: {error}"
                )

        print("Live talk disconnected.")

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
    frame = get_raw_frame()

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
        camera_data_fresh = (
            _camera_data_is_fresh_locked()
        )

        camera_data_age_seconds = (
            time.monotonic()
            - latest_camera_update_time
            if latest_camera_update_time is not None
            else None
        )

        if camera_data_fresh:
            fps = current_fps
            detection_count = current_detection_count
            valid_depth_count = current_valid_depth_count

            closest_object = (
                current_closest_object.copy()
                if current_closest_object is not None
                else None
            )

            candidate_status = (
                latest_candidate_status.copy()
            )

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
        else:
            fps = 0.0
            detection_count = 0
            valid_depth_count = 0

            closest_object = None

            candidate_status = {
                "label": None,
               
                "frames_seen": 0,
                "average_confidence": None,
                "average_distance_mm": None,
            }

            selected_target = None
            navigation_target = None

        error = processing_error

    return {
        "camera_running": camera.is_running(),
        "camera_data_fresh": camera_data_fresh,
        "camera_data_age_seconds": (
            round(camera_data_age_seconds, 3)
            if camera_data_age_seconds is not None
            else None
        ),
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

    if deep_cleaning.is_running():
        return {
            "started": False,
            "error": (
                "Deep cleaning is active. "
                "Stop it before starting target navigation."
            ),
        }, 409 

    if navigator.is_returning():
        return {
            "started": False,
            "error": "The robot is currently returning.",
        }, 409

    if navigator.is_running():
        return {
            "started": False,
            "error": "Navigation is already running.",
        }, 409
    navigation_target = get_navigation_target()
    selected_target = get_confirmed_target()

    if (
        navigation_target is None
        or selected_target is None
    ):
        return {
            "started": False,
            "error": (
                "No confirmed target is available."
            ),
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

@app.route("/deep-cleaning/start")
def start_deep_cleaning():
    if not arduino.is_connected():
        return {
            "started": False,
            "error": (
                arduino_error
                or "Arduino is not connected."
            ),
        }, 503

    if (
        navigator.is_running()
        or navigator.is_returning()
    ):
        return {
            "started": False,
            "error": (
                "Target-search navigation is active. "
                "Stop it before starting deep cleaning."
            ),
        }, 409

    try:
        deep_cleaning.start()
    except RuntimeError as error:
        return {
            "started": False,
            "error": str(error),
        }, 409

    return {
        "started": True,
        **deep_cleaning.get_status(),
    }


@app.route("/deep-cleaning/stop")
def stop_deep_cleaning():
    deep_cleaning.stop()

    return {
        "stopped": True,
        **deep_cleaning.get_status(),
    }


@app.route("/deep-cleaning/status")
def deep_cleaning_status():
    return deep_cleaning.get_status()

@app.route("/emergency-stop")
def emergency_stop():
    navigator.stop()
    deep_cleaning.stop()

    try:
        arduino.stop()
    except Exception as error:
        return {
            "stopped": False,
            "error": str(error),
        }, 500

    return {
        "stopped": True,
        "target_navigation": navigator.status,
        "deep_cleaning": deep_cleaning.get_status(),
    }
    
@app.route("/navigation/status")
def navigation_status():
    navigation_target = get_navigation_target()

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



@app.get("/audio/lullaby/play")
def play_lullaby():
    error = None

    try:
        microphone.stop()
        speaker.play_baby_lullaby()

    except Exception as exc:
        error = exc

    finally:
        try:
            audio_service.reset()
        except Exception as reset_error:
            print(
                f"Audio reset error: {reset_error}"
            )

        if not shutdown_complete:
            try:
                microphone.start()

            except Exception as restart_error:
                print(
                    f"Microphone restart error: "
                    f"{restart_error}"
                )

                if error is None:
                    error = restart_error

    if error is not None:
        return {
            "success": False,
            "error": str(error),
        }, 500

    return {
        "success": True,
        "message": "Lullaby played successfully.",
    }
        
def start_system():
    """
    Start the physical robot hardware once.

    Importing this Flask module does not call this function.
    The real application entry point calls it explicitly.
    """
    global system_started
    global processing_thread
    global arduino_error
    global audio_thread
   

    with system_start_lock:
        if system_started:
            return

        if shutdown_complete:
            raise RuntimeError(
                "The robot system has already been shut down."
            )

        print("RobotProject startup started.")

        camera.create_rgb()
        camera.create_depth()
        camera.start()
        initialize_firebase()
        microphone.start()

        arduino_error = None

        try:
            arduino.connect()

            if not arduino.ping():
                raise RuntimeError(
                    "Arduino did not respond to PING."
                )

        except Exception as error:
            arduino_error = str(error)

            print(
                f"Arduino connection error: {error}"
            )

        processing_thread = threading.Thread(
            target=camera_processing_loop,
            daemon=False,
        )

        audio_thread = threading.Thread(
            target=audio_processing_loop,
            daemon=False,
        )

        processing_thread.start()
        audio_thread.start()

        atexit.register(shutdown_system)

        if (
            threading.current_thread()
            is threading.main_thread()
        ):
            signal.signal(
                signal.SIGINT,
                handle_shutdown_signal,
            )

            signal.signal(
                signal.SIGTERM,
                handle_shutdown_signal,
            )

        system_started = True

        print("RobotProject startup complete.")

def shutdown_system():
    """
    Stop robot activity and release hardware resources.

    The flag prevents cleanup from running twice through both
    the signal handler and atexit.
    """
    global shutdown_complete

    with shutdown_lock:
        if shutdown_complete:
            return

        shutdown_complete = True

    print("RobotProject shutdown started.")

    # Stop navigation and robot movement first.
    try:
        navigator.stop()
    except Exception as error:
        print(
            f"Navigation shutdown warning: {error}"
        )

    try:
        deep_cleaning.stop()
    except Exception as error:
        print(
            f"Deep-cleaning shutdown warning: {error}"
        )
    # Stop the OAK-D pipeline so queue reads can exit.
    try:
        camera.close()
    except Exception as error:
        print(
            f"Camera shutdown warning: {error}"
        )

    try:
        microphone.stop()
    except Exception as error:
        print(
            f"Microphone shutdown warning: {error}"
        )    

    # Allow the navigation thread to stop before closing
    # the Arduino serial connection.
    navigation_thread = navigator.navigation_thread

    if (
        navigation_thread is not None
        and navigation_thread.is_alive()
        and navigation_thread
        is not threading.current_thread()
    ):
        navigation_thread.join(timeout=5.0)
    
    # Deep-cleaning object sorting uses a separate TargetNavigator
    # instance. Wait for its thread too before closing Arduino serial.
    deep_cleaning_sort_thread = (
        deep_cleaning_sorter.navigation_thread
    )

    if (
        deep_cleaning_sort_thread is not None
        and deep_cleaning_sort_thread.is_alive()
        and deep_cleaning_sort_thread
        is not threading.current_thread()
    ):
        deep_cleaning_sort_thread.join(timeout=5.0)

    # Allow the camera-processing thread to finish.
    if (
        processing_thread is not None
        and processing_thread.is_alive()
        and processing_thread is not threading.current_thread()
    ):
        processing_thread.join(timeout=10.0)

        if processing_thread.is_alive():
            print(
                "WARNING: Camera processing thread "
                "did not stop within 10 seconds."
            )
        else:
            print("Camera processing thread joined successfully.")

    cleaning_thread = deep_cleaning.cleaning_thread

    if (
        cleaning_thread is not None
        and cleaning_thread.is_alive()
        and cleaning_thread
        is not threading.current_thread()
    ):
        cleaning_thread.join(timeout=3.0)



    if (
        audio_thread is not None
        and audio_thread.is_alive()
        and audio_thread is not threading.current_thread()
    ):
        audio_thread.join(timeout=3.0)

        
    # Send one final motor stop before closing serial.
    try:
        arduino.stop()
    except Exception as error:
        print(
            f"Arduino STOP warning: {error}"
        )

    try:
        arduino.close()
    except Exception as error:
        print(
            f"Arduino close warning: {error}"
        )

    print("RobotProject shutdown complete.")


def handle_shutdown_signal(
    signal_number,
    current_frame,
):
    shutdown_system()
    raise SystemExit(0)