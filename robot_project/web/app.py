

from robot_project.web.capture import save_image, image_count
from robot_project.world.manager import ObjectManager
from flask import Flask, Response
import cv2

from robot_project.camera.pipeline import CameraPipeline

from robot_project.detection.detector import Detector

camera = CameraPipeline()
detector = Detector()
manager = ObjectManager()
camera.create_rgb()
camera.create_depth()

import threading

#threading.Thread(target=camera.debug_depth, daemon=True).start()
camera.start()

app = Flask(__name__)
latest_frame = None


def generate_frames():
    while camera.is_running():

        frame = camera.rgb_queue.get().getCvFrame()
        global latest_frame
        latest_frame = frame.copy()
        depth_frame = camera.depth_queue.get().getCvFrame()

        results, frame = detector.detect(frame, depth_frame)
        manager.update(results)
      
        closest = manager.get_closest_object()

        if closest:
            print(
                f"Closest: {closest.label} "
                f"({closest.distance_mm} mm)"
            )
        depth_frame = camera.depth_queue.get().getCvFrame()
        height, width = depth_frame.shape

        center_x = width // 2
        center_y = height // 2

       
        
        
        _, buffer = cv2.imencode(".jpg", frame)

        frame_bytes = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame_bytes +
            b"\r\n"
        )


def generate_capture_frames():
    global latest_frame

    while camera.is_running():

        frame = camera.rgb_queue.get().getCvFrame()

        latest_frame = frame.copy()

        _, buffer = cv2.imencode(".jpg", frame)

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buffer.tobytes()
            + b"\r\n"
        )

@app.route("/")
def index():

    return """
    <html>
        <body>
            <h2>RobotProject - OAK-D Live Camera</h2>
            <img src="/video" width="640">
        </body>
    </html>
    """


@app.route("/video")
def video():

    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/depth")
def depth():
    def gen():
        while camera.is_running():
            frame = camera.depth_queue.get().getCvFrame()

            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
            frame = frame.astype("uint8")

            frame = cv2.medianBlur(frame, 5)

            
            _, buffer = cv2.imencode(".jpg", frame)

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                buffer.tobytes() +
                b"\r\n"
            )

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/capture")
def capture():

    return f"""
    <html>

    <head>

        <meta http-equiv="refresh" content="2">

    </head>

    <body>

        <h2>Dataset Capture</h2>

        <p>Images Saved: {image_count()}</p>

        <img src="/capture_video" width="800">

        <br><br>

        <a href="/save">

            <button style="font-size:24px;padding:15px;">

                Save Image

            </button>

        </a>

    </body>

    </html>
    """

@app.route("/class/<name>")
def choose_class(name):

    set_class(name)

    return f"""
    <h2>Current class: {name}</h2>

    <a href="/capture">
        Back
    </a>
    """

@app.route("/save")
def save():

    global latest_frame

    if latest_frame is None:
        return "No frame available."

    filename = save_image(latest_frame)

    return f"""
    <html>

    <head>

    <meta http-equiv="refresh" content="0; url=/capture">

    </head>

    <body>

    Saved {filename}

    </body>

    </html>
    """


@app.route("/capture_video")
def capture_video():

    return Response(
        generate_capture_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )