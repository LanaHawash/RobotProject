from flask import Flask, Response
import cv2

from robot_project.camera.pipeline import CameraPipeline



camera = CameraPipeline()

camera.create_rgb()
camera.create_depth()

import threading

threading.Thread(target=camera.debug_depth, daemon=True).start()
camera.start()

app = Flask(__name__)



def generate_frames():
    while camera.is_running():

        frame = camera.rgb_queue.get().getCvFrame()

        _, buffer = cv2.imencode(".jpg", frame)

        frame_bytes = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame_bytes +
            b"\r\n"
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

            import cv2
            _, buffer = cv2.imencode(".jpg", frame)

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                buffer.tobytes() +
                b"\r\n"
            )

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")