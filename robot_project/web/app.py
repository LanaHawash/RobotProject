from flask import Flask, Response
import cv2

from robot_project.camera.pipeline import CameraPipeline

camera = CameraPipeline()

camera.create_rgb()

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
            <img src="/video">
        </body>
    </html>
    """


@app.route("/video")
def video():

    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )
