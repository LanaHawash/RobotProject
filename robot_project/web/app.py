from turtle import width

from flask import Flask, Response
import cv2

from robot_project.camera.pipeline import CameraPipeline



camera = CameraPipeline()

camera.create_rgb()
camera.create_depth()

import threading

#threading.Thread(target=camera.debug_depth, daemon=True).start()
camera.start()

app = Flask(__name__)



def generate_frames():
    while camera.is_running():

        frame = camera.rgb_queue.get().getCvFrame()
        depth_frame = camera.depth_queue.get().getCvFrame()
        height, width = depth_frame.shape

        center_x = width // 2
        center_y = height // 2

        distance = depth_frame[center_y, center_x]

        cv2.putText(
             frame,
             f"Distance: {distance} mm",
             (20, 40),
             cv2.FONT_HERSHEY_SIMPLEX,
             1,
             (0, 255, 0),
             2
        )
        
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