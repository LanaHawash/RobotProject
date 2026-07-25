from ultralytics import YOLO

from robot_project.config import YOLO_MODEL_PATH


class ObjectDetector:

    def __init__(self):
        if not YOLO_MODEL_PATH.exists():
            raise FileNotFoundError(
                f"YOLO model was not found: {YOLO_MODEL_PATH}"
            )

        print(f"Loading YOLO model: {YOLO_MODEL_PATH}")

        self.model = YOLO(str(YOLO_MODEL_PATH))

    def track(self, frame):
        """
        Detect objects and preserve their identities between frames
        using the ByteTrack tracker.
        """

        results = self.model.track(
            source=frame,
            conf=0.20,
            tracker="config/trackers/bytetrack_robot.yaml",
            persist=True,
            verbose=False,
        )

        return results