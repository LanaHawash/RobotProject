from ultralytics import YOLO

from robot_project.config import (
    YOLO_CONFIDENCE_THRESHOLD,
    YOLO_MODEL_PATH,
)


class ObjectDetector:

    def __init__(self):
        if not YOLO_MODEL_PATH.exists():
            raise FileNotFoundError(
                f"YOLO model was not found: {YOLO_MODEL_PATH}"
            )

        print(f"Loading YOLO model: {YOLO_MODEL_PATH}")

        self.model = YOLO(str(YOLO_MODEL_PATH))

    def detect(self, frame):
        results = self.model.predict(
            source=frame,
            conf=YOLO_CONFIDENCE_THRESHOLD,
            verbose=False,
        )

        return results