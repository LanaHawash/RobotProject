from ultralytics import YOLO


class ObjectDetector:

    def __init__(self):

        print("Loading YOLO model...")

        self.model = YOLO("yolov8n.pt")

        print("YOLO loaded successfully!")

    def detect(self, frame):

        results = self.model(frame, verbose=False)

        return results