from ultralytics import YOLO


class ObjectDetector:

    def __init__(self):

        print("Loading YOLO model...")

        self.model = YOLO("runs/detect/models/toy_detector-2/weights/best.pt")

        print("YOLO loaded successfully!")

    def detect(self, frame):

        results = self.model(frame, verbose=False)

        return results