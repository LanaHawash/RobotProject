from robot_project.detection.yolo import ObjectDetector


class Detector:

    def __init__(self):
        self.detector = ObjectDetector()

    def detect(self, frame, depth_frame=None):

        results = self.detector.detect(frame)

        annotated = frame.copy()

        detections = []

        if depth_frame is None:
            return [], annotated

        for r in results[0].boxes:

            x1, y1, x2, y2 = r.xyxy[0].tolist()

            cls_id = int(r.cls[0])
            conf = float(r.conf[0])

            label = results[0].names[cls_id]

            # object center
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # safe bounds check
            h, w = depth_frame.shape
            cx = max(0, min(cx, w - 1))
            cy = max(0, min(cy, h - 1))

            distance = int(depth_frame[cy, cx])

            # store structured result
            detections.append({
                "class": label,
                "confidence": conf,
                "distance_mm": distance
            })

        return detections, results[0].plot()