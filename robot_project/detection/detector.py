import numpy as np

from robot_project.detection.yolo import ObjectDetector


class Detector:

    def __init__(self):
        self.detector = ObjectDetector()

    @staticmethod
    def get_median_depth(depth_frame, center_x, center_y, radius=5):
        """
        Return the median valid depth around an object's center.

        radius=5 creates an 11 x 11 sampling region.
        Zero depth values are ignored.
        """

        if depth_frame is None:
            return None

        height, width = depth_frame.shape[:2]

        x1 = max(0, center_x - radius)
        x2 = min(width, center_x + radius + 1)

        y1 = max(0, center_y - radius)
        y2 = min(height, center_y + radius + 1)

        depth_region = depth_frame[y1:y2, x1:x2]

        valid_depth = depth_region[depth_region > 0]

        if valid_depth.size == 0:
            return None

        return int(np.median(valid_depth))

    def detect(self, frame, depth_frame=None):

        results = self.detector.detect(frame)

        detections = []

        if not results:
            return detections, frame.copy()

        result = results[0]

        for box in result.boxes:

            x1, y1, x2, y2 = box.xyxy[0].tolist()

            class_id = int(box.cls[0])
            confidence = float(box.conf[0])

            label = result.names[class_id]

            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            if depth_frame is not None:
                height, width = depth_frame.shape[:2]

                center_x = max(0, min(center_x, width - 1))
                center_y = max(0, min(center_y, height - 1))

            distance_mm = self.get_median_depth(
                depth_frame,
                center_x,
                center_y,
            )

            detections.append({
                "class": label,
                "confidence": confidence,
                "distance_mm": distance_mm,
                "center": (center_x, center_y),
                "bounding_box": (
                    int(x1),
                    int(y1),
                    int(x2),
                    int(y2),
                ),
            })

        annotated_frame = result.plot()

        return detections, annotated_frame
