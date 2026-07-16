from dataclasses import dataclass
from typing import Optional


@dataclass
class SelectedTarget:
    label: str
    average_confidence: float
    average_distance_mm: int
    center_x: int
    center_y: int
    destination: str
    confirmation_frames: int


class ObjectSelector:
    """
    Selects a stable target across several consecutive frames.

    The selector does not immediately approve the closest detection.
    It first checks that approximately the same object remains visible
    for several frames.
    """

    def __init__(
        self,
        normal_confidence_threshold=0.40,
        minimum_detection_confidence=0.20,
        normal_confirmation_frames=5,
        uncertain_confirmation_frames=8,
        maximum_center_shift=80,
    ):
        self.normal_confidence_threshold = normal_confidence_threshold
        self.minimum_detection_confidence = minimum_detection_confidence

        self.normal_confirmation_frames = normal_confirmation_frames
        self.uncertain_confirmation_frames = uncertain_confirmation_frames

        self.maximum_center_shift = maximum_center_shift

        self.candidate_label = None
        self.candidate_center_x = None
        self.candidate_center_y = None

        self.confidence_history = []
        self.distance_history = []

        self.confirmed_target: Optional[SelectedTarget] = None

    def reset_candidate(self):
        self.candidate_label = None
        self.candidate_center_x = None
        self.candidate_center_y = None

        self.confidence_history.clear()
        self.distance_history.clear()

    def clear_target(self):
        self.confirmed_target = None
        self.reset_candidate()

    def _is_same_candidate(self, detection):
        if self.candidate_label is None:
            return False

        if detection["class"] != self.candidate_label:
            return False

        center_x, center_y = detection["center"]

        horizontal_shift = abs(center_x - self.candidate_center_x)
        vertical_shift = abs(center_y - self.candidate_center_y)

        return (
            horizontal_shift <= self.maximum_center_shift
            and vertical_shift <= self.maximum_center_shift
        )

    def _choose_closest_valid_detection(self, detections):
        valid_detections = [
            detection
            for detection in detections
            if (
                detection["distance_mm"] is not None
                and detection["distance_mm"] > 0
                and detection["confidence"]
                >= self.minimum_detection_confidence
            )
        ]

        if not valid_detections:
            return None

        return min(
            valid_detections,
            key=lambda detection: detection["distance_mm"],
        )

    def _start_new_candidate(self, detection):
        center_x, center_y = detection["center"]

        self.candidate_label = detection["class"]
        self.candidate_center_x = center_x
        self.candidate_center_y = center_y

        self.confidence_history = [detection["confidence"]]
        self.distance_history = [detection["distance_mm"]]

    def _update_candidate(self, detection):
        center_x, center_y = detection["center"]

        self.candidate_center_x = center_x
        self.candidate_center_y = center_y

        self.confidence_history.append(detection["confidence"])
        self.distance_history.append(detection["distance_mm"])

    def update(self, detections):
        """
        Process detections from one frame.

        Returns:
            SelectedTarget when a target is confirmed.
            None while the candidate is still being observed.
        """

        closest = self._choose_closest_valid_detection(detections)

        if closest is None:
            self.reset_candidate()
            self.confirmed_target = None
            return None

        if self._is_same_candidate(closest):
            self._update_candidate(closest)
        else:
            self._start_new_candidate(closest)
            self.confirmed_target = None

        average_confidence = (
            sum(self.confidence_history)
            / len(self.confidence_history)
        )

        average_distance = int(
            sum(self.distance_history)
            / len(self.distance_history)
        )

        if average_confidence >= self.normal_confidence_threshold:
            required_frames = self.normal_confirmation_frames
            destination = self.candidate_label
        else:
            required_frames = self.uncertain_confirmation_frames
            destination = "discharge"

        if len(self.confidence_history) < required_frames:
            return None

        self.confirmed_target = SelectedTarget(
            label=self.candidate_label,
            average_confidence=average_confidence,
            average_distance_mm=average_distance,
            center_x=self.candidate_center_x,
            center_y=self.candidate_center_y,
            destination=destination,
            confirmation_frames=len(self.confidence_history),
        )

        return self.confirmed_target

    def get_candidate_status(self):
        if self.candidate_label is None:
            return {
                "label": None,
                "frames_seen": 0,
                "average_confidence": None,
                "average_distance_mm": None,
            }

        average_confidence = (
            sum(self.confidence_history)
            / len(self.confidence_history)
        )

        average_distance = int(
            sum(self.distance_history)
            / len(self.distance_history)
        )

        return {
            "label": self.candidate_label,
            "frames_seen": len(self.confidence_history),
            "average_confidence": round(average_confidence, 3),
            "average_distance_mm": average_distance,
        }

    def get_confirmed_target(self):
        return self.confirmed_target
