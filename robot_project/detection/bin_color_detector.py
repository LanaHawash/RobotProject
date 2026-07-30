import cv2
import numpy as np

from robot_project.config import (
    BIN_COLOR_HSV_RANGES,
    MIN_BIN_COLOR_AREA,
    RED_BIN_HSV_RANGES,
)


class BinColorDetector:

    @staticmethod
    def _create_mask(hsv_frame, target_color):
        """
        Create a binary mask containing only the requested bin color.
        """

        if target_color == "red":
            masks = []

            for color_range in RED_BIN_HSV_RANGES:
                lower = np.array(
                    color_range["lower"],
                    dtype=np.uint8,
                )

                upper = np.array(
                    color_range["upper"],
                    dtype=np.uint8,
                )

                mask = cv2.inRange(
                    hsv_frame,
                    lower,
                    upper,
                )

                masks.append(mask)

            return cv2.bitwise_or(
                masks[0],
                masks[1],
            )

        color_range = BIN_COLOR_HSV_RANGES.get(
            target_color
        )

        if color_range is None:
            return None

        lower = np.array(
            color_range["lower"],
            dtype=np.uint8,
        )

        upper = np.array(
            color_range["upper"],
            dtype=np.uint8,
        )

        return cv2.inRange(
            hsv_frame,
            lower,
            upper,
        )

    def detect(self, frame, target_color):
        """
        Find the largest visible region matching target_color.

        Returns a dictionary containing the detected region information,
        or None when no valid region is found.
        """

        if frame is None:
            return None

        if not target_color:
            return None

        hsv_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV,
        )

        mask = self._create_mask(
            hsv_frame,
            target_color,
        )

        if mask is None:
            return None

        kernel = np.ones(
            (5, 5),
            dtype=np.uint8,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return None

        largest_contour = max(
            contours,
            key=cv2.contourArea,
        )

        area = cv2.contourArea(
            largest_contour
        )

        if area < MIN_BIN_COLOR_AREA:
            return None

        x, y, width, height = cv2.boundingRect(
            largest_contour
        )

        center_x = x + width // 2
        center_y = y + height // 2

        return {
            "color": target_color,
            "center_x": center_x,
            "center_y": center_y,
            "width": width,
            "height": height,
            "area": int(area),
            "bounding_box": (
                x,
                y,
                x + width,
                y + height,
            ),
        }