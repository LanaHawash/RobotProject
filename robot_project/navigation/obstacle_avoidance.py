import time

from robot_project.hardware.arduino_controller import (
    ArduinoController,
)
from robot_project.navigation.movement_history import (
    MovementHistory,
)


class ObstacleAvoidance:
    """
    Isolated obstacle-avoidance helper.

    This class does NOT:
    - select targets
    - clear targets
    - control pickup
    - control bins
    - change navigation state permanently

    It only:
    - decides whether a front ultrasonic reading looks like
      an obstacle during the FAR target approach
    - performs a temporary detour
    - records that detour in MovementHistory
    """

    # --------------------------------------------------
    # DEMO SETTINGS
    # --------------------------------------------------

    # If the ultrasonic sees something this close while
    # the visual target still looks far away, treat the
    # closer object as an obstacle.
    OBSTACLE_DISTANCE_CM = 25.0

    # The target bounding box must still occupy no more
    # than 5% of the camera image.
    #
    # This is what prevents the selected target itself
    # from normally being treated as an obstacle when
    # the robot gets close to it.
    MAX_TARGET_OCCUPANCY_FOR_OBSTACLE = 0.05

    # Require consecutive detections to reject one bad
    # ultrasonic measurement.
    REQUIRED_CONFIRMATIONS = 2

    # --------------------------------------------------
    # DETOUR SETTINGS
    # --------------------------------------------------

    DETOUR_TURN_DEGREES = 30.0

    # First diagonal movement away from original path.
    DETOUR_OUTWARD_MS = 450

    # Forward movement beside the obstacle.
    DETOUR_PASS_MS = 700

    # Diagonal movement back toward original path.
    DETOUR_RETURN_MS = 450

    SETTLE_SECONDS = 0.20

    # Prevent immediate re-triggering after completing
    # the maneuver.
    RETRIGGER_DELAY_SECONDS = 1.5

    def __init__(
        self,
        arduino: ArduinoController,
        history: MovementHistory,
    ):
        self.arduino = arduino
        self.history = history

        self.confirmation_count = 0
        self.ignore_until = 0.0

    def reset(self) -> None:
        self.confirmation_count = 0
        self.ignore_until = 0.0

    def should_avoid(
        self,
        distance_cm,
        target_occupancy: float,
    ) -> bool:
        """
        Decide whether the current front ultrasonic object
        should be treated as a demo obstacle.

        The target must still look visually far away.
        """

        if time.monotonic() < self.ignore_until:
            self.confirmation_count = 0
            return False

        if distance_cm is None:
            self.confirmation_count = 0
            return False

        # ------------------------------------------------
        # IMPORTANT:
        # If the selected target already looks large in
        # the camera, allow normal target approach.
        #
        # This protects the existing pickup behavior.
        # ------------------------------------------------
        if (
            target_occupancy
            > self.MAX_TARGET_OCCUPANCY_FOR_OBSTACLE
        ):
            self.confirmation_count = 0
            return False

        # Target still looks far away, but ultrasonic
        # sees something nearby.
        if distance_cm <= self.OBSTACLE_DISTANCE_CM:
            self.confirmation_count += 1

            print(
                "Possible obstacle detected: "
                f"distance={distance_cm:.1f} cm, "
                f"target_occupancy="
                f"{target_occupancy:.3f}, "
                f"confirmation="
                f"{self.confirmation_count}/"
                f"{self.REQUIRED_CONFIRMATIONS}"
            )
        else:
            self.confirmation_count = 0

        return (
            self.confirmation_count
            >= self.REQUIRED_CONFIRMATIONS
        )

    def avoid_right(self) -> None:
        """
        Perform one fixed right-side bypass.

        DEMO ASSUMPTION:
        The right side of the obstacle is clear.
        """

        print()
        print("================================")
        print("OBSTACLE CONFIRMED")
        print("Starting right-side avoidance")
        print("================================")

        self.confirmation_count = 0

        # ------------------------------------------------
        # STOP FIRST
        # ------------------------------------------------

        self.arduino.stop()
        time.sleep(self.SETTLE_SECONDS)

        # ------------------------------------------------
        # 1. Turn diagonally right
        # ------------------------------------------------

        actual_angle = self.arduino.turn_right(
            self.DETOUR_TURN_DEGREES
        )

        self.history.record_turn(
            command="TURN_RIGHT",
            angle=actual_angle,
        )

        time.sleep(self.SETTLE_SECONDS)

        # ------------------------------------------------
        # 2. Move outward
        # ------------------------------------------------

        actual_duration = self.arduino.forward(
            self.DETOUR_OUTWARD_MS
        )

        self.history.record_linear(
            command="FORWARD",
            duration_ms=actual_duration,
            source="OBSTACLE_AVOIDANCE",
        )

        time.sleep(self.SETTLE_SECONDS)

        # ------------------------------------------------
        # 3. Restore original heading
        # ------------------------------------------------

        actual_angle = self.arduino.turn_left(
            self.DETOUR_TURN_DEGREES
        )

        self.history.record_turn(
            command="TURN_LEFT",
            angle=actual_angle,
        )

        time.sleep(self.SETTLE_SECONDS)

        # ------------------------------------------------
        # 4. Move beside / past obstacle
        # ------------------------------------------------

        actual_duration = self.arduino.forward(
            self.DETOUR_PASS_MS
        )

        self.history.record_linear(
            command="FORWARD",
            duration_ms=actual_duration,
            source="OBSTACLE_AVOIDANCE",
        )

        time.sleep(self.SETTLE_SECONDS)

        # ------------------------------------------------
        # 5. Turn toward original path
        # ------------------------------------------------

        actual_angle = self.arduino.turn_left(
            self.DETOUR_TURN_DEGREES
        )

        self.history.record_turn(
            command="TURN_LEFT",
            angle=actual_angle,
        )

        time.sleep(self.SETTLE_SECONDS)

        # ------------------------------------------------
        # 6. Move back toward original path
        # ------------------------------------------------

        actual_duration = self.arduino.forward(
            self.DETOUR_RETURN_MS
        )

        self.history.record_linear(
            command="FORWARD",
            duration_ms=actual_duration,
            source="OBSTACLE_AVOIDANCE",
        )

        time.sleep(self.SETTLE_SECONDS)

        # ------------------------------------------------
        # 7. Restore original heading
        # ------------------------------------------------

        actual_angle = self.arduino.turn_right(
            self.DETOUR_TURN_DEGREES
        )

        self.history.record_turn(
            command="TURN_RIGHT",
            angle=actual_angle,
        )

        self.arduino.stop()

        self.ignore_until = (
            time.monotonic()
            + self.RETRIGGER_DELAY_SECONDS
        )

        print("================================")
        print("OBSTACLE AVOIDANCE COMPLETE")
        print("Returning control to navigator")
        print("================================")
        print()