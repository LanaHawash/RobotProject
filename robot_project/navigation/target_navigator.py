import math
import threading
import time
from typing import Callable, Optional

from robot_project.hardware.arduino_controller import (
    ArduinoController,
)
from robot_project.navigation.movement_history import (
    MovementHistory,
)


class TargetNavigator:
    FRAME_CENTER_X = 320
    CENTER_TOLERANCE_PX = 70

    SMALL_TURN_DEGREES = 3
    LARGE_TURN_DEGREES = 6

    TARGET_UPDATE_DELAY_SECONDS = 0.6
    MAX_LOST_TARGET_UPDATES = 8

    # The selected target must remain centered for two consecutive
    # camera updates before ultrasonic monitoring is allowed.
    REQUIRED_CENTERED_UPDATES = 2

    # When the ultrasonic cannot yet obtain a trustworthy reading,
    # make only small camera-approved search movements.
    ULTRASONIC_SEARCH_FORWARD_MS = 100
    MAX_ULTRASONIC_SEARCH_PULSES = 4

    # Ultrasonic-controlled normal approach.
    FAR_DISTANCE_CM = 50.0
    MEDIUM_DISTANCE_CM = 30.0
    PICKUP_POSITIONING_START_CM = 20.0
    EMERGENCY_DISTANCE_CM = 6.0
    MAX_PLAUSIBLE_APPROACH_DISTANCE_CM = 100.0

    FAR_FORWARD_MS = 300
    MEDIUM_FORWARD_MS = 150
    CLOSE_FORWARD_MS = 80

    MIN_VALID_OCCUPANCY = 0.001
    MAX_VALID_OCCUPANCY = 0.80

    # Return-home dead reckoning.
    # PLACEHOLDER - measure this on your robot before relying on
    # it: drive FORWARD for a fixed duration a few times, measure
    # the distance traveled, and set this to cm-traveled / ms-run.
    CM_PER_MS = 0.05
    MAX_FORWARD_CHUNK_MS = 5000
    MIN_EXECUTABLE_TURN_DEGREES = 1.0

    def __init__(
        self,
        arduino: ArduinoController,
        get_target: Callable[
            [], Optional[dict]
        ],
    ):
        self.arduino = arduino
        self.get_target = get_target

        self.navigation_thread = None
        self.stop_event = threading.Event()

        self.history = MovementHistory()

        self.status = "IDLE"
        self.last_action = None
        self.error = None

        self.pickup_distance_cm = None
        self.pickup_pulses = []

        self.centered_updates = 0
        self.ultrasonic_search_pulses = 0
        self.last_ultrasonic_distance_cm = None

        self.last_pose = None

    def start(self) -> None:
        if self.is_running():
            return

        self.stop_event.clear()
        self.history.clear()

        self.error = None
        self.status = "STARTING"
        self.last_action = None

        self.pickup_distance_cm = None
        self.pickup_pulses = []

        self.centered_updates = 0
        self.ultrasonic_search_pulses = 0
        self.last_ultrasonic_distance_cm = None

        self.last_pose = None

        self.navigation_thread = threading.Thread(
            target=self._navigation_loop,
            daemon=True,
        )
        self.navigation_thread.start()

    def stop(self) -> None:
        self.stop_event.set()

        try:
            self.arduino.stop()
        except Exception as error:
            print(
                f"Navigation stop warning: {error}"
            )

        self.status = "STOPPED"
        self.last_action = "STOP"

    def is_running(self) -> bool:
        return (
            self.navigation_thread is not None
            and self.navigation_thread.is_alive()
        )

    def is_returning(self) -> bool:
        """
        True while the robot is anywhere in the post-pickup
        sequence (grabbing, driving home, or turning to face the
        bins). Used to stop a new navigation run from starting on
        top of an in-progress return.
        """
        return self.status in {
            "GRABBING_OBJECT",
            "RETURNING_HOME",
            "HOME_REACHED",
            "FACING_BINS",
        }

    def movement_history(self) -> list[dict]:
        return self.history.snapshot()

    def simplified_history(self) -> list[dict]:
        return self.history.simplified()

    def return_route(self) -> list[dict]:
        return self.history.inverse_route()

    def _navigation_loop(self) -> None:
        print("Target navigation started.")

        self.status = "SEARCHING"
        lost_target_updates = 0
        robot_already_stopped = False

        try:
            while not self.stop_event.is_set():
                target = self.get_target()

                if target is None:
                    if not robot_already_stopped:
                        self.arduino.stop()
                        robot_already_stopped = True

                    self.centered_updates = 0
                    lost_target_updates += 1

                    self.status = "TARGET_NOT_AVAILABLE"
                    self.last_action = "WAITING_FOR_TARGET"

                    if (
                        lost_target_updates
                        >= self.MAX_LOST_TARGET_UPDATES
                    ):
                        self.status = "TARGET_LOST"
                        break

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue

                lost_target_updates = 0
                robot_already_stopped = False

                parsed_target = self._parse_target(target)

                if parsed_target is None:
                    self.arduino.stop()
                    robot_already_stopped = True
                    self.centered_updates = 0

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue

                center_x, box_occupancy = parsed_target

                horizontal_error = (
                    center_x - self.FRAME_CENTER_X
                )

                target_is_centered = (
                    abs(horizontal_error)
                    <= self.CENTER_TOLERANCE_PX
                )

                if not target_is_centered:
                    self.centered_updates = 0
                    self.ultrasonic_search_pulses = 0
                    self.last_ultrasonic_distance_cm = None

                    self._align_target(horizontal_error)

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue

                # Ultrasonic remains inactive until the same selected
                # target is centered for consecutive camera updates.
                self.centered_updates += 1

                if (
                    self.centered_updates
                    < self.REQUIRED_CENTERED_UPDATES
                ):
                    self.arduino.stop()
                    robot_already_stopped = True

                    self.status = "CONFIRMING_ALIGNMENT"
                    self.last_action = (
                        "CENTERED_CONFIRMATION "
                        f"{self.centered_updates}/"
                        f"{self.REQUIRED_CENTERED_UPDATES}"
                    )

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue

                # The robot is stopped before every ultrasonic read.
                self.arduino.stop()
                robot_already_stopped = True

                self.status = "ULTRASONIC_MONITORING"
                self.last_action = "READ_DISTANCE"

                distance_cm = (
                    self.arduino.read_distance_cm()
                )
                self.last_ultrasonic_distance_cm = (
                    distance_cm
                )

                if self._distance_is_unreliable(
                    distance_cm
                ):
                    if (
                        self.ultrasonic_search_pulses
                        >=
                        self.MAX_ULTRASONIC_SEARCH_PULSES
                    ):
                        self.status = (
                            "ULTRASONIC_NOT_ACQUIRED"
                        )
                        self.last_action = (
                            "STOP_ULTRASONIC_UNRELIABLE"
                        )
                        break

                    self.ultrasonic_search_pulses += 1

                    self.status = (
                        "CAUTIOUS_ULTRASONIC_SEARCH"
                    )
                    self.last_action = (
                        "FORWARD_SEARCH "
                        f"{self.ULTRASONIC_SEARCH_FORWARD_MS} "
                        f"ATTEMPT="
                        f"{self.ultrasonic_search_pulses}"
                    )

                    actual_duration = (
                        self.arduino.forward(
                            self.ULTRASONIC_SEARCH_FORWARD_MS
                        )
                    )

                    self.history.record_linear(
                        command="FORWARD",
                        duration_ms=actual_duration,
                        source="ULTRASONIC_SEARCH",
                    )

                    robot_already_stopped = False

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue

                self.ultrasonic_search_pulses = 0

                if (
                    distance_cm
                    <= self.EMERGENCY_DISTANCE_CM
                ):
                    self.arduino.stop()
                    robot_already_stopped = True

                    raise RuntimeError(
                        "ERROR,OBJECT_TOO_CLOSE,"
                        f"DISTANCE_CM={distance_cm:.1f}"
                    )

                if (
                    distance_cm
                    <=
                    self.PICKUP_POSITIONING_START_CM
                ):
                    self.status = (
                        "POSITIONING_FOR_PICKUP"
                    )
                    self.last_action = (
                        "POSITION_FOR_PICKUP "
                        f"DISTANCE_CM={distance_cm:.1f}"
                    )

                    print(
                        "Centered target is "
                        f"{distance_cm:.1f} cm away. "
                        "Starting fine pickup positioning."
                    )

                    result = (
                        self.arduino
                        .position_for_pickup()
                    )

                    self._complete_pickup_positioning(
                        result
                    )

                    self._grab_object()
                    self._return_to_start()
                    self._face_bins()

                    break

                forward_duration = (
                    self._approach_duration(distance_cm)
                )

                self.status = (
                    "ULTRASONIC_APPROACHING"
                )
                self.last_action = (
                    f"FORWARD {forward_duration} "
                    f"DISTANCE_CM={distance_cm:.1f}"
                )

                actual_duration = (
                    self.arduino.forward(
                        forward_duration
                    )
                )

                self.history.record_linear(
                    command="FORWARD",
                    duration_ms=actual_duration,
                    source="ULTRASONIC_APPROACH",
                )

                robot_already_stopped = False

                time.sleep(
                    self.TARGET_UPDATE_DELAY_SECONDS
                )

        except Exception as error:
            if self.stop_event.is_set():
                self.status = "STOPPED"
                self.last_action = "STOP"
            else:
                self.error = str(error)
                self.status = "ERROR"
                print(f"Navigation error: {error}")

        finally:
            try:
                self.arduino.stop()
            except Exception as error:
                print(
                    "Final navigation STOP warning: "
                    f"{error}"
                )

            print(
                "Target navigation stopped. "
                f"Status: {self.status}"
            )

    def _parse_target(
        self,
        target: dict,
    ) -> Optional[tuple[int, float]]:
        center_x = target.get("center_x")
        box_occupancy = target.get("box_occupancy")

        if (
            center_x is None
            or box_occupancy is None
        ):
            self.status = "INVALID_TARGET_DATA"
            self.last_action = "INVALID_TARGET_DATA"
            return None

        try:
            center_x = int(center_x)
            box_occupancy = float(box_occupancy)
        except (TypeError, ValueError):
            self.status = "INVALID_TARGET_DATA"
            self.last_action = "INVALID_TARGET_TYPES"
            return None

        if not (
            self.MIN_VALID_OCCUPANCY
            <= box_occupancy
            <= self.MAX_VALID_OCCUPANCY
        ):
            self.status = "INVALID_BOX_OCCUPANCY"
            self.last_action = (
                "IGNORE_OCCUPANCY "
                f"{box_occupancy:.4f}"
            )
            return None

        return center_x, box_occupancy

    def _align_target(
        self,
        horizontal_error: int,
    ) -> None:
        self.status = "ALIGNING"

        if abs(horizontal_error) > 220:
            turn_angle = self.LARGE_TURN_DEGREES
        else:
            turn_angle = self.SMALL_TURN_DEGREES

        # Current camera orientation:
        # image left -> robot turns right
        # image right -> robot turns left
        if horizontal_error < 0:
            self.last_action = (
                f"TURN_RIGHT {turn_angle} "
                f"ERROR_PX={horizontal_error}"
            )

            actual_angle = (
                self.arduino.turn_right(turn_angle)
            )

            self.history.record_turn(
                command="TURN_RIGHT",
                angle=actual_angle,
            )
        else:
            self.last_action = (
                f"TURN_LEFT {turn_angle} "
                f"ERROR_PX={horizontal_error}"
            )

            actual_angle = (
                self.arduino.turn_left(turn_angle)
            )

            self.history.record_turn(
                command="TURN_LEFT",
                angle=actual_angle,
            )

    def _distance_is_unreliable(
        self,
        distance_cm: Optional[float],
    ) -> bool:
        return (
            distance_cm is None
            or distance_cm <= 0.0
            or distance_cm >
            self.MAX_PLAUSIBLE_APPROACH_DISTANCE_CM
        )

    def _approach_duration(
        self,
        distance_cm: float,
    ) -> int:
        if distance_cm > self.FAR_DISTANCE_CM:
            return self.FAR_FORWARD_MS

        if distance_cm > self.MEDIUM_DISTANCE_CM:
            return self.MEDIUM_FORWARD_MS

        return self.CLOSE_FORWARD_MS

    def _complete_pickup_positioning(
        self,
        result: dict,
    ) -> None:
        self.pickup_pulses = [
            pulse.copy()
            for pulse in result["pulses"]
        ]

        self.history.extend_pulses(
            self.pickup_pulses
        )

        self.pickup_distance_cm = (
            result["distance_cm"]
        )

        self.status = "TARGET_REACHED"
        self.last_action = (
            "PICKUP_POSITION_REACHED "
            f"DISTANCE_CM="
            f"{self.pickup_distance_cm:.1f}"
        )

        print(
            "Pickup position reached at "
            f"{self.pickup_distance_cm:.1f} cm."
        )

        print(
            "Recorded movement history: "
            f"{self.history.snapshot()}"
        )
        print(
            "Simplified movement history: "
            f"{self.history.simplified()}"
        )
        print(
            "Generated return route: "
            f"{self.history.inverse_route()}"
        )

    def _grab_object(self) -> None:
        """
        Placeholder for the robotic arm pickup. This is the exact
        point where the arm call will go once it exists - for now
        it just marks status and assumes the object is held.
        """
        self.status = "GRABBING_OBJECT"
        self.last_action = "GRAB_OBJECT_PLACEHOLDER"

        print(
            "Arm grab not implemented yet. "
            "Assuming the object is now held."
        )

    def _return_to_start(self) -> None:
        """
        Turn once toward the estimated starting point and drive
        straight there, using the pose estimated from the outbound
        history. This does not replay any recorded commands.
        """
        self.status = "RETURNING_HOME"

        pose = self.history.estimate_pose(
            self.CM_PER_MS
        )
        self.last_pose = pose

        distance_home_cm = math.hypot(
            pose["x"], pose["y"]
        )
        bearing_home = math.degrees(
            math.atan2(-pose["x"], -pose["y"])
        )
        turn_needed = self._normalize_angle(
            bearing_home - pose["heading_degrees"]
        )

        self.last_action = (
            "RETURN_HOME_PLAN "
            f"DISTANCE_CM={distance_home_cm:.1f} "
            f"TURN_DEGREES={turn_needed:.1f}"
        )

        print(
            "Return-home plan: "
            f"distance={distance_home_cm:.1f} cm, "
            f"turn={turn_needed:.1f} degrees "
            f"(estimated pose: {pose})"
        )

        self._execute_turn(turn_needed)
        self._drive_distance_cm(distance_home_cm)

        self.status = "HOME_REACHED"
        self.last_action = "HOME_REACHED"

    def _face_bins(self) -> None:
        """
        Turn to face the bins behind the starting point, using the
        heading estimated after the return drive - not a fixed
        180-degree turn.
        """
        self.status = "FACING_BINS"

        pose = self.history.estimate_pose(
            self.CM_PER_MS
        )
        self.last_pose = pose

        turn_to_bins = self._normalize_angle(
            180 - pose["heading_degrees"]
        )

        self.last_action = (
            f"FACE_BINS TURN_DEGREES={turn_to_bins:.1f}"
        )

        self._execute_turn(turn_to_bins)

        self.status = "READY_TO_SORT"
        self.last_action = "READY_TO_SORT"

        print(
            f"Facing bins after turning "
            f"{turn_to_bins:.1f} degrees."
        )

    def _execute_turn(
        self,
        angle_degrees: float,
    ) -> None:
        if (
            abs(angle_degrees)
            < self.MIN_EXECUTABLE_TURN_DEGREES
        ):
            return

        angle = min(abs(angle_degrees), 180.0)

        if angle_degrees > 0:
            actual_angle = (
                self.arduino.turn_right(angle)
            )
            self.history.record_turn(
                command="TURN_RIGHT",
                angle=actual_angle,
            )
        else:
            actual_angle = (
                self.arduino.turn_left(angle)
            )
            self.history.record_turn(
                command="TURN_LEFT",
                angle=actual_angle,
            )

    def _drive_distance_cm(
        self,
        distance_cm: float,
    ) -> None:
        if distance_cm <= 0:
            return

        remaining_ms = int(
            distance_cm / self.CM_PER_MS
        )

        while remaining_ms > 0:
            chunk_ms = min(
                remaining_ms,
                self.MAX_FORWARD_CHUNK_MS,
            )

            actual_duration = self.arduino.forward(
                chunk_ms
            )

            self.history.record_linear(
                command="FORWARD",
                duration_ms=actual_duration,
                source="RETURN_HOME",
            )

            remaining_ms -= chunk_ms

    @staticmethod
    def _normalize_angle(
        angle_degrees: float,
    ) -> float:
        angle = angle_degrees % 360.0

        if angle > 180.0:
            angle -= 360.0

        return angle