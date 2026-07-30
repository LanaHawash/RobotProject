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

from robot_project.config import DESTINATION_BIN_COLORS
from robot_project.detection.bin_color_detector import (
    BinColorDetector,
)


class TargetNavigator:
    FRAME_CENTER_X = 320
    # Precise initial alignment before movement.
    CENTER_TOLERANCE_PX = 70

    # Slightly wider tolerance while the robot is moving.
    DRIVING_CENTER_TOLERANCE_PX = 95

    # Stop after two consecutive genuinely misaligned updates.
    MISALIGNMENT_CONFIRMATION_UPDATES = 2

    SMALL_TURN_DEGREES = 3
    LARGE_TURN_DEGREES = 6

    TARGET_UPDATE_DELAY_SECONDS = 0.35
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
    PICKUP_POSITIONING_START_CM = 15.0
    CONTINUOUS_FORWARD_MIN_DISTANCE_CM = 40.0
    EMERGENCY_DISTANCE_CM = 3.0
    MAX_PLAUSIBLE_APPROACH_DISTANCE_CM = 100.0

    FAR_FORWARD_MS = 500
    MEDIUM_FORWARD_MS = 120
    CLOSE_FORWARD_MS = 100

    # Smooth far-distance drive. Arduino stops automatically if
    # this heartbeat is not refreshed within its safety timeout.
    CONTINUOUS_FORWARD_REFRESH_SECONDS = 0.4
    MAX_CONTINUOUS_FORWARD_SEGMENT_SECONDS = 0.65

    MIN_VALID_OCCUPANCY = 0.001
    MAX_VALID_OCCUPANCY = 0.80

    # Return-home dead reckoning.
    # PLACEHOLDER - measure this on your robot before relying on
    # it: drive FORWARD for a fixed duration a few times, measure
    # the distance traveled, and set this to cm-traveled / ms-run.
    CM_PER_MS = 0.055
    MAX_FORWARD_CHUNK_MS = 5000
    MIN_EXECUTABLE_TURN_DEGREES = 1.0
    RETURN_DURATION_SCALE = 0.4

    BIN_SEARCH_UPDATE_DELAY_SECONDS = 0.25
    MAX_BIN_SEARCH_UPDATES = 40
    REQUIRED_BIN_DETECTIONS = 3

    def __init__(
        self,
        arduino: ArduinoController,
        get_target: Callable[[], Optional[dict]],
        get_raw_frame: Callable[[], object],
    ):
        self.arduino = arduino
        self.get_target = get_target
        self.get_raw_frame = get_raw_frame
        self.bin_color_detector = BinColorDetector()

        self.navigation_thread = None
        self.stop_event = threading.Event()

        self.history = MovementHistory()

        self.status = "IDLE"
        self.last_action = None
        self.error = None
        self.locked_object_class = None
        self.locked_destination = None
        self.locked_bin_color = None
        self.current_bin_target = None

        self.pickup_distance_cm = None
        self.pickup_pulses = []

        self.centered_updates = 0
        self.ultrasonic_search_pulses = 0
        self.last_ultrasonic_distance_cm = None

        self.last_pose = None

        self.continuous_forward_active = False
        self.last_forward_refresh_time = 0.0
        self.continuous_forward_start_time = None
        self.misaligned_updates = 0
        
        
    def start(self, selected_target: dict) -> None:
        if self.is_running():
            return

        object_class = selected_target.get("label")
        destination = selected_target.get("destination")

        if not object_class or not destination:
            raise RuntimeError(
                "The confirmed target has no class or destination."
            )
        self.locked_object_class = object_class
        self.locked_destination = destination

        self.locked_bin_color = DESTINATION_BIN_COLORS.get(
            self.locked_destination
        )

        if self.locked_bin_color is None:
            raise RuntimeError(
                "No bin color is configured for destination "
                f"'{self.locked_destination}'."
            )

        print(
            "Navigation target locked: "
            f"object={self.locked_object_class}, "
            f"destination={self.locked_destination}, "
            f"bin_color={self.locked_bin_color}"
        )
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

        self.continuous_forward_active = False
        self.last_forward_refresh_time = 0.0
        self.continuous_forward_start_time = None
        self.misaligned_updates = 0
        self.current_bin_target = None

        self.navigation_thread = threading.Thread(
            target=self._navigation_loop,
            daemon=True,
        )
        self.navigation_thread.start()

    def start_return(self) -> None:
        """
        Start the existing return-home calculation manually.

        This reuses _return_to_start() and _face_bins() without
        changing their calculations.
        """
        if self.is_running():
            raise RuntimeError(
                "Navigation is already running."
            )

        if not self.history.snapshot():
            raise RuntimeError(
                "No movement history is available."
            )

        self.stop_event.clear()
        self.error = None
        self.status = "RETURNING_HOME"

        self.navigation_thread = threading.Thread(
            target=self._manual_return_loop,
            daemon=True,
        )
        self.navigation_thread.start()

    def _manual_return_loop(self) -> None:
        try:
            self._return_to_start()
            self._face_bins()
        except Exception as error:
            if self.stop_event.is_set():
                self.status = "STOPPED"
                self.last_action = "STOP"
            else:
                self.error = str(error)
                self.status = "ERROR"
                print(f"Manual return error: {error}")
        finally:
            try:
                self.arduino.stop()
            except Exception as error:
                print(
                    "Final manual-return STOP warning: "
                    f"{error}"
                )

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
            "OBJECT_HELD",
            "RETURNING_HOME",
            "HOME_REACHED",
            "FACING_BINS",
            "READY_TO_SORT",
            "RELEASING_OBJECT",
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

        try:
            while not self.stop_event.is_set():
                target = self.get_target()

                if target is None:
                    self._stop_continuous_forward()
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
                parsed_target = self._parse_target(target)

                if parsed_target is None:
                    self._stop_continuous_forward()
                    self.centered_updates = 0
                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue

                center_x, box_occupancy = parsed_target
                horizontal_error = (
                    center_x - self.FRAME_CENTER_X
                )
                # Initial alignment is strict. Once the robot is already
                # moving forward, allow a wider center area and require
                # several consecutive misaligned updates before stopping.
                if self.continuous_forward_active:
                    allowed_offset = (
                        self.DRIVING_CENTER_TOLERANCE_PX
                    )
                else:
                    allowed_offset = self.CENTER_TOLERANCE_PX

                target_is_centered = (
                    abs(horizontal_error) <= allowed_offset
                )

                if target_is_centered:
                    self.misaligned_updates = 0
                    self.centered_updates += 1

                else:
                    self.centered_updates = 0

                    if self.continuous_forward_active:
                        self.misaligned_updates += 1

                        self.status = "DRIVING_WITH_MINOR_DRIFT"
                        self.last_action = (
                            "MONITORING_ALIGNMENT "
                            f"ERROR_PX={horizontal_error} "
                            f"COUNT={self.misaligned_updates}/"
                            f"{self.MISALIGNMENT_CONFIRMATION_UPDATES}"
                        )

                        if (
                            self.misaligned_updates
                            < self.MISALIGNMENT_CONFIRMATION_UPDATES
                        ):
                            self._maintain_continuous_forward(
                                self.last_ultrasonic_distance_cm
                                or 999.0
                            )
                            time.sleep(
                                self.TARGET_UPDATE_DELAY_SECONDS
                            )
                            continue

                    was_driving = self.continuous_forward_active

                    self._stop_continuous_forward()

                    self.misaligned_updates = 0
                    self.ultrasonic_search_pulses = 0
                    self.last_ultrasonic_distance_cm = None

                    self._align_target(
                        horizontal_error,
                        driving_correction=was_driving,
                    )

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue
                if (
                    not self.continuous_forward_active
                    and self.centered_updates
                    < self.REQUIRED_CENTERED_UPDATES
                ):
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
                # READ_DISTANCE does not stop the motors. During the far
                # stage the robot continues forward while this reading is
                # requested.
                self.status = "ULTRASONIC_MONITORING"
                self.last_action = "READ_DISTANCE"
                distance_cm = self.arduino.read_distance_cm()
                self.last_ultrasonic_distance_cm = distance_cm

                if self._distance_is_unreliable(distance_cm):
                    self._stop_continuous_forward()

                    if (
                        self.ultrasonic_search_pulses
                        >= self.MAX_ULTRASONIC_SEARCH_PULSES
                    ):
                        self.status = "ULTRASONIC_NOT_ACQUIRED"
                        self.last_action = (
                            "STOP_ULTRASONIC_UNRELIABLE"
                        )
                        break

                    self.ultrasonic_search_pulses += 1
                    self.status = "CAUTIOUS_ULTRASONIC_SEARCH"
                    self.last_action = (
                        "FORWARD_SEARCH "
                        f"{self.ULTRASONIC_SEARCH_FORWARD_MS} "
                        f"ATTEMPT={self.ultrasonic_search_pulses}"
                    )
                    actual_duration = self.arduino.forward(
                        self.ULTRASONIC_SEARCH_FORWARD_MS
                    )
                    self.history.record_linear(
                        command="FORWARD",
                        duration_ms=actual_duration,
                        source="ULTRASONIC_SEARCH",
                    )
                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue

                self.ultrasonic_search_pulses = 0

                if distance_cm <= self.EMERGENCY_DISTANCE_CM:
                    self._stop_continuous_forward()
                    raise RuntimeError(
                        "ERROR,OBJECT_TOO_CLOSE,"
                        f"DISTANCE_CM={distance_cm:.1f}"
                    )

                # At 15 cm the smooth far drive ends. The existing
                # Arduino fine-positioning routine then slows down and
                # calibrates the final pickup position.
                if (
                    distance_cm
                    <= self.PICKUP_POSITIONING_START_CM
                ):
                    self._stop_continuous_forward()
                    self.status = "POSITIONING_FOR_PICKUP"
                    self.last_action = (
                        "POSITION_FOR_PICKUP "
                        f"DISTANCE_CM={distance_cm:.1f}"
                    )

                    print(
                        "Centered target is "
                        f"{distance_cm:.1f} cm away. "
                        "Starting fine pickup positioning."
                    )

                    result = self.arduino.position_for_pickup()

                    if not result["success"]:
                        if self.stop_event.is_set():
                            self.status = "STOPPED"
                            self.last_action = "STOP"
                            break

                        self.history.extend_pulses(
                            result["pulses"]
                        )
                        reason = result.get("reason", "UNKNOWN")
                        self.status = "PICKUP_POSITIONING_RETRY"
                        self.last_action = (
                            "RETURN_TO_CAMERA_CONTROL "
                            f"REASON={reason}"
                        )
                        print(
                            "Pickup positioning was not completed. "
                            "Returning control to the camera: "
                            f"{reason}"
                        )
                        self.centered_updates = 0
                        time.sleep(
                            self.TARGET_UPDATE_DELAY_SECONDS
                        )
                        continue

                    self._complete_pickup_positioning(result)
                    self._grab_object()
                    self._return_to_start()
                    self._face_bins()
                    self._find_locked_bin()
                    break

                # Far away: use smooth continuous forward movement.
                if (
                    distance_cm
                    > self.CONTINUOUS_FORWARD_MIN_DISTANCE_CM
                ):
                    # Start or refresh smooth forward movement.
                    self._maintain_continuous_forward(
                        distance_cm
                    )

                    # Do not allow one uninterrupted forward section
                    # to become too long.
                    if (
                        self.continuous_forward_start_time is not None
                        and time.monotonic()
                        - self.continuous_forward_start_time
                        >= self.MAX_CONTINUOUS_FORWARD_SEGMENT_SECONDS
                    ):
                        self._stop_continuous_forward()

                        # Require camera confirmation before the next burst.
                        self.centered_updates = 0

                        self.status = "FORWARD_SEGMENT_COMPLETE"
                        self.last_action = (
                            "STOP_AFTER_SMOOTH_SEGMENT "
                            f"DISTANCE_CM={distance_cm:.1f}"
                        )

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue


                # Between 15 cm and 30 cm, do not restart a long
                # continuous movement. Use a short controlled movement,
                # then check camera alignment and distance again.
                self._stop_continuous_forward()

                self.status = "CONTROLLED_MEDIUM_APPROACH"
                self.last_action = (
                    "FORWARD_MEDIUM "
                    f"{self.MEDIUM_FORWARD_MS} "
                    f"DISTANCE_CM={distance_cm:.1f}"
                )

                actual_duration = self.arduino.forward(
                    self.MEDIUM_FORWARD_MS
                )

                self.history.record_linear(
                    command="FORWARD",
                    duration_ms=actual_duration,
                    source="MEDIUM_APPROACH",
                )

                # Require camera alignment confirmation again before
                # allowing the next forward movement.
                self.centered_updates = 0

                time.sleep(
                    self.TARGET_UPDATE_DELAY_SECONDS
                )
                continue

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
                self._stop_continuous_forward()
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

    def _maintain_continuous_forward(
        self,
        distance_cm: float,
    ) -> None:
        now = time.monotonic()

        if not self.continuous_forward_active:
            self.arduino.start_continuous_forward()
            self.continuous_forward_active = True
            self.last_forward_refresh_time = now
            self.continuous_forward_start_time = now
            self.status = "SMOOTH_FORWARD"
            self.last_action = (
                "START_CONTINUOUS_FORWARD "
                f"DISTANCE_CM={distance_cm:.1f}"
            )
            return

        if (
            now - self.last_forward_refresh_time
            >= self.CONTINUOUS_FORWARD_REFRESH_SECONDS
        ):
            self.arduino.refresh_continuous_forward()
            self.last_forward_refresh_time = now

        self.status = "SMOOTH_FORWARD"
        self.last_action = (
            "CONTINUOUS_FORWARD "
            f"DISTANCE_CM={distance_cm:.1f}"
        )

    def _stop_continuous_forward(self) -> None:
        if not self.continuous_forward_active:
            return

        stop_time = time.monotonic()
        self.arduino.stop()

        if self.continuous_forward_start_time is not None:
            duration_ms = max(
                1,
                int(
                    (stop_time - self.continuous_forward_start_time)
                    * 1000
                ),
            )
            self.history.record_linear(
                command="FORWARD",
                duration_ms=duration_ms,
                source="CONTINUOUS_FORWARD",
            )

        self.continuous_forward_active = False
        self.last_forward_refresh_time = 0.0
        self.continuous_forward_start_time = None

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
        driving_correction: bool = False,
    ) -> None:
        self.status = "ALIGNING"

        if driving_correction:
            turn_angle = self.SMALL_TURN_DEGREES
        elif abs(horizontal_error) > 220:
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
        Ask the Arduino to perform the calibrated pickup sequence.

        The robot has already completed ultrasonic positioning before
        this method is called.
        """
        self.status = "GRABBING_OBJECT"
        self.last_action = "GRAB_OBJECT"

        print("Starting robotic arm pickup.")

        self.arduino.grab_object()

        self.status = "OBJECT_HELD"
        self.last_action = "OBJECT_HELD"

        print("Robotic arm pickup completed.")

    def _release_object(self) -> None:
        """
        Ask the Arduino to release the held object after the robot
        has returned home and completed the existing bin-facing turn.
        """
        self.status = "RELEASING_OBJECT"
        self.last_action = "RELEASE_OBJECT"

        print("Releasing object into the bin.")

        self.arduino.release_object()

        self.status = "OBJECT_RELEASED"
        self.last_action = "OBJECT_RELEASED"

        print("Object release completed.")    

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

    def _find_locked_bin(self) -> dict:
        """
        Search fresh camera frames for the bin whose color was locked
        when navigation started.

        This method only detects and confirms the bin. It does not turn
        or drive the robot.
        """
        if not self.locked_bin_color:
            raise RuntimeError(
                "Cannot search for a bin because no bin color is locked."
            )

        self.status = "SEARCHING_FOR_BIN"
        self.last_action = (
            f"SEARCH_BIN COLOR={self.locked_bin_color}"
        )
        self.current_bin_target = None

        consecutive_detections = 0

        for _ in range(self.MAX_BIN_SEARCH_UPDATES):
            if self.stop_event.is_set():
                raise RuntimeError(
                    "Bin search was stopped."
                )

            frame = self.get_raw_frame()

            detection = self.bin_color_detector.detect(
                frame,
                self.locked_bin_color,
            )

            if detection is None:
                consecutive_detections = 0
                self.current_bin_target = None
                self.last_action = (
                    f"SEARCH_BIN COLOR={self.locked_bin_color} "
                    "NOT_VISIBLE"
                )
            else:
                consecutive_detections += 1
                self.current_bin_target = detection

                self.last_action = (
                    f"BIN_VISIBLE COLOR={self.locked_bin_color} "
                    f"CONFIRMATION={consecutive_detections}/"
                    f"{self.REQUIRED_BIN_DETECTIONS}"
                )

                if (
                    consecutive_detections
                    >= self.REQUIRED_BIN_DETECTIONS
                ):
                    self.status = "BIN_FOUND"
                    self.last_action = (
                        f"BIN_FOUND COLOR={self.locked_bin_color} "
                        f"CENTER_X={detection['center_x']} "
                        f"AREA={detection['area']}"
                    )

                    print(
                        "Locked bin found: "
                        f"color={self.locked_bin_color}, "
                        f"center_x={detection['center_x']}, "
                        f"area={detection['area']}"
                    )

                    return detection

            time.sleep(
                self.BIN_SEARCH_UPDATE_DELAY_SECONDS
            )

        self.current_bin_target = None

        raise RuntimeError(
            "The locked bin was not found within the search period: "
            f"{self.locked_bin_color}"
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

        estimated_ms = (
            distance_cm / self.CM_PER_MS
        )

        remaining_ms = int(
            estimated_ms
            * self.RETURN_DURATION_SCALE
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