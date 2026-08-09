import threading
import time

from robot_project.hardware.arduino_controller import (
    ArduinoController,
)


class DeepCleaningNavigator:
    """
    Controls the room-coverage cleaning pattern separately from
    target-search navigation.

    Stage 2:
    Drive through lane 1 and stop when the ultrasonic sensor
    detects the wall.
    """

    WALL_STOP_DISTANCE_CM = 25.0

    # Temporary safety limit for the first movement test.
    MAX_LANE_DRIVE_SECONDS = 8.0

    DISTANCE_CHECK_DELAY_SECONDS = 0.10

    FIRST_TURN_ANGLE_DEGREES = 86.0
    SECOND_TURN_ANGLE_DEGREES = 80.0
    TURN_SETTLE_SECONDS = 1.0

    # Temporary lane-shift duration for calibration.
    # Increase or decrease this after observing the physical distance.
    LANE_SHIFT_DURATION_MS = 200

    SHIFT_SETTLE_SECONDS = 2.0
    BEFORE_SHIFT_SETTLE_SECONDS = 1.0
    AFTER_SHIFT_SETTLE_SECONDS = 2.0

    def __init__(
        self,
        arduino: ArduinoController,
    ):
        self.arduino = arduino

        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.cleaning_thread = None

        self.active = False
        self.status = "IDLE"
        self.last_action = None
        self.error = None

        self.lane_number = 0
        self.bins_direction = None

        self.last_distance_cm = None
        self.lane_drive_seconds = 0.0
        self.last_turn_angle = None
        self.last_shift_duration_ms = None
        self.second_turn_angle = None
        self.last_shift_duration_ms = None
        self.second_turn_angle = None

    def start(self) -> None:
        if self.is_running():
            raise RuntimeError(
                "Deep cleaning is already active."
            )

        self.stop_event.clear()

        with self.state_lock:
            self.active = True
            self.status = "STARTING_LANE_1"
            self.last_action = "CHECK_INITIAL_DISTANCE"
            self.error = None

            self.lane_number = 1
            self.bins_direction = "BEHIND"

            self.last_distance_cm = None
            self.lane_drive_seconds = 0.0
            self.last_turn_angle = None

        self.cleaning_thread = threading.Thread(
            target=self._run_lane_one_test,
            daemon=True,
        )

        self.cleaning_thread.start()

    def _run_lane_one_test(self) -> None:
        """
        Stage 3B test:

        Drive through lane 1, stop at the wall,
        turn left 90 degrees, shift toward lane 2,
        turn left 90 degrees, and stop.
        """

        forward_started = False
        drive_start_time = None

        print("Deep-cleaning lane 1 test started.")

        try:
            initial_distance = (
                self.arduino.read_distance_cm()
            )

            self._save_distance(initial_distance)

            if not self._distance_is_valid(
                initial_distance
            ):
                raise RuntimeError(
                    "No valid ultrasonic distance was "
                    "available before movement."
                )

            if (
                initial_distance
                <= self.WALL_STOP_DISTANCE_CM
            ):
                with self.state_lock:
                    self.status = "WALL_ALREADY_CLOSE"
                    self.last_action = (
                        "NO_MOVEMENT "
                        f"DISTANCE_CM={initial_distance:.1f}"
                    )

                print(
                    "Lane 1 did not start because the wall "
                    f"is already {initial_distance:.1f} cm away."
                )

                return

            if self.stop_event.is_set():
                return

            self.arduino.start_continuous_forward()

            forward_started = True
            drive_start_time = time.monotonic()

            with self.state_lock:
                self.status = "DRIVING_LANE_1"
                self.last_action = (
                    "START_CONTINUOUS_FORWARD "
                    f"DISTANCE_CM={initial_distance:.1f}"
                )

            while not self.stop_event.is_set():
                elapsed_seconds = (
                    time.monotonic()
                    - drive_start_time
                )

                with self.state_lock:
                    self.lane_drive_seconds = (
                        elapsed_seconds
                    )

                if (
                    elapsed_seconds
                    >= self.MAX_LANE_DRIVE_SECONDS
                ):
                    self.arduino.stop()
                    forward_started = False

                    with self.state_lock:
                        self.status = (
                            "LANE_1_SAFETY_TIMEOUT"
                        )
                        self.last_action = (
                            "STOP_AFTER_SAFETY_TIMEOUT "
                            f"SECONDS={elapsed_seconds:.1f}"
                        )

                    print(
                        "Lane 1 stopped because the temporary "
                        "safety timeout was reached."
                    )

                    return

                # Refresh Arduino's continuous-forward safety
                # heartbeat before requesting another distance.
                self.arduino.refresh_continuous_forward()

                distance_cm = (
                    self.arduino.read_distance_cm()
                )

                self._save_distance(distance_cm)

                if not self._distance_is_valid(
                    distance_cm
                ):
                    self.arduino.stop()
                    forward_started = False

                    raise RuntimeError(
                        "Ultrasonic distance became invalid "
                        "while driving lane 1."
                    )

                with self.state_lock:
                    self.status = "DRIVING_LANE_1"
                    self.last_action = (
                        "MONITOR_WALL "
                        f"DISTANCE_CM={distance_cm:.1f}"
                    )

                if (
                    distance_cm
                    <= self.WALL_STOP_DISTANCE_CM
                ):
                    self.arduino.stop()
                    forward_started = False

                    with self.state_lock:
                        self.status = "LANE_1_COMPLETE"
                        self.last_action = (
                            "WALL_REACHED "
                            f"DISTANCE_CM={distance_cm:.1f}"
                        )

                    print(
                        "Lane 1 complete. "
                        f"Wall detected at {distance_cm:.1f} cm."
                    )

                    # Let the chassis and IMU become stationary before
                    # starting the first lane-transition turn.
                    time.sleep(self.TURN_SETTLE_SECONDS)

                    if self.stop_event.is_set():
                        return

                    with self.state_lock:
                        self.status = "TURNING_LEFT_AFTER_LANE_1"
                        self.last_action = (
                            "TURN_LEFT "
                            f"REQUESTED_ANGLE={self.FIRST_TURN_ANGLE_DEGREES:.1f}"
                        )

                    final_turn_angle = self.arduino.turn_left(
                        self.FIRST_TURN_ANGLE_DEGREES
                    )

                    with self.state_lock:
                        self.last_turn_angle = final_turn_angle
                        self.status = "FIRST_LEFT_TURN_COMPLETE"
                        self.last_action = (
                            "TURN_LEFT_COMPLETE "
                            f"FINAL_ANGLE={final_turn_angle:.2f}"
                        )

                        # The robot is currently sideways during the
                        # transition, so lane 2 has not started yet.
                        self.bins_direction = "LEFT"

                    print(
                        "First lane-transition turn complete. "
                        f"Final angle: {final_turn_angle:.2f} degrees."
                    )

                    # Allow the robot to become stationary before shifting.
                    time.sleep(
                        self.BEFORE_SHIFT_SETTLE_SECONDS
                    )

                    if self.stop_event.is_set():
                        return

                    with self.state_lock:
                        self.status = "SHIFTING_TO_LANE_2"
                        self.last_action = (
                            "FORWARD_LANE_SHIFT "
                            f"REQUESTED_MS={self.LANE_SHIFT_DURATION_MS}"
                        )

                    completed_shift_ms = self.arduino.forward(
                        self.LANE_SHIFT_DURATION_MS
                    )

                    with self.state_lock:
                        self.last_shift_duration_ms = completed_shift_ms
                        self.status = "LANE_SHIFT_COMPLETE"
                        self.last_action = (
                            "FORWARD_LANE_SHIFT_COMPLETE "
                            f"DURATION_MS={completed_shift_ms}"
                        )

                    print(
                        "Lane shift complete. "
                        f"Duration: {completed_shift_ms} ms."
                    )

                    # Stop completely before the second IMU turn.
                    time.sleep(
                        self.AFTER_SHIFT_SETTLE_SECONDS
                    )

                    if self.stop_event.is_set():
                        return

                    with self.state_lock:
                        self.status = "TURNING_LEFT_TO_FACE_LANE_2"
                        self.last_action = (
                            "TURN_LEFT_TO_LANE_2 "
                            f"REQUESTED_ANGLE={self.SECOND_TURN_ANGLE_DEGREES:.1f}"
                        )

                    second_turn_angle = self.arduino.turn_left(
                        self.SECOND_TURN_ANGLE_DEGREES
                    )

                    with self.state_lock:
                        self.second_turn_angle = second_turn_angle

                        # The robot is now aligned with the second lane.
                        self.lane_number = 2
                        self.bins_direction = "AHEAD"

                        self.status = "LANE_2_READY"
                        self.last_action = (
                            "LANE_2_ALIGNMENT_COMPLETE "
                            f"FINAL_ANGLE={second_turn_angle:.2f}"
                        )

                    print(
                        "Lane 2 alignment complete. "
                        f"Second turn angle: {second_turn_angle:.2f} degrees."
                    )

                    return

                time.sleep(
                    self.DISTANCE_CHECK_DELAY_SECONDS
                )

        except Exception as error:
            if self.stop_event.is_set():
                with self.state_lock:
                    self.status = "STOPPED"
                    self.last_action = "STOP"
            else:
                with self.state_lock:
                    self.error = str(error)
                    self.status = "ERROR"
                    self.last_action = (
                        "STOP_AFTER_ERROR"
                    )

                print(
                    "Deep-cleaning lane 1 error: "
                    f"{error}"
                )

        finally:
            if forward_started:
                try:
                    self.arduino.stop()
                except Exception as error:
                    print(
                        "Final deep-cleaning STOP warning: "
                        f"{error}"
                    )

            with self.state_lock:
                self.active = False

            print(
                "Deep-cleaning lane 1 test finished. "
                f"Status: {self.status}"
            )

    def stop(self) -> None:
        self.stop_event.set()

        try:
            self.arduino.stop()
        except Exception as error:
            print(
                "Deep-cleaning stop warning: "
                f"{error}"
            )

        with self.state_lock:
            self.active = False
            self.status = "STOPPED"
            self.last_action = "STOP"

    def is_running(self) -> bool:
        with self.state_lock:
            return self.active

    def get_status(self) -> dict:
        with self.state_lock:
            return {
                "running": self.active,
                "status": self.status,
                "last_action": self.last_action,
                "error": self.error,
                "lane_number": self.lane_number,
                "bins_direction": self.bins_direction,
                "last_distance_cm":
                    self.last_distance_cm,
                "lane_drive_seconds": round(
                    self.lane_drive_seconds,
                    2,
                ),
                "last_turn_angle": self.last_turn_angle,
                "first_turn_angle_degrees":
                    self.FIRST_TURN_ANGLE_DEGREES,
                "second_turn_angle_degrees":
                    self.SECOND_TURN_ANGLE_DEGREES,
                "wall_stop_distance_cm":
                    self.WALL_STOP_DISTANCE_CM,
                "maximum_test_seconds":
                    self.MAX_LANE_DRIVE_SECONDS,

                "last_shift_duration_ms":
                    self.last_shift_duration_ms,
                "configured_shift_duration_ms":
                    self.LANE_SHIFT_DURATION_MS,
                "second_turn_angle":
                    self.second_turn_angle,    
            }

    def _save_distance(
        self,
        distance_cm,
    ) -> None:
        with self.state_lock:
            self.last_distance_cm = distance_cm

    @staticmethod
    def _distance_is_valid(
        distance_cm,
    ) -> bool:
        return (
            distance_cm is not None
            and distance_cm > 0
        )