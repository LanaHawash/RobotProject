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

    FIRST_TURN_ANGLE_DEGREES = 83.0
    SECOND_TURN_ANGLE_DEGREES = 83.0
    TURN_SETTLE_SECONDS = 1.0

    # Temporary lane-shift duration for calibration.
    # Increase or decrease this after observing the physical distance.
    LANE_SHIFT_DURATION_MS = 400

    SHIFT_SETTLE_SECONDS = 2.0
    BEFORE_SHIFT_SETTLE_SECONDS = 1.0
    AFTER_SHIFT_SETTLE_SECONDS = 2.0

    # Temporary end condition until automatic room-width
    # detection is implemented.
    MAX_LANES = 4

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
        
        

        self.completed_lanes = 0
        self.transition_direction = None
        self.next_lane_number = None
        self.lane_travel_direction = None


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
            self.completed_lanes = 0

            self.bins_direction = "BEHIND"
            self.lane_travel_direction = "AWAY_FROM_BINS"

            self.transition_direction = None
            self.next_lane_number = None

            self.last_distance_cm = None
            self.lane_drive_seconds = 0.0
            self.last_turn_angle = None
            self.last_shift_duration_ms = None
            self.second_turn_angle = None

        self.cleaning_thread = threading.Thread(
            target=self._run_cleaning_pattern,
            daemon=True,
        )

        self.cleaning_thread.start()
 
    def _run_cleaning_pattern(self) -> None:
        print("Deep-cleaning ZikZak mode started.")

        try:
            while not self.stop_event.is_set():

                lane_completed = (
                    self._drive_current_lane_until_wall()
                )

                if not lane_completed:
                    return

                if self.stop_event.is_set():
                    return

                with self.state_lock:
                    current_lane = self.lane_number
                    self.completed_lanes = current_lane

                print(
                    f"Lane {current_lane} completed."
                )

                # Temporary completion rule.
                # Later Stage 5 will replace this with
                # actual remaining-side-space detection.
                if current_lane >= self.MAX_LANES:
                    with self.state_lock:
                        self.status = "CLEANING_COMPLETE"
                        self.last_action = (
                            "ALL_CONFIGURED_LANES_COMPLETE "
                            f"COUNT={current_lane}"
                        )

                    print(
                        "Deep cleaning complete. "
                        f"Completed {current_lane} lanes."
                    )

                    return

                transition_completed = (
                    self._transition_to_next_lane()
                )

                if not transition_completed:
                    return

        except Exception as error:
            if self.stop_event.is_set():
                with self.state_lock:
                    self.status = "STOPPED"
                    self.last_action = "STOP"

            else:
                with self.state_lock:
                    self.error = str(error)
                    self.status = "ERROR"
                    self.last_action = "STOP_AFTER_ERROR"

                print(
                    "Deep-cleaning error: "
                    f"{error}"
                )

            try:
                self.arduino.stop()
            except Exception as stop_error:
                print(
                    "Deep-cleaning error STOP warning: "
                    f"{stop_error}"
                )

        finally:
            with self.state_lock:
                self.active = False

                if self.stop_event.is_set():
                    self.status = "STOPPED"
                    self.last_action = "STOP"
            print(
                "Deep-cleaning ZikZak mode finished. "
                f"Status: {self.status}"
            )

    def _drive_current_lane_until_wall(self) -> bool:
        with self.state_lock:
            lane_number = self.lane_number

            self.lane_drive_seconds = 0.0

            self.status = (
                f"CHECKING_LANE_{lane_number}_START"
            )

            self.last_action = (
                f"CHECK_LANE_{lane_number}_DISTANCE"
            )

        initial_distance = (
            self.arduino.read_distance_cm()
        )

        self._save_distance(initial_distance)

        if not self._distance_is_valid(
            initial_distance
        ):
            raise RuntimeError(
                f"No valid ultrasonic distance before "
                f"lane {lane_number} movement."
            )

        if (
            initial_distance
            <= self.WALL_STOP_DISTANCE_CM
        ):
            with self.state_lock:
                self.status = (
                    f"LANE_{lane_number}_WALL_ALREADY_CLOSE"
                )

                self.last_action = (
                    "NO_MOVEMENT "
                    f"DISTANCE_CM={initial_distance:.1f}"
                )

            return False

        if self.stop_event.is_set():
            return False

        self.arduino.start_continuous_forward()

        forward_started = True
        drive_start_time = time.monotonic()
        invalid_distance_readings = 0
        MAX_INVALID_DISTANCE_READINGS = 3

        try:
            with self.state_lock:
                self.status = (
                    f"DRIVING_LANE_{lane_number}"
                )

                self.last_action = (
                    "START_CONTINUOUS_FORWARD "
                    f"LANE={lane_number} "
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
                            f"LANE_{lane_number}_"
                            "SAFETY_TIMEOUT"
                        )

                        self.last_action = (
                            "STOP_AFTER_SAFETY_TIMEOUT "
                            f"LANE={lane_number} "
                            f"SECONDS={elapsed_seconds:.1f}"
                        )

                    return False

                self.arduino.refresh_continuous_forward()

                distance_cm = (
                    self.arduino.read_distance_cm()
                )

                self._save_distance(distance_cm)

                if not self._distance_is_valid(
                    distance_cm
                ):
                    invalid_distance_readings += 1

                    with self.state_lock:
                        self.status = (
                            f"DRIVING_LANE_{lane_number}"
                        )

                        self.last_action = (
                            "TEMPORARY_DISTANCE_INVALID "
                            f"LANE={lane_number} "
                            f"COUNT={invalid_distance_readings}"
                        )

                    print(
                        f"Lane {lane_number}: temporary "
                        "invalid ultrasonic reading "
                        f"({invalid_distance_readings}/"
                        f"{MAX_INVALID_DISTANCE_READINGS})."
                    )

                    if (
                        invalid_distance_readings
                        >= MAX_INVALID_DISTANCE_READINGS
                    ):
                        self.arduino.stop()
                        forward_started = False

                        raise RuntimeError(
                            "Ultrasonic distance remained invalid "
                            f"while driving lane {lane_number}."
                        )

                    time.sleep(
                        self.DISTANCE_CHECK_DELAY_SECONDS
                    )

                    continue

                invalid_distance_readings = 0

                with self.state_lock:
                    self.status = (
                        f"DRIVING_LANE_{lane_number}"
                    )

                    self.last_action = (
                        "MONITOR_WALL "
                        f"LANE={lane_number} "
                        f"DISTANCE_CM={distance_cm:.1f}"
                    )

                if (
                    distance_cm
                    <= self.WALL_STOP_DISTANCE_CM
                ):
                    self.arduino.stop()
                    forward_started = False

                    with self.state_lock:
                        self.status = (
                            f"LANE_{lane_number}_COMPLETE"
                        )

                        self.last_action = (
                            "WALL_REACHED "
                            f"LANE={lane_number} "
                            f"DISTANCE_CM={distance_cm:.1f}"
                        )

                    print(
                        f"Lane {lane_number} complete. "
                        f"Wall detected at "
                        f"{distance_cm:.1f} cm."
                    )

                    return True

                time.sleep(
                    self.DISTANCE_CHECK_DELAY_SECONDS
                )

            return False

        finally:
            if forward_started:
                try:
                    self.arduino.stop()
                except Exception as error:
                    print(
                        "Lane STOP warning: "
                        f"{error}"
                    )


    def _transition_to_next_lane(self) -> bool:
        with self.state_lock:
            current_lane = self.lane_number

        next_lane = current_lane + 1

        if current_lane % 2 == 1:
            direction = "LEFT"
        else:
            direction = "RIGHT"

        with self.state_lock:
            self.transition_direction = direction
            self.next_lane_number = next_lane

            self.status = (
                f"PREPARING_TRANSITION_"
                f"{current_lane}_TO_{next_lane}"
            )

            self.last_action = (
                f"TRANSITION_{direction}"
            )

        time.sleep(
            self.TURN_SETTLE_SECONDS
        )

        if self.stop_event.is_set():
            return False

        # --------------------------------
        # FIRST TURN
        # --------------------------------

        with self.state_lock:
            self.status = (
                f"TURNING_{direction}_"
                f"AFTER_LANE_{current_lane}"
            )

            self.last_action = (
                f"TURN_{direction} "
                f"REQUESTED_ANGLE="
                f"{self.FIRST_TURN_ANGLE_DEGREES:.1f}"
            )

        if direction == "LEFT":
            first_turn_angle = (
                self.arduino.turn_left(
                    self.FIRST_TURN_ANGLE_DEGREES
                )
            )
        else:
            first_turn_angle = (
                self.arduino.turn_right(
                    self.FIRST_TURN_ANGLE_DEGREES
                )
            )

        with self.state_lock:
            self.last_turn_angle = first_turn_angle

            # During the sideways lane shift,
            # the bins are on the robot's left
            # under our agreed starting geometry.
            self.bins_direction = "LEFT"

            self.status = (
                f"FIRST_{direction}_TURN_COMPLETE"
            )

            self.last_action = (
                f"TURN_{direction}_COMPLETE "
                f"FINAL_ANGLE={first_turn_angle:.2f}"
            )

        if self.stop_event.is_set():
            return False

        time.sleep(
            self.BEFORE_SHIFT_SETTLE_SECONDS
        )

        # --------------------------------
        # LANE SHIFT
        # --------------------------------

        with self.state_lock:
            self.status = (
                f"SHIFTING_TO_LANE_{next_lane}"
            )

            self.last_action = (
                "FORWARD_LANE_SHIFT "
                f"FROM={current_lane} "
                f"TO={next_lane} "
                f"REQUESTED_MS="
                f"{self.LANE_SHIFT_DURATION_MS}"
            )

        completed_shift_ms = (
            self.arduino.forward(
                self.LANE_SHIFT_DURATION_MS
            )
        )

        with self.state_lock:
            self.last_shift_duration_ms = (
                completed_shift_ms
            )

            self.status = "LANE_SHIFT_COMPLETE"

            self.last_action = (
                "FORWARD_LANE_SHIFT_COMPLETE "
                f"DURATION_MS={completed_shift_ms}"
            )

        if self.stop_event.is_set():
            return False

        time.sleep(
            self.AFTER_SHIFT_SETTLE_SECONDS
        )

        # --------------------------------
        # SECOND TURN
        # --------------------------------

        with self.state_lock:
            self.status = (
                f"TURNING_{direction}_"
                f"TO_FACE_LANE_{next_lane}"
            )

            self.last_action = (
                f"TURN_{direction}_TO_LANE_"
                f"{next_lane} "
                f"REQUESTED_ANGLE="
                f"{self.SECOND_TURN_ANGLE_DEGREES:.1f}"
            )

        if direction == "LEFT":
            second_turn_angle = (
                self.arduino.turn_left(
                    self.SECOND_TURN_ANGLE_DEGREES
                )
            )
        else:
            second_turn_angle = (
                self.arduino.turn_right(
                    self.SECOND_TURN_ANGLE_DEGREES
                )
            )

        # --------------------------------
        # NOW THE NEXT LANE REALLY EXISTS
        # --------------------------------

        with self.state_lock:
            self.second_turn_angle = (
                second_turn_angle
            )

            self.lane_number = next_lane

            if next_lane % 2 == 1:
                self.bins_direction = "BEHIND"
                self.lane_travel_direction = (
                    "AWAY_FROM_BINS"
                )
            else:
                self.bins_direction = "AHEAD"
                self.lane_travel_direction = (
                    "TOWARD_BINS"
                )

            self.transition_direction = None
            self.next_lane_number = None

            self.status = (
                f"LANE_{next_lane}_READY"
            )

            self.last_action = (
                f"LANE_{next_lane}_"
                "ALIGNMENT_COMPLETE "
                f"FINAL_ANGLE="
                f"{second_turn_angle:.2f}"
            )

        print(
            f"Lane {next_lane} ready. "
            f"Travel direction: "
            f"{self.lane_travel_direction}."
        )

        time.sleep(
            self.TURN_SETTLE_SECONDS
        )

        return not self.stop_event.is_set()


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
            self.status = "STOPPING"
            self.last_action = "STOP_REQUESTED"

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

                "completed_lanes": self.completed_lanes,
                "next_lane_number": self.next_lane_number,
                "transition_direction": self.transition_direction,
                "lane_travel_direction": self.lane_travel_direction,
                "configured_max_lanes": self.MAX_LANES,     
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