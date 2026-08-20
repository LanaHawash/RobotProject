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


    MICRO_TURN_DEGREES = 1
    MICRO_TURN_MAX_ERROR_PX = 120
    SMALL_TURN_DEGREES = 3
    LARGE_TURN_DEGREES = 6

    TARGET_UPDATE_DELAY_SECONDS = 0.35
    MAX_LOST_TARGET_UPDATES = 8
    SEARCH_TURN_DEGREES = 30.0
    SEARCH_RETURN_TO_CENTER_DEGREES = 90.0
    MAX_SEARCH_TURNS_RIGHT = 3
    MAX_SEARCH_TURNS_LEFT = 3
    SEARCH_SETTLE_SECONDS = 0.6
    SEARCH_DETECTION_PAUSE_SECONDS = 2.0

    # The selected target must remain centered for two consecutive
    # camera updates before ultrasonic monitoring is allowed.
    REQUIRED_CENTERED_UPDATES = 2

    # When the ultrasonic cannot yet obtain a trustworthy reading,
    # make only small camera-approved search movements.
    

    # Ultrasonic-controlled normal approach.
    FAR_DISTANCE_CM = 50.0
    MEDIUM_DISTANCE_CM = 30.0
    PICKUP_POSITIONING_START_CM = 15.0
    # Arduino accepts pickup-positioning handoff only when its
    # initial ultrasonic readings are no farther than 16 cm.
    CAMERA_LOST_HANDOFF_MAX_CM = 16.0
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
    RETURN_DURATION_SCALE = 0.5

   
    
   


    BIN_CENTER_TOLERANCE_PX = 45
    REQUIRED_BIN_CENTERED_UPDATES = 3

    # Once the bin's color blob is at least this large, the robot is
    # close enough that even the smallest turn sweeps it across many
    # more pixels than the same turn would from a distance. Without
    # this, alignment keeps overshooting the tolerance band and
    # times out even though the bin is plainly in front of it.
    BIN_CLOSE_RANGE_AREA_PX = 35000
    BIN_CLOSE_RANGE_TOLERANCE_PX = 70

    BIN_ALIGNMENT_TURN_DEGREES = 3
    BIN_ALIGNMENT_DELAY_SECONDS = 0.25
    MAX_BIN_ALIGNMENT_UPDATES = 120

    BIN_SEARCH_UPDATE_DELAY_SECONDS = 0.25
    MAX_BIN_SEARCH_UPDATES = 10
    REQUIRED_BIN_DETECTIONS = 3

    BIN_SCAN_SETTLE_SECONDS = 0.75

    BIN_RELEASE_DISTANCE_CM = 10.0
    BIN_EMERGENCY_DISTANCE_CM = 5.0

    BIN_FAR_DISTANCE_CM = 45.0
    BIN_MEDIUM_DISTANCE_CM = 25.0

    BIN_FAR_FORWARD_MS = 180
    BIN_MEDIUM_FORWARD_MS = 100
    BIN_CLOSE_FORWARD_MS = 50

    MAX_BIN_APPROACH_UPDATES = 100
    BIN_APPROACH_DELAY_SECONDS = 0.20
    MAX_BIN_LOST_UPDATES = 8
    MAX_BIN_ALIGNMENT_LOST_UPDATES = 8

    

    NEXT_TARGET_CHECK_DELAY_SECONDS = 0.30

    def __init__(
        self,
        arduino: ArduinoController,
        get_target: Callable[[], Optional[dict]],
        get_raw_frame: Callable[[], object],
        get_confirmed_target: Callable[[], Optional[dict]],
        clear_confirmed_target: Callable[[], None],
    ):
        self.arduino = arduino
        self.get_target = get_target
        self.get_raw_frame = get_raw_frame
        self.get_confirmed_target = get_confirmed_target
        self.clear_confirmed_target = clear_confirmed_target
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
        
        self.last_ultrasonic_distance_cm = None
        self.close_range_mode = False

        self.last_pose = None

        self.continuous_forward_active = False
        self.last_forward_refresh_time = 0.0
        self.continuous_forward_start_time = None
        self.misaligned_updates = 0
        
        
    def start(self, selected_target: dict) -> None:
        if self.is_running():
            raise RuntimeError(
                "Navigation is already running."
            )

        self._lock_selected_target(selected_target)

        self.stop_event.clear()
        self.history.clear()

        self.error = None
        self.status = "STARTING"
        self.last_action = None

        self.pickup_distance_cm = None
        self.pickup_pulses = []

        self.centered_updates = 0
     
        self.last_ultrasonic_distance_cm = None
        self.close_range_mode = False

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
            "SEARCHING_FOR_BIN",
            "BIN_FOUND",
            "ALIGNING_WITH_BIN",
            "BIN_ALIGNED",
            "APPROACHING_BIN",
            "APPROACHING_BIN_CONTINUOUSLY",
            "APPROACHING_BIN_CONTROLLED",
            "BIN_RELEASE_POSITION_REACHED",
            "RELEASING_OBJECT",
            "OBJECT_RELEASED",
            "FACING_ENVIRONMENT",
            "READY_FOR_NEXT_OBJECT",
            "RECALIBRATING_IMU",
        }

    def movement_history(self) -> list[dict]:
        return self.history.snapshot()

    def simplified_history(self) -> list[dict]:
        return self.history.simplified()

    def return_route(self) -> list[dict]:
        return self.history.inverse_route()

    def _lock_selected_target(
        self,
        selected_target: dict,
    ) -> None:
        object_class = selected_target.get("label")
        destination = selected_target.get("destination")

        if not object_class or not destination:
            raise RuntimeError(
                "The confirmed target has no class or destination."
            )

        bin_color = DESTINATION_BIN_COLORS.get(
            destination
        )

        if bin_color is None:
            raise RuntimeError(
                "No bin color is configured for destination "
                f"'{destination}'."
            )

        self.locked_object_class = object_class
        self.locked_destination = destination
        self.locked_bin_color = bin_color

        print(
            "Navigation target locked: "
            f"object={self.locked_object_class}, "
            f"destination={self.locked_destination}, "
            f"bin_color={self.locked_bin_color}"
        )
    def _prepare_next_automatic_cycle(self) -> None:
      
        """
        Recalibrate and clear the completed cycle after the robot
        has returned home and faced the object-search environment.
        """
        if self.stop_event.is_set():
            return

        self.arduino.stop()

        self.status = "RECALIBRATING_IMU"
        self.last_action = "CALIBRATE_IMU"

        self.arduino.calibrate_imu()

        # Clear the completed home-to-bin-to-home route.
        self.history.clear()

        self.pickup_distance_cm = None
        self.pickup_pulses = []

        self.last_pose = None
        self.current_bin_target = None
        self.centered_updates = 0
        self.misaligned_updates = 0

        self.last_ultrasonic_distance_cm = None
        self.close_range_mode = False

        self.continuous_forward_active = False
        self.last_forward_refresh_time = 0.0
        self.continuous_forward_start_time = None

        self.locked_object_class = None
        self.locked_destination = None
        self.locked_bin_color = None

        self.clear_confirmed_target()

        self.error = None
        self.status = "WAITING_FOR_NEXT_TARGET"
        self.last_action = "WAITING_FOR_CONFIRMED_TARGET"

        print(
            "Robot returned home. "
            "Previous sorting cycle cleared. "
            "Waiting for a new confirmed target."
        )

        


    def _wait_for_next_confirmed_target(self) -> bool:
        """
        Wait for the detector to confirm a new object, then lock it
        automatically without requiring another Start button press.
        """
        while not self.stop_event.is_set():
            selected_target = (
                self._search_for_confirmed_target()
            )

            if selected_target is None:
                self.status = "WAITING_FOR_NEXT_TARGET"
                self.last_action = (
                    "SEARCH_COMPLETE_NO_TARGET"
                )

                time.sleep(
                    self.NEXT_TARGET_CHECK_DELAY_SECONDS
                )
                continue

            self._lock_selected_target(
                selected_target
            )

            self.current_bin_target = None

            self.pickup_distance_cm = None
            self.pickup_pulses = []
            self.last_pose = None

            self.centered_updates = 0
            self.misaligned_updates = 0
           
            self.last_ultrasonic_distance_cm = None
            self.close_range_mode = False

            self.continuous_forward_active = False
            self.last_forward_refresh_time = 0.0
            self.continuous_forward_start_time = None

            self.error = None
            self.status = "SEARCHING"
            self.last_action = "NEW_TARGET_LOCKED"

            print(
                "New target confirmed automatically. "
                "Starting the next sorting cycle."
            )

            return True

        return False
    def _navigation_loop(self) -> None:
        print("Target navigation started.")

        self.status = "SEARCHING"
        lost_target_updates = 0
        camera_loss_handoff_active = False

        try:
            while not self.stop_event.is_set():
                target = self.get_target()

                

                if target is None:
                    # Capture the alignment state only when the
                    # camera first loses the object. Preserve this
                    # permission during the following missing frames
                    # so ultrasonic gets more than one chance.
                    if lost_target_updates == 0:
                        camera_loss_handoff_active = (
                            camera_loss_handoff_active
                            or (
                                self.centered_updates
                                >= self.REQUIRED_CENTERED_UPDATES
                                and self.misaligned_updates == 0
                            )
                        )

                    self._stop_continuous_forward()
                    self.centered_updates = 0
                    lost_target_updates += 1

                    if camera_loss_handoff_active:
                        self.status = (
                            "CHECKING_CLOSE_ULTRASONIC_HANDOFF"
                        )
                        self.last_action = (
                            "READ_DISTANCE_AFTER_CAMERA_LOSS "
                            f"ATTEMPT={lost_target_updates}/"
                            f"{self.MAX_LOST_TARGET_UPDATES}"
                        )

                        distance_cm = (
                            self.arduino.read_distance_cm()
                        )

                        if not self._distance_is_unreliable(
                            distance_cm
                        ):
                            self.last_ultrasonic_distance_cm = (
                                distance_cm
                            )

                            if (
                                distance_cm
                                <= self.EMERGENCY_DISTANCE_CM
                            ):
                                raise RuntimeError(
                                    "ERROR,OBJECT_TOO_CLOSE,"
                                    f"DISTANCE_CM="
                                    f"{distance_cm:.1f}"
                                )

                            if (
                                distance_cm
                                <= self.CAMERA_LOST_HANDOFF_MAX_CM
                            ):
                                self.status = (
                                    "CAMERA_BLIND_ZONE_HANDOFF"
                                )
                                self.last_action = (
                                    "ULTRASONIC_TAKEOVER "
                                    f"DISTANCE_CM="
                                    f"{distance_cm:.1f}"
                                )

                                print(
                                    "Camera lost the centered "
                                    "object, but ultrasonic found "
                                    f"it at {distance_cm:.1f} cm."
                                )

                                if not self._run_pickup_handoff(
                                    distance_cm
                                ):
                                    break

                                # A failed positioning attempt may
                                # retry while the camera remains
                                # unable to see the close object.
                                if (
                                    self.status
                                    == "PICKUP_POSITIONING_RETRY"
                                ):
                                    if (
                                        lost_target_updates
                                        >= self.MAX_LOST_TARGET_UPDATES
                                    ):
                                        self.status = "TARGET_LOST"
                                        break

                                    time.sleep(
                                        self
                                        .TARGET_UPDATE_DELAY_SECONDS
                                    )
                                    continue

                                # A complete sorting cycle has ended.
                                lost_target_updates = 0
                                camera_loss_handoff_active = False
                                continue

                    self.status = "TARGET_NOT_AVAILABLE"
                    self.last_action = (
                        "WAITING_FOR_CAMERA_OR_CLOSE_ULTRASONIC"
                    )

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
                    camera_loss_handoff_active = False
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
                    camera_loss_handoff_active = False
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
                # The object is confirmed as centered. Preserve
                # ultrasonic-handoff permission if it disappears
                # beneath the camera after the next movement.
                camera_loss_handoff_active = True

                # READ_DISTANCE does not stop the motors. During
                # the far stage, the robot continues forward while
                # this reading is requested.
                self.status = "ULTRASONIC_MONITORING"
                self.last_action = "READ_DISTANCE"
                distance_cm = self.arduino.read_distance_cm()
                

                if self._distance_is_unreliable(
                    distance_cm
                ):
                    if self.close_range_mode:
                        # Once a valid reading has established that
                        # the robot is close, do not return to long
                        # continuous movement because of one failed
                        # ultrasonic reading.
                        self._stop_continuous_forward()

                        self.status = (
                            "WAITING_FOR_CLOSE_ULTRASONIC"
                        )
                        self.last_action = (
                            "CAMERA_CENTERED_DISTANCE_UNAVAILABLE"
                        )

                    else:
                        # During the far stage, the centered camera
                        # target remains authoritative. A failed
                        # ultrasonic reading does not interrupt the
                        # camera-approved forward segment.
                        self.status = (
                            "CAMERA_GUIDED_FAR_APPROACH"
                        )
                        self.last_action = (
                            "IGNORE_FAR_ULTRASONIC_MISREAD"
                        )

                        self._maintain_continuous_forward(
                            None
                        )

                        if (
                            self.continuous_forward_start_time
                            is not None
                            and (
                                time.monotonic()
                                - self.continuous_forward_start_time
                            )
                            >= (
                                self
                                .MAX_CONTINUOUS_FORWARD_SEGMENT_SECONDS
                            )
                        ):
                            self._stop_continuous_forward()
                            self.centered_updates = 0

                            self.status = (
                                "FORWARD_SEGMENT_COMPLETE"
                            )
                            self.last_action = (
                                "STOP_AFTER_CAMERA_GUIDED_SEGMENT "
                                "DISTANCE_CM=UNAVAILABLE"
                            )

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )
                    continue

                self.last_ultrasonic_distance_cm = (
                    distance_cm
                )

                if (
                    distance_cm
                    <= self.CONTINUOUS_FORWARD_MIN_DISTANCE_CM
                ):
                    # Once close range is reached, never switch
                    # back to long continuous movement because
                    # of a later sensor misreading.
                    self.close_range_mode = True
                

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
                    if not self._run_pickup_handoff(
                        distance_cm
                    ):
                        break

                    lost_target_updates = 0

                    # Preserve takeover permission only when pickup
                    # positioning failed and the same close object may
                    # still be beneath the camera.
                    if (
                        self.status
                        != "PICKUP_POSITIONING_RETRY"
                    ):
                        camera_loss_handoff_active = False

                    continue

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
        distance_cm: Optional[float],
    ) -> None:
        now = time.monotonic()
        distance_text = (
            f"{distance_cm:.1f}"
            if distance_cm is not None
            else "UNAVAILABLE"
        )        

        if not self.continuous_forward_active:
            self.arduino.start_continuous_forward()
            self.continuous_forward_active = True
            self.last_forward_refresh_time = now
            self.continuous_forward_start_time = now
            self.status = "SMOOTH_FORWARD"
            self.last_action = (
                "START_CONTINUOUS_FORWARD "
                f"DISTANCE_CM={distance_text}"
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
            f"DISTANCE_CM={distance_text}"
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

        error_px = abs(horizontal_error)

        if error_px <= self.MICRO_TURN_MAX_ERROR_PX:
            turn_angle = self.MICRO_TURN_DEGREES

        elif driving_correction:
            turn_angle = self.SMALL_TURN_DEGREES

        elif error_px > 220:
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


    def _search_turn(
        self,
        direction: str,
        angle: Optional[float] = None,
    ) -> None:

        turn_angle = (
            self.SEARCH_TURN_DEGREES
            if angle is None
            else float(angle)
        )

        if direction == "RIGHT":
            self.status = "SEARCHING_RIGHT"
            self.last_action = (
                f"SEARCH_TURN_RIGHT "
                f"{turn_angle}"
            )

            actual_angle = self.arduino.turn_right(
                turn_angle
            )

            self.history.record_turn(
                command="TURN_RIGHT",
                angle=actual_angle,
            )

        elif direction == "LEFT":
            self.status = "SEARCHING_LEFT"
            self.last_action = (
                f"SEARCH_TURN_LEFT "
                f"{turn_angle}"
            )

            actual_angle = self.arduino.turn_left(
                turn_angle
            )

            self.history.record_turn(
                command="TURN_LEFT",
                angle=actual_angle,
            )

        else:
            raise ValueError(
                f"Invalid search direction: {direction}"
            )

        time.sleep(
            self.SEARCH_SETTLE_SECONDS
        )

    def _search_for_confirmed_target(
        self,
    ) -> Optional[dict]:
        """
        Search for an object around the robot.

        Sequence:
            check original view

            right 30 degrees -> check
            right 30 degrees -> check
            right 30 degrees -> check

            return left 90 degrees to original heading

            left 30 degrees -> check
            left 30 degrees -> check
            left 30 degrees -> check

        If nothing is found, return right 90 degrees
        to the original heading before finishing.

        Every physical turn is recorded in MovementHistory.
        """

        # --------------------------------
        # CHECK ORIGINAL VIEW
        # --------------------------------

        selected_target = (
            self.get_confirmed_target()
        )

        if selected_target is not None:
            return selected_target

        # --------------------------------
        # SEARCH RIGHT
        # --------------------------------

        for _ in range(
            self.MAX_SEARCH_TURNS_RIGHT
        ):
            if self.stop_event.is_set():
                return None

            self._search_turn("RIGHT")

            # Give the camera/detector time to observe
            # the new direction before checking.
            time.sleep(
                self.SEARCH_DETECTION_PAUSE_SECONDS
            )

            selected_target = (
                self.get_confirmed_target()
            )

            if selected_target is not None:
                return selected_target

        # --------------------------------
        # RETURN TO ORIGINAL HEADING
        # --------------------------------

        if self.stop_event.is_set():
            return None

        self.status = "RETURNING_SEARCH_TO_CENTER"
        self.last_action = (
            "SEARCH_RETURN_TO_CENTER_LEFT "
            f"{self.SEARCH_RETURN_TO_CENTER_DEGREES}"
        )

        self._search_turn(
            "LEFT",
            self.SEARCH_RETURN_TO_CENTER_DEGREES,
        )

        # --------------------------------
        # SEARCH LEFT
        # --------------------------------

        for _ in range(
            self.MAX_SEARCH_TURNS_LEFT
        ):
            if self.stop_event.is_set():
                return None

            self._search_turn("LEFT")

            selected_target = (
                self.get_confirmed_target()
            )

            if selected_target is not None:
                return selected_target

        # --------------------------------
        # NOTHING FOUND:
        # RETURN TO ORIGINAL HEADING
        # --------------------------------

        if self.stop_event.is_set():
            return None

        self.status = "RETURNING_SEARCH_TO_CENTER"
        self.last_action = (
            "SEARCH_RETURN_TO_CENTER_RIGHT "
            f"{self.SEARCH_RETURN_TO_CENTER_DEGREES}"
        )

        self._search_turn(
            "RIGHT",
            self.SEARCH_RETURN_TO_CENTER_DEGREES,
        )

        return None
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


    def _bin_approach_duration(
        self,
        distance_cm: float,
    ) -> int:
        if distance_cm > self.BIN_FAR_DISTANCE_CM:
            return self.BIN_FAR_FORWARD_MS

        if distance_cm > self.BIN_MEDIUM_DISTANCE_CM:
            return self.BIN_MEDIUM_FORWARD_MS

        return self.BIN_CLOSE_FORWARD_MS

    def _run_pickup_handoff(
        self,
        distance_cm: float,
    ) -> bool:
        """
        Transfer control from camera-guided navigation to
        Arduino ultrasonic fine positioning.

        Returns False when navigation must stop. Returns True
        when the navigation loop may continue.
        """
        self._stop_continuous_forward()

        self.status = "POSITIONING_FOR_PICKUP"
        self.last_action = (
            "POSITION_FOR_PICKUP "
            f"DISTANCE_CM={distance_cm:.1f}"
        )

        print(
            "Starting ultrasonic pickup positioning at "
            f"{distance_cm:.1f} cm."
        )

        result = self.arduino.position_for_pickup()

        if not result["success"]:
            if self.stop_event.is_set():
                self.status = "STOPPED"
                self.last_action = "STOP"
                return False

            self.history.extend_pulses(
                result["pulses"]
            )

            reason = result.get(
                "reason",
                "UNKNOWN",
            )

            self.status = "PICKUP_POSITIONING_RETRY"
            self.last_action = (
                "WAIT_FOR_CAMERA_OR_ULTRASONIC "
                f"REASON={reason}"
            )

            print(
                "Pickup positioning was not completed: "
                f"{reason}"
            )

            self.centered_updates = 0

            time.sleep(
                self.TARGET_UPDATE_DELAY_SECONDS
            )

            return True

        self._complete_pickup_positioning(result)
        self._grab_object()

        # Use the object-route history to return home.
        self._return_to_start()
        self._face_bins()

        # The robot is now at home and facing the bins.
        # Make this the origin of a new movement history.
        self.history.clear()
        self.last_pose = None

        print(
            "Object route cleared. "
            "Starting a new home-to-bin movement history."
        )

        # Record a completely new route from home to the bin.
        self._find_locked_bin()
        self._align_with_locked_bin()
        self._approach_locked_bin()
        self._release_object()

        if self.stop_event.is_set():
            return False

        # Create clearance before turning near the bin.
        actual_duration = self.arduino.backward(300)

        self.history.record_linear(
            command="BACKWARD",
            duration_ms=actual_duration,
            source="BIN_EXIT",
        )

        # The history still represents the complete home-to-bin route,
        # including the small movement away from the bin.
        self._return_to_start()

        if self.stop_event.is_set():
            return False

        # After returning home, face the object-search area.
        self._face_object_environment()

        if self.stop_event.is_set():
            return False

        self._prepare_next_automatic_cycle()

        if self.stop_event.is_set():
            return False

        if not self._wait_for_next_confirmed_target():
            return False

        return True
        
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
        has reached the destination bin.
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

    def _face_object_environment(self) -> None:
        """
        Face the object-search area after returning from the bin.

        The bin-route history begins with heading 0 degrees while
        facing the bins. Therefore, the object-search environment
        is heading 180 degrees in this local coordinate system.
        """
        self.status = "FACING_ENVIRONMENT"

        pose = self.history.estimate_pose(
            self.CM_PER_MS
        )
        self.last_pose = pose

        turn_to_environment = self._normalize_angle(
            180.0 - pose["heading_degrees"]
        )

        self.last_action = (
            "FACE_ENVIRONMENT "
            f"TURN_DEGREES={turn_to_environment:.1f}"
        )

        self._execute_turn(
            turn_to_environment
        )

        self.status = "READY_FOR_NEXT_OBJECT"
        self.last_action = "READY_FOR_NEXT_OBJECT"

        print(
            "Facing the object-search environment after "
            f"turning {turn_to_environment:.1f} degrees."
        )

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
        Search for the locked-color bin.

        The robot first checks the current camera view. If the bin is not
        confirmed, it performs a small right/left scan before declaring
        that the bin cannot be found.
        """
        if not self.locked_bin_color:
            raise RuntimeError(
                "Cannot search for a bin because no bin color is locked."
            )

        self.status = "SEARCHING_FOR_BIN"
        self.current_bin_target = None

        # Search directions:
        # 0 degrees means the current direction.
        # The later turns scan to both sides and finally return near
        # the original direction.
        search_turns = [
            None,
            ("TURN_RIGHT", 30),
            ("TURN_LEFT", 60),
            ("TURN_RIGHT", 30),
        ]

        for search_step, turn_instruction in enumerate(search_turns):
            if self.stop_event.is_set():
                raise RuntimeError(
                    "Bin search was stopped."
                )

            if turn_instruction is not None:
                command, angle = turn_instruction

                self.last_action = (
                    f"SEARCH_BIN_SCAN {command} {angle}"
                )

                if command == "TURN_RIGHT":
                    actual_angle = self.arduino.turn_right(angle)

                    self.history.record_turn(
                        command="TURN_RIGHT",
                        angle=actual_angle,
                    )
                else:
                    actual_angle = self.arduino.turn_left(angle)

                    self.history.record_turn(
                        command="TURN_LEFT",
                        angle=actual_angle,
                    )

                time.sleep(
                    self.BIN_SCAN_SETTLE_SECONDS
                )

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
                        f"STEP={search_step + 1}/"
                        f"{len(search_turns)} "
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
            "The locked bin was not found after scanning "
            f"both directions: {self.locked_bin_color}"
        )  

    def _align_with_locked_bin(self) -> dict:
        """
        Turn in small steps until the locked-color bin is centered.

        Camera orientation in this project:
        image left  -> turn robot right
        image right -> turn robot left
        """
        if not self.locked_bin_color:
            raise RuntimeError(
                "Cannot align with a bin because no bin color is locked."
            )

        self.status = "ALIGNING_WITH_BIN"

        centered_updates = 0
        lost_updates = 0

        for _ in range(self.MAX_BIN_ALIGNMENT_UPDATES):
            if self.stop_event.is_set():
                raise RuntimeError(
                    "Bin alignment was stopped."
                )

            detection = self._get_locked_bin_detection()

            if detection is None:
                centered_updates = 0
                lost_updates += 1

                self.last_action = (
                    "BIN_NOT_VISIBLE "
                    f"{lost_updates}/"
                    f"{self.MAX_BIN_ALIGNMENT_LOST_UPDATES}"
                )

                # Allow several missing frames before attempting recovery.
                if (
                    lost_updates
                    < self.MAX_BIN_ALIGNMENT_LOST_UPDATES
                ):
                    time.sleep(
                        self.BIN_ALIGNMENT_DELAY_SECONDS
                    )
                    continue

                self.last_action = (
                    "RECOVERING_LOST_BIN_ALIGNMENT"
                )

                # Search left, then across to the right, then approximately
                # return to the original heading.
                recovery_turns = [
                    ("TURN_LEFT", 6),
                    ("TURN_RIGHT", 12),
                    ("TURN_LEFT", 6),
                ]

                recovered_detection = None

                for command, angle in recovery_turns:
                    if self.stop_event.is_set():
                        raise RuntimeError(
                            "Bin alignment was stopped."
                        )

                    if command == "TURN_LEFT":
                        actual_angle = self.arduino.turn_left(
                            angle
                        )

                        self.history.record_turn(
                            command="TURN_LEFT",
                            angle=actual_angle,
                        )

                    else:
                        actual_angle = self.arduino.turn_right(
                            angle
                        )

                        self.history.record_turn(
                            command="TURN_RIGHT",
                            angle=actual_angle,
                        )

                    time.sleep(
                        self.BIN_ALIGNMENT_DELAY_SECONDS
                    )

                    # Check several frames after each recovery movement.
                    for _ in range(4):
                        if self.stop_event.is_set():
                            raise RuntimeError(
                                "Bin alignment was stopped."
                            )

                        recovered_detection = (
                            self._get_locked_bin_detection()
                        )

                        if recovered_detection is not None:
                            break

                        time.sleep(
                            self.BIN_ALIGNMENT_DELAY_SECONDS
                        )

                    if recovered_detection is not None:
                        break

                if recovered_detection is None:
                    self.current_bin_target = None

                    raise RuntimeError(
                        "The locked bin was lost during alignment "
                        "and could not be recovered by scanning."
                    )

                detection = recovered_detection
                self.current_bin_target = detection

                lost_updates = 0
                centered_updates = 0

                self.last_action = (
                    "BIN_ALIGNMENT_RECOVERED "
                    f"CENTER_X={detection['center_x']}"
                )

            else:
                # A normal detection was received.
                self.current_bin_target = detection
                lost_updates = 0

            horizontal_error = (
                detection["center_x"]
                - self.FRAME_CENTER_X
            )

            # A large color blob means the bin is already close, so
            # the same turn angle sweeps it across far more pixels
            # than it would from a distance.
            is_close_range = (
                detection["area"]
                >= self.BIN_CLOSE_RANGE_AREA_PX
            )

            alignment_tolerance_px = (
                self.BIN_CLOSE_RANGE_TOLERANCE_PX
                if is_close_range
                else self.BIN_CENTER_TOLERANCE_PX
            )

            if (
                abs(horizontal_error)
                <= alignment_tolerance_px
            ):
                centered_updates += 1

                self.last_action = (
                    "BIN_CENTERED_CONFIRMATION "
                    f"{centered_updates}/"
                    f"{self.REQUIRED_BIN_CENTERED_UPDATES} "
                    f"ERROR_PX={horizontal_error}"
                )

                if (
                    centered_updates
                    >= self.REQUIRED_BIN_CENTERED_UPDATES
                ):
                    self.status = "BIN_ALIGNED"

                    self.last_action = (
                        "BIN_ALIGNED "
                        f"COLOR={self.locked_bin_color} "
                        f"CENTER_X={detection['center_x']}"
                    )

                    print(
                        "Bin alignment completed: "
                        f"color={self.locked_bin_color}, "
                        f"center_x={detection['center_x']}"
                    )

                    return detection

                time.sleep(
                    self.BIN_ALIGNMENT_DELAY_SECONDS
                )
                continue

            centered_updates = 0

            absolute_error = abs(horizontal_error)

            # Use large corrections when far away and small corrections
            # when close to the center to reduce left-right oscillation.
            # At close range, always use the smallest step regardless
            # of pixel error, since proximity already amplifies it.
            if is_close_range:
                turn_degrees = 1
            elif absolute_error > 180:
                turn_degrees = 3
            elif absolute_error > 100:
                turn_degrees = 2
            else:
                turn_degrees = 1

            if horizontal_error < 0:
                self.last_action = (
                    "ALIGN_BIN TURN_RIGHT "
                    f"{turn_degrees} "
                    f"ERROR_PX={horizontal_error}"
                )

                actual_angle = self.arduino.turn_right(
                    turn_degrees
                )

                self.history.record_turn(
                    command="TURN_RIGHT",
                    angle=actual_angle,
                )

            else:
                self.last_action = (
                    "ALIGN_BIN TURN_LEFT "
                    f"{turn_degrees} "
                    f"ERROR_PX={horizontal_error}"
                )

                actual_angle = self.arduino.turn_left(
                    turn_degrees
                )

                self.history.record_turn(
                    command="TURN_LEFT",
                    angle=actual_angle,
                )

            time.sleep(
                self.BIN_ALIGNMENT_DELAY_SECONDS
            )

        self.current_bin_target = None

        raise RuntimeError(
            "The robot could not align with the locked bin."
        )

    def _approach_locked_bin(self) -> float:
        """
        Move continuously toward the locked bin while monitoring
        camera alignment and ultrasonic distance.

        Continuous movement stops only when:
        - the bin is meaningfully misaligned,
        - the bin is temporarily lost,
        - the distance reading is unreliable,
        - the release distance is reached,
        - or an emergency distance is detected.
        """
        self.status = "APPROACHING_BIN"

        lost_updates = 0
        misaligned_updates = 0

        # Wider than the precise initial-alignment tolerance.
        # This prevents small camera shifts from causing constant stops.
        approach_alignment_tolerance_px = 90

        # Require repeated misalignment before correcting.
        required_misaligned_updates = 2

        for _ in range(self.MAX_BIN_APPROACH_UPDATES):
            if self.stop_event.is_set():
                self._stop_continuous_forward()

                raise RuntimeError(
                    "Bin approach was stopped."
                )

            detection = self._get_locked_bin_detection()

            if detection is None:
                lost_updates += 1

                # Never continue driving while the bin is invisible.
                self._stop_continuous_forward()

                self.last_action = (
                    "BIN_LOST_DURING_APPROACH "
                    f"{lost_updates}/{self.MAX_BIN_LOST_UPDATES}"
                )

                if lost_updates >= self.MAX_BIN_LOST_UPDATES:
                    raise RuntimeError(
                        "The locked bin was lost during approach."
                    )

                time.sleep(
                    self.BIN_APPROACH_DELAY_SECONDS
                )
                continue

            lost_updates = 0

            horizontal_error = (
                detection["center_x"]
                - self.FRAME_CENTER_X
            )

            if (
                abs(horizontal_error)
                > approach_alignment_tolerance_px
            ):
                misaligned_updates += 1

                self.last_action = (
                    "BIN_APPROACH_MISALIGNED "
                    f"{misaligned_updates}/"
                    f"{required_misaligned_updates} "
                    f"ERROR_PX={horizontal_error}"
                )

                # Ignore one isolated shifted camera frame.
                if (
                    misaligned_updates
                    < required_misaligned_updates
                ):
                    time.sleep(
                        self.BIN_APPROACH_DELAY_SECONDS
                    )
                    continue

                # Stop before making a steering correction.
                self._stop_continuous_forward()

                self.last_action = (
                    "BIN_APPROACH_REALIGN "
                    f"ERROR_PX={horizontal_error}"
                )

                if horizontal_error < 0:
                    actual_angle = self.arduino.turn_right(
                        self.BIN_ALIGNMENT_TURN_DEGREES
                    )

                    self.history.record_turn(
                        command="TURN_RIGHT",
                        angle=actual_angle,
                    )
                else:
                    actual_angle = self.arduino.turn_left(
                        self.BIN_ALIGNMENT_TURN_DEGREES
                    )

                    self.history.record_turn(
                        command="TURN_LEFT",
                        angle=actual_angle,
                    )

                misaligned_updates = 0

                time.sleep(
                    self.BIN_APPROACH_DELAY_SECONDS
                )
                continue

            misaligned_updates = 0

            distance_cm = self.arduino.read_distance_cm()

            if self._distance_is_unreliable(distance_cm):
                # Do not drive blindly without a trustworthy distance.
                self._stop_continuous_forward()
                self.last_ultrasonic_distance_cm = (
                    distance_cm
                )
                self.last_action = (
                    "BIN_DISTANCE_UNRELIABLE"
                )

                time.sleep(
                    self.BIN_APPROACH_DELAY_SECONDS
                )
                continue

            if distance_cm <= self.BIN_EMERGENCY_DISTANCE_CM:
                self._stop_continuous_forward()

                raise RuntimeError(
                    "Emergency stop during bin approach: "
                    f"{distance_cm:.1f} cm."
                )

            if distance_cm <= self.BIN_RELEASE_DISTANCE_CM:
                self._stop_continuous_forward()

                self.status = "BIN_RELEASE_POSITION_REACHED"
                self.last_action = (
                    "BIN_RELEASE_POSITION_REACHED "
                    f"DISTANCE_CM={distance_cm:.1f}"
                )

                print(
                    "Bin release position reached at "
                    f"{distance_cm:.1f} cm."
                )

                return distance_cm

            if distance_cm > self.BIN_FAR_DISTANCE_CM:
                # Far from the bin: smooth continuous forward is
                # safe here, same as the far-range object approach.
                self._maintain_continuous_forward(
                    distance_cm
                )

                self.status = "APPROACHING_BIN_CONTINUOUSLY"
                self.last_action = (
                    "APPROACH_BIN_CONTINUOUS "
                    f"DISTANCE_CM={distance_cm:.1f} "
                    f"ERROR_PX={horizontal_error}"
                )

                time.sleep(
                    self.BIN_APPROACH_DELAY_SECONDS
                )
                continue

            # Close enough that continuous full-speed driving could
            # cover the remaining gap to BIN_EMERGENCY_DISTANCE_CM
            # between two ultrasonic reads. Stop and creep in with
            # short pulses instead, re-checking distance after each
            # one, the same way pickup positioning does for objects.
            self._stop_continuous_forward()

            pulse_ms = self._bin_approach_duration(
                distance_cm
            )

            self.status = "APPROACHING_BIN_CONTROLLED"
            self.last_action = (
                "APPROACH_BIN_PULSE "
                f"{pulse_ms} "
                f"DISTANCE_CM={distance_cm:.1f}"
            )

            actual_duration = self.arduino.forward(
                pulse_ms
            )

            self.history.record_linear(
                command="FORWARD",
                duration_ms=actual_duration,
                source="BIN_APPROACH",
            )

        self._stop_continuous_forward()

        raise RuntimeError(
            "The robot did not reach the bin release distance."
        )

    def _get_locked_bin_detection(self):
        frame = self.get_raw_frame()

        if frame is None:
            return None

        detection = self.bin_color_detector.detect(
            frame,
            self.locked_bin_color,
        )

        self.current_bin_target = (
            detection.copy()
            if detection is not None
            else None
        )

        return detection  

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