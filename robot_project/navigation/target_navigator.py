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
from robot_project.navigation.obstacle_avoidance import (
    ObstacleAvoidance,
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
    OBJECT_RETURN_DURATION_SCALE = 0.52
    BIN_RETURN_DURATION_SCALE = 0.40

   
    
   


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

    # Bin discovery scans the full area behind the robot from
    # 90 degrees right to 90 degrees left, in 30-degree steps.
    BIN_SEARCH_STEP_DEGREES = 30.0
    BIN_SEARCH_MAX_OFFSET_DEGREES = 90.0
    BIN_SCAN_SETTLE_SECONDS = 0.75

    # If a previously visible bin disappears during fine alignment,
    # perform a wider local sweep around the current heading instead
    # of the old 6/12/6-degree recovery.
    BIN_ALIGNMENT_RECOVERY_STEP_DEGREES = 5.0
    BIN_ALIGNMENT_RECOVERY_MAX_OFFSET_DEGREES = 20.0
    BIN_ALIGNMENT_RECOVERY_CHECKS_PER_STEP = 3

    BIN_RELEASE_DISTANCE_CM = 10.0
    BIN_EMERGENCY_DISTANCE_CM = 5.0

    BIN_FAR_DISTANCE_CM = 45.0
    BIN_MEDIUM_DISTANCE_CM = 25.0

    # Bin approach is intentionally pulse-based. The bins can sit beside
    # one another, so the camera must re-confirm the locked color after
    # every short movement instead of driving continuously toward whatever
    # the front ultrasonic sensor happens to see.
    BIN_FAR_FORWARD_MS = 180
    BIN_MEDIUM_FORWARD_MS = 120
    BIN_CLOSE_FORWARD_MS = 80

    # Keep the locked-color bin tighter to the camera center while walking.
    BIN_APPROACH_ALIGNMENT_TOLERANCE_PX = 55
    BIN_APPROACH_REALIGN_DEGREES = 1

    # A close ultrasonic reading is accepted as the destination bin only
    # when the locked-color blob is also centered and visibly close.
    BIN_RELEASE_VISUAL_CENTER_TOLERANCE_PX = 45
    MAX_BIN_CLOSE_UNCONFIRMED_UPDATES = 8

    MAX_BIN_APPROACH_UPDATES = 140
    BIN_APPROACH_DELAY_SECONDS = 0.20
    MAX_BIN_LOST_UPDATES = 8
    MAX_BIN_ALIGNMENT_LOST_UPDATES = 8

    

    NEXT_TARGET_CHECK_DELAY_SECONDS = 2.0
    # ==================================================
    # IMU DIAGNOSTIC SETTINGS
    # ==================================================

    # Diagnostic 1:
    # False = keep the original startup IMU bias for every sorting cycle.
    # True  = recalibrate between sorting cycles as normal.
    ENABLE_RUNTIME_IMU_RECALIBRATION = False

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
        

        self.obstacle_avoidance = ObstacleAvoidance(
            arduino=self.arduino,
            history=self.history,
        )

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

        self.sorting_cycle_number = 0
        
        
    def start(self, selected_target: dict) -> None:
        if self.is_running():
            raise RuntimeError(
                "Navigation is already running."
            )

        self.sorting_cycle_number = 1

        print(
            "\n"
            "========================================\n"
            f"SORTING CYCLE {self.sorting_cycle_number} START\n"
            "========================================"
        )

        self._lock_selected_target(selected_target)

        self.stop_event.clear()
        self.history.clear()
        self.obstacle_avoidance.reset()

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
            self._return_to_start(
                self.OBJECT_RETURN_DURATION_SCALE
            )
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

        if self.ENABLE_RUNTIME_IMU_RECALIBRATION:
            self.last_action = "CALIBRATE_IMU"

            print(
                "\n"
                "IMU DIAGNOSTIC: "
                f"recalibrating after cycle "
                f"{self.sorting_cycle_number}."
            )

            self.arduino.calibrate_imu()

        else:
            self.last_action = (
                "CALIBRATE_IMU_SKIPPED_DIAGNOSTIC"
            )

            print(
                "\n"
                "IMU DIAGNOSTIC: "
                "runtime recalibration SKIPPED. "
                f"Cycle {self.sorting_cycle_number + 1} "
                "will continue using the existing "
                "startup gyro bias."
            )

        # Clear the completed home-to-bin-to-home route.
        self.history.clear()
        self.obstacle_avoidance.reset()

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

        
    def _wait_for_confirmed_target(
        self,
        duration_seconds: float,
    ) -> Optional[dict]:
        deadline = (
            time.monotonic()
            + duration_seconds
        )

        while (
            not self.stop_event.is_set()
            and time.monotonic() < deadline
        ):
            selected_target = (
                self.get_confirmed_target()
            )

            if selected_target is not None:
                return selected_target

            time.sleep(0.10)

        return None


    def _wait_for_next_confirmed_target(self) -> bool:
        """
        Wait for a newly confirmed object.

        First watch the current view while stationary.
        If nothing appears, perform one physical scan.
        After an unsuccessful scan, stop and wait before
        trying again.
        """

        while not self.stop_event.is_set():

            # --------------------------------
            # WATCH CURRENT VIEW FIRST
            # --------------------------------

            self.arduino.stop()

            self.status = "WAITING_FOR_NEXT_TARGET"
            self.last_action = (
                "WATCHING_ENVIRONMENT"
            )

            selected_target = (
                self._wait_for_confirmed_target(
                    3.0
                )
            )

            # --------------------------------
            # NOTHING STRAIGHT AHEAD:
            # PERFORM ONE SEARCH
            # --------------------------------

            if selected_target is None:
                selected_target = (
                    self._search_for_confirmed_target()
                )

            # --------------------------------
            # NOTHING FOUND
            # --------------------------------

            if selected_target is None:
                self.arduino.stop()

                self.status = "WAITING_FOR_NEXT_TARGET"
                self.last_action = (
                    "NO_TARGET_FOUND_WAITING"
                )

                # Stay stationary before another scan.
                time.sleep(2.0)

                continue

            # --------------------------------
            # NEW OBJECT FOUND
            # --------------------------------

            self.sorting_cycle_number += 1

            print(
                "\n"
                "========================================\n"
                f"SORTING CYCLE {self.sorting_cycle_number} START\n"
                "========================================"
            )

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

                # --------------------------------------------------
                # EMERGENCY STOP ALWAYS HAS HIGHEST PRIORITY
                # --------------------------------------------------

                if distance_cm <= self.EMERGENCY_DISTANCE_CM:
                    self._stop_continuous_forward()

                    raise RuntimeError(
                        "ERROR,OBJECT_TOO_CLOSE,"
                        f"DISTANCE_CM={distance_cm:.1f}"
                    )

                # --------------------------------------------------
                # OBSTACLE AVOIDANCE
                # --------------------------------------------------
                #
                # The camera still sees the selected target and gives
                # us box_occupancy.
                #
                # If the selected target still looks visually far away
                # but ultrasonic sees something close in front of the
                # robot, ObstacleAvoidance treats that close object as
                # an obstacle instead of the selected pickup target.
                # --------------------------------------------------

                obstacle_confirmed = (
                    self.obstacle_avoidance.should_avoid(
                        distance_cm=distance_cm,
                        target_occupancy=box_occupancy,
                    )
                )

                if obstacle_confirmed:
                    # Stop through TargetNavigator so continuous-forward
                    # state and MovementHistory remain correct.
                    self._stop_continuous_forward()

                    self.status = "AVOIDING_OBSTACLE"
                    self.last_action = (
                        "AVOID_OBSTACLE_RIGHT "
                        f"DISTANCE_CM={distance_cm:.1f} "
                        f"TARGET_OCCUPANCY={box_occupancy:.3f}"
                    )

                    print(
                        "Obstacle confirmed during target approach: "
                        f"distance={distance_cm:.1f} cm, "
                        f"target_occupancy={box_occupancy:.3f}"
                    )

                    self.obstacle_avoidance.avoid_right()

                    # The robot has physically changed position during
                    # the detour. Do not assume the target is still
                    # centered. Give control back to the normal camera
                    # alignment logic.
                    self.centered_updates = 0
                    self.misaligned_updates = 0

                    self.last_ultrasonic_distance_cm = None

                    # The close ultrasonic reading belonged to the
                    # obstacle, not necessarily to the selected target.
                    self.close_range_mode = False

                    # Do not allow a close obstacle reading to trigger
                    # the camera-blind pickup handoff.
                    camera_loss_handoff_active = False

                    self.status = "SEARCHING"
                    self.last_action = (
                        "RESUME_TARGET_AFTER_OBSTACLE"
                    )

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )

                    continue

                # --------------------------------------------------
                # POSSIBLE OBSTACLE - WAIT FOR CONFIRMATION
                # --------------------------------------------------
                #
                # ObstacleAvoidance requires consecutive ultrasonic
                # confirmations. If this was only the first suspicious
                # reading, stop here instead of moving closer before
                # obtaining the next reading.
                # --------------------------------------------------

                if (
                    self.obstacle_avoidance.confirmation_count
                    > 0
                ):
                    self._stop_continuous_forward()

                    self.status = "CONFIRMING_OBSTACLE"
                    self.last_action = (
                        "WAIT_FOR_OBSTACLE_CONFIRMATION "
                        f"DISTANCE_CM={distance_cm:.1f} "
                        f"COUNT="
                        f"{self.obstacle_avoidance.confirmation_count}/"
                        f"{self.obstacle_avoidance.REQUIRED_CONFIRMATIONS}"
                    )

                    # This ultrasonic reading may belong to an obstacle,
                    # so do not let it activate close-target behavior.
                    self.close_range_mode = False
                    camera_loss_handoff_active = False

                    # Re-confirm camera alignment before taking the
                    # next obstacle measurement.
                    self.centered_updates = 0

                    time.sleep(
                        self.TARGET_UPDATE_DELAY_SECONDS
                    )

                    continue

                # --------------------------------------------------
                # NORMAL TARGET APPROACH CONTINUES UNCHANGED
                # --------------------------------------------------

                if (
                    distance_cm
                    <= self.CONTINUOUS_FORWARD_MIN_DISTANCE_CM
                ):
                    # Once close range is reached, never switch
                    # back to long continuous movement because
                    # of a later sensor misreading.
                    self.close_range_mode = True

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

        self.arduino.stop()

        self.status = "WATCHING_FOR_TARGET"
        self.last_action = (
            "WATCHING_CURRENT_VIEW"
        )

        selected_target = (
            self._wait_for_confirmed_target(
                3.0
            )
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

            selected_target = (
                self._wait_for_confirmed_target(
                    self.SEARCH_DETECTION_PAUSE_SECONDS
                )
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
                self._wait_for_confirmed_target(
                    self.SEARCH_DETECTION_PAUSE_SECONDS
                )
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
        self._return_to_start(
            self.OBJECT_RETURN_DURATION_SCALE
        )
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
        self._return_to_start(
            self.BIN_RETURN_DURATION_SCALE
)

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

    def _return_to_start(
        self,
        duration_scale: float,
    ) -> None:
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

        self._drive_distance_cm(
            distance_home_cm,
            duration_scale,
        )

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
        Search widely for the locked-color bin.

        Search pattern relative to the heading that faces the bins:

            0 -> +30 -> +60 -> +90
            return to 0
            -30 -> -60 -> -90
            return to 0 if nothing is found

        This covers a full 180-degree field behind the robot while
        still returning to the original heading when the search fails.
        Every physical turn is recorded in MovementHistory.
        """
        if not self.locked_bin_color:
            raise RuntimeError(
                "Cannot search for a bin because no bin color is locked."
            )

        expected_bin_color = DESTINATION_BIN_COLORS.get(
            self.locked_destination
        )

        if expected_bin_color != self.locked_bin_color:
            raise RuntimeError(
                "Locked bin state is inconsistent: "
                f"destination={self.locked_destination}, "
                f"locked_color={self.locked_bin_color}, "
                f"expected_color={expected_bin_color}."
            )

        self.status = "SEARCHING_FOR_BIN"
        self.current_bin_target = None

        print(
            "Starting wide locked-bin search: "
            f"destination={self.locked_destination}, "
            f"color={self.locked_bin_color}"
        )

        max_side_steps = int(
            self.BIN_SEARCH_MAX_OFFSET_DEGREES
            / self.BIN_SEARCH_STEP_DEGREES
        )

        total_search_positions = 1 + (2 * max_side_steps)
        search_position = 0

        def check_current_heading() -> Optional[dict]:
            nonlocal search_position

            search_position += 1
            consecutive_detections = 0
            latest_detection = None

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
                    latest_detection = None
                    self.current_bin_target = None

                    self.last_action = (
                        f"SEARCH_BIN COLOR={self.locked_bin_color} "
                        f"POSITION={search_position}/"
                        f"{total_search_positions} "
                        "NOT_VISIBLE"
                    )
                else:
                    consecutive_detections += 1
                    latest_detection = detection
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

            return latest_detection if (
                consecutive_detections
                >= self.REQUIRED_BIN_DETECTIONS
            ) else None

        def turn_and_record(
            command: str,
            angle: float,
            action: str,
        ) -> None:
            self.last_action = action

            if command == "TURN_RIGHT":
                actual_angle = self.arduino.turn_right(angle)
            elif command == "TURN_LEFT":
                actual_angle = self.arduino.turn_left(angle)
            else:
                raise ValueError(
                    f"Unsupported bin-search turn: {command}"
                )

            self.history.record_turn(
                command=command,
                angle=actual_angle,
            )

            time.sleep(
                self.BIN_SCAN_SETTLE_SECONDS
            )

        # Check straight ahead first.
        detection = check_current_heading()
        if detection is not None:
            return detection

        # Scan right in small increments to +90 degrees.
        for step in range(1, max_side_steps + 1):
            if self.stop_event.is_set():
                raise RuntimeError(
                    "Bin search was stopped."
                )

            turn_and_record(
                "TURN_RIGHT",
                self.BIN_SEARCH_STEP_DEGREES,
                (
                    "SEARCH_BIN_RIGHT "
                    f"OFFSET={step * self.BIN_SEARCH_STEP_DEGREES:.0f}"
                ),
            )

            # Check straight ahead first.
            detection = check_current_heading()
            if detection is not None:
                return detection

            # Scan LEFT first in small increments to -90 degrees.
            for step in range(1, max_side_steps + 1):
                if self.stop_event.is_set():
                    raise RuntimeError(
                        "Bin search was stopped."
                    )

                turn_and_record(
                    "TURN_LEFT",
                    self.BIN_SEARCH_STEP_DEGREES,
                    (
                        "SEARCH_BIN_LEFT "
                        f"OFFSET={step * self.BIN_SEARCH_STEP_DEGREES:.0f}"
                    ),
                )

                detection = check_current_heading()
                if detection is not None:
                    return detection

            # Nothing on the left. Return to center.
            turn_and_record(
                "TURN_RIGHT",
                self.BIN_SEARCH_MAX_OFFSET_DEGREES,
                "SEARCH_BIN_RETURN_CENTER_FROM_LEFT",
            )

            # Scan RIGHT in small increments to +90 degrees.
            for step in range(1, max_side_steps + 1):
                if self.stop_event.is_set():
                    raise RuntimeError(
                        "Bin search was stopped."
                    )

                turn_and_record(
                    "TURN_RIGHT",
                    self.BIN_SEARCH_STEP_DEGREES,
                    (
                        "SEARCH_BIN_RIGHT "
                        f"OFFSET={step * self.BIN_SEARCH_STEP_DEGREES:.0f}"
                    ),
                )

                detection = check_current_heading()
                if detection is not None:
                    return detection

            # Search failed. Return to original center heading.
            turn_and_record(
                "TURN_LEFT",
                self.BIN_SEARCH_MAX_OFFSET_DEGREES,
                "SEARCH_BIN_RETURN_CENTER_FROM_RIGHT",
            )

        #     detection = check_current_heading()
        #     if detection is not None:
        #         return detection

        # # Nothing on the right. Return exactly to the original
        # # bin-facing heading before scanning the left side.
        # turn_and_record(
        #     "TURN_LEFT",
        #     self.BIN_SEARCH_MAX_OFFSET_DEGREES,
        #     "SEARCH_BIN_RETURN_CENTER_FROM_RIGHT",
        # )

        # # Scan left in small increments to -90 degrees.
        # for step in range(1, max_side_steps + 1):
        #     if self.stop_event.is_set():
        #         raise RuntimeError(
        #             "Bin search was stopped."
        #         )

        #     turn_and_record(
        #         "TURN_LEFT",
        #         self.BIN_SEARCH_STEP_DEGREES,
        #         (
        #             "SEARCH_BIN_LEFT "
        #             f"OFFSET={step * self.BIN_SEARCH_STEP_DEGREES:.0f}"
        #         ),
        #     )

        #     detection = check_current_heading()
        #     if detection is not None:
        #         return detection

        # # Search failed. Restore the original heading before raising
        # # the error so the robot never remains parked at -90 degrees.
        # turn_and_record(
        #     "TURN_RIGHT",
        #     self.BIN_SEARCH_MAX_OFFSET_DEGREES,
        #     "SEARCH_BIN_RETURN_CENTER_FROM_LEFT",
        # )

        self.current_bin_target = None

        raise RuntimeError(
            "The locked bin was not found after a full "
            "180-degree scan: "
            f"{self.locked_bin_color}"
        )

    def _recover_locked_bin_alignment(
        self,
        last_horizontal_error: Optional[int],
    ) -> Optional[dict]:
        """
        Reacquire a bin that disappeared during fine alignment.

        Start searching in the direction the previous camera error
        suggested, sweep up to 20 degrees on that side, then sweep
        through the original heading to 20 degrees on the opposite
        side. If still not found, return to the recovery-start heading.

        Every turn is small and every step checks several camera frames.
        """
        if last_horizontal_error is None:
            preferred_command = "TURN_LEFT"
        elif last_horizontal_error < 0:
            # Bin was left in image -> robot correction is right.
            preferred_command = "TURN_RIGHT"
        else:
            # Bin was right in image -> robot correction is left.
            preferred_command = "TURN_LEFT"

        opposite_command = (
            "TURN_LEFT"
            if preferred_command == "TURN_RIGHT"
            else "TURN_RIGHT"
        )

        side_steps = int(
            self.BIN_ALIGNMENT_RECOVERY_MAX_OFFSET_DEGREES
            / self.BIN_ALIGNMENT_RECOVERY_STEP_DEGREES
        )

        # From the recovery-start heading:
        #   preferred side to +20
        #   sweep through center to -20
        #   return to center if nothing is found
        recovery_commands = (
            [preferred_command] * side_steps
            + [opposite_command] * (side_steps * 2)
            + [preferred_command] * side_steps
        )

        for step_index, command in enumerate(
            recovery_commands,
            start=1,
        ):
            if self.stop_event.is_set():
                raise RuntimeError(
                    "Bin alignment was stopped."
                )

            self.last_action = (
                "RECOVER_BIN_ALIGNMENT "
                f"STEP={step_index}/{len(recovery_commands)} "
                f"{command} "
                f"{self.BIN_ALIGNMENT_RECOVERY_STEP_DEGREES:.1f}"
            )

            if command == "TURN_LEFT":
                actual_angle = self.arduino.turn_left(
                    self.BIN_ALIGNMENT_RECOVERY_STEP_DEGREES
                )
            else:
                actual_angle = self.arduino.turn_right(
                    self.BIN_ALIGNMENT_RECOVERY_STEP_DEGREES
                )

            self.history.record_turn(
                command=command,
                angle=actual_angle,
            )

            time.sleep(
                self.BIN_ALIGNMENT_DELAY_SECONDS
            )

            for _ in range(
                self.BIN_ALIGNMENT_RECOVERY_CHECKS_PER_STEP
            ):
                if self.stop_event.is_set():
                    raise RuntimeError(
                        "Bin alignment was stopped."
                    )

                detection = (
                    self._get_locked_bin_detection()
                )

                if detection is not None:
                    print(
                        "Bin alignment recovered: "
                        f"center_x={detection['center_x']}, "
                        f"area={detection['area']}"
                    )
                    return detection

                time.sleep(
                    self.BIN_ALIGNMENT_DELAY_SECONDS
                )

        return None

    def _align_with_locked_bin(self) -> dict:
        """
        Center the locked-color bin with conservative corrections.

        Camera orientation in this project:
            image left  -> turn robot right
            image right -> turn robot left

        If the bin disappears, use a local +/-20-degree recovery sweep
        instead of immediately failing after a very small 6/12/6 scan.
        """
        if not self.locked_bin_color:
            raise RuntimeError(
                "Cannot align with a bin because no bin color is locked."
            )

        self.status = "ALIGNING_WITH_BIN"

        centered_updates = 0
        lost_updates = 0
        last_horizontal_error = None

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

                detection = (
                    self._recover_locked_bin_alignment(
                        last_horizontal_error
                    )
                )

                if detection is None:
                    self.current_bin_target = None

                    raise RuntimeError(
                        "The locked bin was lost during alignment "
                        "and could not be recovered within +/-"
                        f"{self.BIN_ALIGNMENT_RECOVERY_MAX_OFFSET_DEGREES:.0f} "
                        "degrees."
                    )

                self.current_bin_target = detection
                lost_updates = 0
                centered_updates = 0

                self.last_action = (
                    "BIN_ALIGNMENT_RECOVERED "
                    f"CENTER_X={detection['center_x']}"
                )

            else:
                self.current_bin_target = detection
                lost_updates = 0

            horizontal_error = (
                detection["center_x"]
                - self.FRAME_CENTER_X
            )

            last_horizontal_error = horizontal_error

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

            # Be more conservative than the old 3/2/1-degree logic.
            # A bin near the edge gets at most 2 degrees per update;
            # once reasonably close to center, use only 1 degree.
            if is_close_range:
                turn_degrees = 1
            elif absolute_error > 180:
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
        Walk toward the already-locked bin using short, camera-confirmed
        forward pulses.

        Important rules:
        - Never drive while the locked-color bin is outside the steering
          tolerance.
        - Re-center with only 1-degree turns.
        - Re-check the camera after every forward pulse.
        - Never release based on ultrasonic distance alone. The locked
          color must also be centered and visually close.
        - If the bin disappears, stop first and try the same local recovery
          sweep used by fine alignment.
        """
        self.status = "APPROACHING_BIN"

        lost_updates = 0
        close_unconfirmed_updates = 0
        last_horizontal_error = None

        self._stop_continuous_forward()

        for _ in range(self.MAX_BIN_APPROACH_UPDATES):
            if self.stop_event.is_set():
                self._stop_continuous_forward()
                raise RuntimeError(
                    "Bin approach was stopped."
                )

            detection = self._get_locked_bin_detection()

            if detection is None:
                self._stop_continuous_forward()
                lost_updates += 1

                self.last_action = (
                    "BIN_LOST_DURING_APPROACH "
                    f"{lost_updates}/{self.MAX_BIN_LOST_UPDATES}"
                )

                if lost_updates < self.MAX_BIN_LOST_UPDATES:
                    time.sleep(
                        self.BIN_APPROACH_DELAY_SECONDS
                    )
                    continue

                self.last_action = (
                    "RECOVERING_BIN_DURING_APPROACH"
                )

                detection = self._recover_locked_bin_alignment(
                    last_horizontal_error
                )

                if detection is None:
                    self.current_bin_target = None
                    raise RuntimeError(
                        "The locked bin was lost during approach "
                        "and could not be recovered."
                    )

                self.current_bin_target = detection
                lost_updates = 0
            else:
                lost_updates = 0
                self.current_bin_target = detection

            horizontal_error = (
                detection["center_x"]
                - self.FRAME_CENTER_X
            )
            last_horizontal_error = horizontal_error

            if (
                abs(horizontal_error)
                > self.BIN_APPROACH_ALIGNMENT_TOLERANCE_PX
            ):
                self._stop_continuous_forward()
                close_unconfirmed_updates = 0

                if horizontal_error < 0:
                    self.last_action = (
                        "BIN_APPROACH_REALIGN TURN_RIGHT "
                        f"{self.BIN_APPROACH_REALIGN_DEGREES} "
                        f"ERROR_PX={horizontal_error}"
                    )

                    actual_angle = self.arduino.turn_right(
                        self.BIN_APPROACH_REALIGN_DEGREES
                    )

                    self.history.record_turn(
                        command="TURN_RIGHT",
                        angle=actual_angle,
                    )
                else:
                    self.last_action = (
                        "BIN_APPROACH_REALIGN TURN_LEFT "
                        f"{self.BIN_APPROACH_REALIGN_DEGREES} "
                        f"ERROR_PX={horizontal_error}"
                    )

                    actual_angle = self.arduino.turn_left(
                        self.BIN_APPROACH_REALIGN_DEGREES
                    )

                    self.history.record_turn(
                        command="TURN_LEFT",
                        angle=actual_angle,
                    )

                time.sleep(
                    self.BIN_APPROACH_DELAY_SECONDS
                )
                continue

            distance_cm = self.arduino.read_distance_cm()

            if self._distance_is_unreliable(distance_cm):
                self._stop_continuous_forward()
                self.last_ultrasonic_distance_cm = distance_cm
                self.last_action = (
                    "BIN_DISTANCE_UNRELIABLE"
                )

                time.sleep(
                    self.BIN_APPROACH_DELAY_SECONDS
                )
                continue

            self.last_ultrasonic_distance_cm = distance_cm

            if distance_cm <= self.BIN_EMERGENCY_DISTANCE_CM:
                self._stop_continuous_forward()
                raise RuntimeError(
                    "Emergency stop during bin approach: "
                    f"{distance_cm:.1f} cm."
                )

            visually_centered_for_release = (
                abs(horizontal_error)
                <= self.BIN_RELEASE_VISUAL_CENTER_TOLERANCE_PX
            )
            visually_close_to_locked_bin = (
                detection["area"]
                >= self.BIN_CLOSE_RANGE_AREA_PX
            )

            if distance_cm <= self.BIN_RELEASE_DISTANCE_CM:
                self._stop_continuous_forward()

                if (
                    visually_centered_for_release
                    and visually_close_to_locked_bin
                ):
                    self.status = (
                        "BIN_RELEASE_POSITION_REACHED"
                    )
                    self.last_action = (
                        "BIN_RELEASE_POSITION_REACHED "
                        f"DISTANCE_CM={distance_cm:.1f} "
                        f"ERROR_PX={horizontal_error} "
                        f"AREA={detection['area']}"
                    )

                    print(
                        "Bin release position reached: "
                        f"color={self.locked_bin_color}, "
                        f"distance={distance_cm:.1f} cm, "
                        f"center_x={detection['center_x']}, "
                        f"area={detection['area']}"
                    )

                    return distance_cm

                close_unconfirmed_updates += 1
                self.status = (
                    "BIN_CLOSE_BUT_NOT_VISUALLY_CONFIRMED"
                )
                self.last_action = (
                    "HOLD_FOR_LOCKED_BIN_CONFIRMATION "
                    f"{close_unconfirmed_updates}/"
                    f"{self.MAX_BIN_CLOSE_UNCONFIRMED_UPDATES} "
                    f"DISTANCE_CM={distance_cm:.1f} "
                    f"ERROR_PX={horizontal_error} "
                    f"AREA={detection['area']}"
                )

                if (
                    close_unconfirmed_updates
                    >= self.MAX_BIN_CLOSE_UNCONFIRMED_UPDATES
                ):
                    raise RuntimeError(
                        "Ultrasonic sees a close obstacle, but the "
                        f"locked {self.locked_bin_color} bin is not "
                        "visually confirmed at the release position."
                    )

                time.sleep(
                    self.BIN_APPROACH_DELAY_SECONDS
                )
                continue

            close_unconfirmed_updates = 0

            self._stop_continuous_forward()

            pulse_ms = self._bin_approach_duration(
                distance_cm
            )

            self.status = "APPROACHING_BIN_CONTROLLED"
            self.last_action = (
                "APPROACH_BIN_PULSE "
                f"{pulse_ms} "
                f"DISTANCE_CM={distance_cm:.1f} "
                f"ERROR_PX={horizontal_error} "
                f"COLOR={self.locked_bin_color}"
            )

            actual_duration = self.arduino.forward(
                pulse_ms
            )

            self.history.record_linear(
                command="FORWARD",
                duration_ms=actual_duration,
                source="BIN_APPROACH",
            )

            time.sleep(
                self.BIN_APPROACH_DELAY_SECONDS
            )

        self._stop_continuous_forward()

        raise RuntimeError(
            "The robot did not reach the locked bin release position."
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
        duration_scale: float,
    ) -> None:
        if distance_cm <= 0:
            return

        estimated_ms = (
            distance_cm / self.CM_PER_MS
        )

        remaining_ms = int(
            estimated_ms
            * duration_scale
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