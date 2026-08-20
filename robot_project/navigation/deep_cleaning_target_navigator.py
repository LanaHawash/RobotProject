import threading

from robot_project.navigation.target_navigator import TargetNavigator


class DeepCleaningTargetNavigator(TargetNavigator):
    """
    TargetNavigator extension used by the zig-zag cleaning mission.

    Standalone target navigation keeps the original TargetNavigator
    behavior. During deep cleaning, a confirmed object temporarily
    owns the robot, uses the lane interruption point as a local origin,
    delivers the object to its locked bin, returns to that origin, and
    restores the original lane heading before the cleaning navigator
    resumes.
    """

    MODE_STANDALONE = "STANDALONE"
    MODE_DEEP_CLEANING = "DEEP_CLEANING"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.navigation_mode = self.MODE_STANDALONE
        self.deep_cleaning_lane_number = None
        self.deep_cleaning_bins_direction = None
        self.deep_cleaning_sort_result = None

        # Prevent two cleaning interruptions from starting at once.
        self.deep_cleaning_sort_lock = threading.Lock()

    def start(self, selected_target: dict) -> None:
        """Start the original standalone sorting behavior."""
        self.navigation_mode = self.MODE_STANDALONE
        self.deep_cleaning_lane_number = None
        self.deep_cleaning_bins_direction = None
        self.deep_cleaning_sort_result = None

        super().start(selected_target)

    def sort_for_deep_cleaning(
        self,
        selected_target: dict,
        lane_number: int,
    ) -> dict:
        """
        Run exactly one object-sorting interruption for zig-zag mode.

        The current robot pose becomes local (0, 0), heading 0. After
        the object is deposited, MovementHistory is used to return to
        that interruption point and restore heading 0 so the same lane
        can continue.
        """
        try:
            lane_number = int(lane_number)
        except (TypeError, ValueError) as error:
            raise ValueError(
                "lane_number must be a positive integer."
            ) from error

        if lane_number < 1:
            raise ValueError(
                "lane_number must be a positive integer."
            )

        with self.deep_cleaning_sort_lock:
            if self.is_running():
                raise RuntimeError(
                    "Target navigation is already running."
                )

            self.navigation_mode = self.MODE_DEEP_CLEANING
            self.deep_cleaning_lane_number = lane_number
            self.deep_cleaning_bins_direction = (
                "BEHIND"
                if lane_number % 2 == 1
                else "AHEAD"
            )
            self.deep_cleaning_sort_result = None

            # Base start() clears MovementHistory. In this mode that is
            # intentional: the lane interruption point becomes the local
            # return origin for this one sorting trip.
            super().start(selected_target)

            navigation_thread = self.navigation_thread

            if navigation_thread is None:
                raise RuntimeError(
                    "Deep-cleaning sorting thread did not start."
                )

            navigation_thread.join()

            if self.stop_event.is_set():
                raise RuntimeError(
                    "Deep-cleaning object sorting was stopped."
                )

            if self.error is not None:
                raise RuntimeError(self.error)

            if self.status != "DEEP_CLEANING_SORT_COMPLETE":
                raise RuntimeError(
                    "Deep-cleaning object sorting ended with "
                    f"status {self.status}."
                )

            return self.deep_cleaning_sort_result.copy()

    def _run_pickup_handoff(
        self,
        distance_cm: float,
    ) -> bool:
        if self.navigation_mode != self.MODE_DEEP_CLEANING:
            return super()._run_pickup_handoff(distance_cm)

        self._stop_continuous_forward()

        self.status = "POSITIONING_FOR_PICKUP"
        self.last_action = (
            "POSITION_FOR_PICKUP "
            f"DISTANCE_CM={distance_cm:.1f}"
        )

        print(
            "Starting deep-cleaning pickup positioning at "
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

            self.centered_updates = 0

            return True

        self._complete_pickup_positioning(result)
        self._grab_object()

        if self.stop_event.is_set():
            return False

        self._deliver_object_and_return_to_lane()

        # False tells the inherited navigation loop that this one-shot
        # interruption is complete. DeepCleaningNavigator will resume.
        return False

    def _deliver_object_and_return_to_lane(self) -> None:
        lane_number = self.deep_cleaning_lane_number

        if lane_number is None:
            raise RuntimeError(
                "Deep-cleaning lane number is not available."
            )

        bins_direction = (
            "BEHIND"
            if lane_number % 2 == 1
            else "AHEAD"
        )
        self.deep_cleaning_bins_direction = bins_direction

        # MovementHistory heading 0 is the lane heading at the exact
        # interruption point. Therefore bins are heading 0 on even
        # lanes and heading 180 on odd lanes.
        pose = self.history.estimate_pose(
            self.CM_PER_MS
        )
        self.last_pose = pose

        desired_bins_heading = (
            180.0
            if bins_direction == "BEHIND"
            else 0.0
        )

        turn_to_bins = self._normalize_angle(
            desired_bins_heading
            - pose["heading_degrees"]
        )

        self.status = "FACING_BINS_FROM_LANE"
        self.last_action = (
            "FACE_BINS_FROM_LANE "
            f"LANE={lane_number} "
            f"DIRECTION={bins_direction} "
            f"TURN_DEGREES={turn_to_bins:.1f}"
        )

        self._execute_turn(turn_to_bins)

        if self.stop_event.is_set():
            return

        self.status = "READY_TO_SORT_FROM_LANE"
        self.last_action = (
            "READY_TO_SORT_FROM_LANE "
            f"LANE={lane_number} "
            f"DIRECTION={bins_direction}"
        )

        # Reuse the existing color-bin sequence unchanged.
        self._find_locked_bin()
        self._align_with_locked_bin()
        self._approach_locked_bin()
        self._release_object()

        if self.stop_event.is_set():
            return

        # Make enough room to turn away from the bin.
        actual_duration = self.arduino.backward(300)

        self.history.record_linear(
            command="BACKWARD",
            duration_ms=actual_duration,
            source="BIN_EXIT",
        )

        # Because history started at the lane interruption, this returns
        # there rather than to the global room start.
        self.status = "RETURNING_TO_INTERRUPTED_LANE"
        self._return_to_start()

        if self.stop_event.is_set():
            return

        self._restore_interrupted_lane_heading()

        if self.stop_event.is_set():
            return

        self.status = "DEEP_CLEANING_SORT_COMPLETE"
        self.last_action = (
            "RETURNED_TO_INTERRUPTED_LANE "
            f"LANE={lane_number}"
        )

        self.deep_cleaning_sort_result = {
            "success": True,
            "lane_number": lane_number,
            "bins_direction": bins_direction,
            "object_class": self.locked_object_class,
            "destination": self.locked_destination,
            "bin_color": self.locked_bin_color,
        }

        print(
            "Deep-cleaning sorting interruption complete. "
            f"Lane {lane_number} can resume."
        )

    def _restore_interrupted_lane_heading(self) -> None:
        """Restore heading 0 of the local interruption coordinate frame."""
        pose = self.history.estimate_pose(
            self.CM_PER_MS
        )
        self.last_pose = pose

        turn_to_lane = self._normalize_angle(
            -pose["heading_degrees"]
        )

        self.status = "RESTORING_INTERRUPTED_LANE_HEADING"
        self.last_action = (
            "RESTORE_LANE_HEADING "
            f"TURN_DEGREES={turn_to_lane:.1f}"
        )

        self._execute_turn(turn_to_lane)