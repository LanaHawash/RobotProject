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

        self.deep_cleaning_return_route = []
        self.deep_cleaning_return_steps_completed = 0
        self.deep_cleaning_return_complete = False

        # Prevent two cleaning interruptions from starting at once.
        self.deep_cleaning_sort_lock = threading.Lock()

    def start(self, selected_target: dict) -> None:
        """Start the original standalone sorting behavior."""
        self.navigation_mode = self.MODE_STANDALONE
        self.deep_cleaning_lane_number = None
        self.deep_cleaning_bins_direction = None
        self.deep_cleaning_sort_result = None
        self.deep_cleaning_return_route = []
        self.deep_cleaning_return_steps_completed = 0
        self.deep_cleaning_return_complete = False

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
            self.deep_cleaning_return_route = []
            self.deep_cleaning_return_steps_completed = 0
            self.deep_cleaning_return_complete = False

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

        bins_direction = self.deep_cleaning_bins_direction

        if bins_direction not in {
            "AHEAD",
            "BEHIND",
        }:
            raise RuntimeError(
                "Deep-cleaning bins direction is not available."
            )

        lane_travel_direction = (
            "AWAY_FROM_BINS"
            if lane_number % 2 == 1
            else "TOWARD_BINS"
        )

        # Heading 0 is always the heading of the interrupted lane.
        #
        # We intentionally calculate only heading from the real
        # gyro-recorded turns. We do NOT use TargetNavigator's
        # return-home position estimation here.
        current_heading = (
            self._current_recorded_heading_degrees()
        )


        desired_bins_heading = (
            180.0
            if bins_direction == "BEHIND"
            else 0.0
        )

        turn_to_bins = self._normalize_angle(
            desired_bins_heading
            - current_heading
        )

        self.status = "FACING_BINS_FROM_LANE"
        self.last_action = (
            "FACE_BINS_FROM_LANE "
            f"LANE={lane_number} "
            f"DIRECTION={bins_direction} "
            f"TURN_DEGREES={turn_to_bins:.1f}"
        )

        self._execute_turn(
            turn_to_bins
        )

        if self.stop_event.is_set():
            return

        self.status = "READY_TO_SORT_FROM_LANE"
        self.last_action = (
            "READY_TO_SORT_FROM_LANE "
            f"LANE={lane_number} "
            f"DIRECTION={bins_direction}"
        )

        # Existing bin behavior remains unchanged.
        self._find_locked_bin()
        self._align_with_locked_bin()
        self._approach_locked_bin()
        self._release_object()

        if self.stop_event.is_set():
            return

        # IMPORTANT:
        # Do NOT call TargetNavigator._return_to_start().
        #
        # Do NOT make an extra BACKWARD 300 movement.
        #
        # Reverse the actual movements that were recorded
        # since the lane interruption point.
        return_steps = (
            self._return_to_interrupted_lane_by_exact_replay()
        )

        if self.stop_event.is_set():
            return

        self.status = "DEEP_CLEANING_SORT_COMPLETE"
        self.last_action = (
            "RETURNED_TO_INTERRUPTED_LANE "
            f"LANE={lane_number} "
            f"RETURN_STEPS={return_steps}"
        )

        self.deep_cleaning_sort_result = {
            "success": True,
            "lane_number": lane_number,
            "bins_direction": bins_direction,
            "lane_travel_direction": (
                lane_travel_direction
            ),
            "object_class": (
                self.locked_object_class
            ),
            "destination": (
                self.locked_destination
            ),
            "bin_color": (
                self.locked_bin_color
            ),
            "returned_to_lane": True,
            "lane_heading_restored": True,
            "return_policy": (
                "EXACT_REVERSE_REPLAY"
            ),
            "return_route_steps": (
                return_steps
            ),
        }

        print(
            "Deep-cleaning sorting interruption complete. "
            f"Returned to lane {lane_number} "
            "with heading restored. "
            f"Travel direction: "
            f"{lane_travel_direction}."
        )


    def _current_recorded_heading_degrees(
        self,
    ) -> float:
        """
        Return the robot's current heading relative to the
        interrupted lane heading.

        The interrupted lane heading is always local heading 0.

        Only gyro-measured turns affect heading. Linear movement
        does not matter here.
        """
        heading_degrees = 0.0

        for movement in self.history.snapshot():
            command = movement.get(
                "command"
            )

            if command == "TURN_RIGHT":
                heading_degrees += float(
                    movement["angle"]
                )

            elif command == "TURN_LEFT":
                heading_degrees -= float(
                    movement["angle"]
                )

        return self._normalize_angle(
            heading_degrees
        )    

    def _build_exact_reverse_route(
        self,
    ) -> list[dict]:
        """
        Build the literal inverse of every recorded movement.

        Unlike MovementHistory.inverse_route(), this uses the
        raw snapshot and does not simplify or combine movements.
        """
        reverse_route = []

        movements = (
            self.history.snapshot()
        )

        for movement in reversed(
            movements
        ):
            command = movement.get(
                "command"
            )

            if command in {
                "FORWARD",
                "BACKWARD",
            }:
                duration_ms = int(
                    movement["duration_ms"]
                )

                if duration_ms <= 0:
                    raise RuntimeError(
                        "Invalid recorded movement duration "
                        "during deep-cleaning return: "
                        f"{duration_ms} ms."
                    )

                reverse_command = (
                    "BACKWARD"
                    if command == "FORWARD"
                    else "FORWARD"
                )

                remaining_ms = duration_ms

                while remaining_ms > 0:
                    chunk_ms = min(
                        remaining_ms,
                        5000,
                    )

                    reverse_route.append(
                        {
                            "command": reverse_command,
                            "duration_ms": chunk_ms,
                            "source": movement.get(
                                "source",
                                "NAVIGATION",
                            ),
                        }
                    )

                    remaining_ms -= chunk_ms

                continue

            if command in {
                "TURN_LEFT",
                "TURN_RIGHT",
            }:
                angle = float(
                    movement["angle"]
                )

                if angle <= 0.0:
                    raise RuntimeError(
                        "Invalid recorded turn angle "
                        "during deep-cleaning return: "
                        f"{angle} degrees."
                    )

                reverse_command = (
                    "TURN_RIGHT"
                    if command == "TURN_LEFT"
                    else "TURN_LEFT"
                )

                remaining_angle = angle

                while (
                    remaining_angle
                    >= self.MIN_EXECUTABLE_TURN_DEGREES
                ):
                    chunk_angle = min(
                        remaining_angle,
                        180.0,
                    )

                    reverse_route.append(
                        {
                            "command": reverse_command,
                            "angle": chunk_angle,
                            "source": movement.get(
                                "source",
                                "NAVIGATION",
                            ),
                        }
                    )

                    remaining_angle -= chunk_angle

                if remaining_angle > 0.0:
                    print(
                        "Deep-cleaning reverse route: "
                        "ignoring residual non-executable "
                        f"turn of {remaining_angle:.2f} degrees."
                    )

                continue

              

            raise RuntimeError(
                "Unsupported movement in "
                "deep-cleaning history: "
                f"{movement}"
            )

        return reverse_route



    def _return_to_interrupted_lane_by_exact_replay(
        self,
    ) -> int:
        """
        Physically retrace the complete deep-cleaning sorting
        route back to the lane interruption point.

        Replay movements are intentionally NOT written back into
        MovementHistory. The history remains the outbound route
        until the complete return succeeds.
        """
        lane_number = (
            self.deep_cleaning_lane_number
        )

        if lane_number is None:
            raise RuntimeError(
                "Cannot return because the interrupted "
                "lane number is missing."
            )

        reverse_route = (
            self._build_exact_reverse_route()
        )

        if not reverse_route:
            raise RuntimeError(
                "Cannot return to the interrupted lane "
                "because movement history is empty."
            )

        self.deep_cleaning_return_route = [
            movement.copy()
            for movement in reverse_route
        ]

        self.deep_cleaning_return_steps_completed = 0
        self.deep_cleaning_return_complete = False

        print(
            "Deep-cleaning exact reverse route: "
            f"lane={lane_number}, "
            f"steps={len(reverse_route)}"
        )

        for index, movement in enumerate(
            reverse_route,
            start=1,
        ):
            if self.stop_event.is_set():
                self.arduino.stop()

                return (
                    self
                    .deep_cleaning_return_steps_completed
                )

            command = movement[
                "command"
            ]

            self.status = (
                "RETURNING_TO_INTERRUPTED_LANE_"
                f"{lane_number}"
            )

            if command in {
                "FORWARD",
                "BACKWARD",
            }:
                duration_ms = movement[
                    "duration_ms"
                ]

                self.last_action = (
                    "REVERSE_ROUTE_STEP "
                    f"{index}/"
                    f"{len(reverse_route)} "
                    f"LANE={lane_number} "
                    f"{command} "
                    f"{duration_ms}"
                )

                print(
                    "Deep-cleaning return: "
                    f"lane={lane_number}, "
                    f"step={index}/"
                    f"{len(reverse_route)}, "
                    f"{command} "
                    f"{duration_ms} ms"
                )

                if command == "FORWARD":
                    self.arduino.forward(
                        duration_ms
                    )

                else:
                    self.arduino.backward(
                        duration_ms
                    )

            elif command in {
                "TURN_LEFT",
                "TURN_RIGHT",
            }:
                angle = movement[
                    "angle"
                ]

                self.last_action = (
                    "REVERSE_ROUTE_STEP "
                    f"{index}/"
                    f"{len(reverse_route)} "
                    f"LANE={lane_number} "
                    f"{command} "
                    f"{angle:.2f}"
                )

                print(
                    "Deep-cleaning return: "
                    f"lane={lane_number}, "
                    f"step={index}/"
                    f"{len(reverse_route)}, "
                    f"{command} "
                    f"{angle:.2f} degrees"
                )

                if command == "TURN_LEFT":
                    self.arduino.turn_left(
                        angle
                    )

                else:
                    self.arduino.turn_right(
                        angle
                    )

            else:
                raise RuntimeError(
                    "Unsupported reverse-route "
                    f"command: {command}"
                )

            self.deep_cleaning_return_steps_completed = (
                index
            )

        self.arduino.stop()

        self.deep_cleaning_return_complete = True

        # IMPORTANT:
        # Clear only after every return command succeeds.
        #
        # If a return command fails halfway through, preserving
        # the original history makes the failure diagnosable.
        self.history.clear()

        # Do not invent a dead-reckoning pose. The successful
        # reverse replay defines the logical return to the
        # interruption origin.
        self.last_pose = None

        print(
            "Deep-cleaning exact reverse complete: "
            f"lane={lane_number}, "
            f"steps={len(reverse_route)}"
        )

        return len(
            reverse_route
        )

   