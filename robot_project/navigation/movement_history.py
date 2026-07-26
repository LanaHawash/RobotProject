import math
import threading
from typing import Iterable


class MovementHistory:
    def __init__(self):
        self._movements = []
        self._lock = threading.Lock()

    def clear(self) -> None:
        with self._lock:
            self._movements.clear()

    def record_linear(
        self,
        command: str,
        duration_ms: int,
        source: str = "NAVIGATION",
    ) -> None:
        if command not in {
            "FORWARD",
            "BACKWARD",
        }:
            raise ValueError(
                "Linear command must be "
                "FORWARD or BACKWARD."
            )

        movement = {
            "command": command,
            "duration_ms": int(duration_ms),
            "source": source,
        }

        with self._lock:
            self._movements.append(movement)

    def record_turn(
        self,
        command: str,
        angle: float,
    ) -> None:
        if command not in {
            "TURN_LEFT",
            "TURN_RIGHT",
        }:
            raise ValueError(
                "Turn command must be "
                "TURN_LEFT or TURN_RIGHT."
            )

        movement = {
            "command": command,
            "angle": round(float(angle), 2),
            "source": "NAVIGATION",
        }

        with self._lock:
            self._movements.append(movement)

    def extend_pulses(
        self,
        pulses: Iterable[dict],
    ) -> None:
        for pulse in pulses:
            self.record_linear(
                command=pulse["command"],
                duration_ms=pulse["duration_ms"],
                source=pulse.get(
                    "source",
                    "ULTRASONIC",
                ),
            )

    def snapshot(self) -> list[dict]:
        with self._lock:
            return [
                movement.copy()
                for movement in self._movements
            ]

    def simplified(self) -> list[dict]:
        """
        Combine each consecutive FORWARD/BACKWARD run into its
        net movement. Turns remain as boundaries.

        Example:
            FORWARD 80
            BACKWARD 60
            FORWARD 40

        becomes:
            FORWARD 60
        """
        original = self.snapshot()
        simplified = []

        linear_total = 0
        linear_sources = set()

        def flush_linear() -> None:
            nonlocal linear_total
            nonlocal linear_sources

            if linear_total == 0:
                linear_sources = set()
                return

            if linear_total > 0:
                command = "FORWARD"
                duration_ms = linear_total
            else:
                command = "BACKWARD"
                duration_ms = -linear_total

            simplified.append(
                {
                    "command": command,
                    "duration_ms": duration_ms,
                    "source": (
                        "ULTRASONIC_NET"
                        if linear_sources
                        == {"ULTRASONIC"}
                        else "NET"
                    ),
                }
            )

            linear_total = 0
            linear_sources = set()

        for movement in original:
            command = movement["command"]

            if command == "FORWARD":
                linear_total += (
                    movement["duration_ms"]
                )
                linear_sources.add(
                    movement.get(
                        "source",
                        "NAVIGATION",
                    )
                )
                continue

            if command == "BACKWARD":
                linear_total -= (
                    movement["duration_ms"]
                )
                linear_sources.add(
                    movement.get(
                        "source",
                        "NAVIGATION",
                    )
                )
                continue

            flush_linear()
            simplified.append(
                movement.copy()
            )

        flush_linear()

        return simplified

    def inverse_route(self) -> list[dict]:
        return_route = []

        for movement in reversed(
            self.simplified()
        ):
            command = movement["command"]

            if command == "FORWARD":
                return_route.append(
                    {
                        "command": "BACKWARD",
                        "duration_ms":
                            movement["duration_ms"],
                    }
                )

            elif command == "BACKWARD":
                return_route.append(
                    {
                        "command": "FORWARD",
                        "duration_ms":
                            movement["duration_ms"],
                    }
                )

            elif command == "TURN_LEFT":
                return_route.append(
                    {
                        "command": "TURN_RIGHT",
                        "angle": movement["angle"],
                    }
                )

            elif command == "TURN_RIGHT":
                return_route.append(
                    {
                        "command": "TURN_LEFT",
                        "angle": movement["angle"],
                    }
                )

        return return_route

    def estimate_pose(
        self,
        cm_per_ms: float,
    ) -> dict:
        """
        Dead-reckoning pose estimate relative to the starting
        point, built from the same simplified() history used
        elsewhere. This does NOT replay or reverse any commands -
        it only integrates them into a position/heading estimate.

        Heading comes from the gyro-reported turn angles recorded
        by record_turn() (accurate, closed-loop on the Arduino
        side). Distance comes from duration_ms * cm_per_ms (an
        open-loop estimate, since there are no wheel encoders).

        Coordinate frame:
            - Start pose is (0, 0), heading 0.
            - heading is in degrees, increasing with TURN_RIGHT
              and decreasing with TURN_LEFT.
            - "forward" at heading 0 is +y; heading rotates
              toward +x as it increases (matches TURN_RIGHT).

        Returns:
            {"x": float, "y": float, "heading_degrees": float}
            x/y are in the same distance unit as cm_per_ms
            produces (centimeters, if cm_per_ms is cm/ms).
        """
        x = 0.0
        y = 0.0
        heading = 0.0

        for movement in self.simplified():
            command = movement["command"]

            if command in ("FORWARD", "BACKWARD"):
                distance = (
                    movement["duration_ms"] * cm_per_ms
                )

                if command == "BACKWARD":
                    distance = -distance

                x += distance * math.sin(
                    math.radians(heading)
                )
                y += distance * math.cos(
                    math.radians(heading)
                )

            elif command == "TURN_RIGHT":
                heading += movement["angle"]

            elif command == "TURN_LEFT":
                heading -= movement["angle"]

        return {
            "x": x,
            "y": y,
            "heading_degrees": heading,
        }