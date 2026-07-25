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