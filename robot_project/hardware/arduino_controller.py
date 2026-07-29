import re
import threading
import time
from typing import Optional

import serial


class ArduinoController:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud_rate: int = 115200,
        timeout: float = 1.0,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout

        self.connection: Optional[serial.Serial] = None

        # Only one normal command may read responses at a time.
        self.command_lock = threading.Lock()

        # Serial writes are protected separately so STOP can be sent
        # while POSITION_FOR_PICKUP is active.
        self.write_lock = threading.Lock()

        self.positioning_active = threading.Event()

    def connect(self) -> None:
        if self.is_connected():
            return

        print(f"Opening Arduino on {self.port}...")

        self.connection = serial.Serial(
            port=self.port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )

        # Opening USB serial resets the Uno.
        # Allow time for MPU6050 calibration.
        deadline = time.monotonic() + 8

        while time.monotonic() < deadline:
            line = self._read_line()

            if line:
                print(f"Arduino: {line}")

            if line == "ARDUINO_READY":
                print("Arduino is ready.")
                return

        self.close()

        raise TimeoutError(
            "Arduino did not report ARDUINO_READY."
        )

    def is_connected(self) -> bool:
        return (
            self.connection is not None
            and self.connection.is_open
        )

    def _read_line(self) -> str:
        if not self.is_connected():
            return ""

        raw_line = self.connection.readline()

        if not raw_line:
            return ""

        return raw_line.decode(
            "utf-8",
            errors="replace",
        ).strip()

    def _write_command(self, command: str) -> None:
        if not self.is_connected():
            raise RuntimeError(
                "Arduino is not connected."
            )

        with self.write_lock:
            print(f"Raspberry Pi -> Arduino: {command}")

            self.connection.write(
                f"{command}\n".encode("utf-8")
            )
            self.connection.flush()

    def _send_and_wait(
        self,
        command: str,
        expected_prefix: str,
        timeout_seconds: float,
    ) -> str:
        if not self.is_connected():
            raise RuntimeError(
                "Arduino is not connected."
            )

        with self.command_lock:
            self._write_command(command)

            deadline = (
                time.monotonic() + timeout_seconds
            )

            while time.monotonic() < deadline:
                line = self._read_line()

                if not line:
                    continue

                print(f"Arduino -> Raspberry Pi: {line}")

                if line.startswith("ERROR,"):
                    raise RuntimeError(line)

                if line.startswith(expected_prefix):
                    return line

            raise TimeoutError(
                f"Timed out waiting for: "
                f"{expected_prefix}"
            )

    def ping(self) -> bool:
        response = self._send_and_wait(
            command="PING",
            expected_prefix="PONG",
            timeout_seconds=3,
        )

        return response == "PONG"


    def start_continuous_forward(self) -> None:
        """Start forward motion without a fixed duration."""
        self._send_and_wait(
            command="START_FORWARD",
            expected_prefix="CONTINUOUS_FORWARD_STARTED",
            timeout_seconds=2.0,
        )



    def refresh_continuous_forward(self) -> None:
        """
        Refresh continuous driving.

        Arduino may reply STARTED if its safety timeout expired
        immediately before the refresh arrived.
        """
        if not self.is_connected():
            raise RuntimeError(
                "Arduino is not connected."
            )

        with self.command_lock:
            self._write_command("REFRESH_FORWARD")

            deadline = time.monotonic() + 2.0

            while time.monotonic() < deadline:
                line = self._read_line()

                if not line:
                    continue

                print(
                    "Arduino -> Raspberry Pi: "
                    f"{line}"
                )

                if line.startswith("ERROR,"):
                    raise RuntimeError(line)

                if line.startswith(
                    "CONTINUOUS_FORWARD_REFRESHED"
                ):
                    return

                if line.startswith(
                    "CONTINUOUS_FORWARD_STARTED"
                ):
                    return

            raise TimeoutError(
                "Timed out waiting for "
                "continuous-forward refresh."
            )

    def forward(self, duration_ms: int) -> int:
        self._validate_duration(duration_ms)

        response = self._send_and_wait(
            command=f"FORWARD {duration_ms}",
            expected_prefix="COMMAND_DONE,FORWARD",
            timeout_seconds=(duration_ms / 1000) + 3,
        )

        return self._extract_duration(
            response=response,
            fallback=duration_ms,
        )

    def backward(self, duration_ms: int) -> int:
        self._validate_duration(duration_ms)

        response = self._send_and_wait(
            command=f"BACKWARD {duration_ms}",
            expected_prefix="COMMAND_DONE,BACKWARD",
            timeout_seconds=(duration_ms / 1000) + 3,
        )

        return self._extract_duration(
            response=response,
            fallback=duration_ms,
        )

    def turn_right(self, angle: float) -> float:
        return self._turn(
            command_name="TURN_RIGHT",
            angle=angle,
        )

    def turn_left(self, angle: float) -> float:
        return self._turn(
            command_name="TURN_LEFT",
            angle=angle,
        )

    def _turn(
        self,
        command_name: str,
        angle: float,
    ) -> float:
        if not 1 <= angle <= 180:
            raise ValueError(
                "Turn angle must be from 1 to 180."
            )

        response = self._send_and_wait(
            command=f"{command_name} {angle}",
            expected_prefix=(
                f"COMMAND_DONE,{command_name}"
            ),
            timeout_seconds=13,
        )

        match = re.search(
            r"FINAL_ANGLE=([-+]?\d+(?:\.\d+)?)",
            response,
        )

        if match is None:
            raise RuntimeError(
                f"Final angle missing: {response}"
            )

        return float(match.group(1))


    def read_distance_cm(self) -> Optional[float]:
        """
        Read one filtered ultrasonic distance while the robot is
        stopped.

        Returns None when Arduino reports no valid echo.
        """
        if not self.is_connected():
            raise RuntimeError(
                "Arduino is not connected."
            )

        with self.command_lock:
            self._write_command("READ_DISTANCE")

            deadline = time.monotonic() + 2.5

            while time.monotonic() < deadline:
                line = self._read_line()

                if not line:
                    continue

                print(
                    "Arduino -> Raspberry Pi: "
                    f"{line}"
                )

                if line == "DISTANCE_INVALID":
                    return None

                if line.startswith("DISTANCE_CM="):
                    return self._extract_float(
                        response=line,
                        field_name="DISTANCE_CM",
                    )

                if line.startswith("ERROR,"):
                    raise RuntimeError(line)

            raise TimeoutError(
                "Timed out waiting for ultrasonic "
                "distance."
            )

    def position_for_pickup(self) -> dict:
        """
        Run Arduino ultrasonic fine positioning.

        Returns every completed pulse so the Raspberry Pi can
        include the real movements in its return-path history.
        """
        if not self.is_connected():
            raise RuntimeError(
                "Arduino is not connected."
            )

        pulses = []
        final_distance_cm = None
        pickup_reached = False

        with self.command_lock:
            self.positioning_active.set()

            try:
                self._write_command(
                    "POSITION_FOR_PICKUP"
                )

                deadline = time.monotonic() + 23

                while time.monotonic() < deadline:
                    line = self._read_line()

                    if not line:
                        continue

                    print(
                        "Arduino -> Raspberry Pi: "
                        f"{line}"
                    )

                    if line.startswith(
                        "FORWARD_PULSE_DONE,"
                    ):
                        duration_ms = (
                            self._extract_duration(
                                response=line,
                                fallback=None,
                            )
                        )

                        pulses.append(
                            {
                                "command": "FORWARD",
                                "duration_ms": duration_ms,
                                "source": "ULTRASONIC",
                            }
                        )
                        continue

                    if line.startswith(
                        "BACKWARD_PULSE_DONE,"
                    ):
                        duration_ms = (
                            self._extract_duration(
                                response=line,
                                fallback=None,
                            )
                        )

                        pulses.append(
                            {
                                "command": "BACKWARD",
                                "duration_ms": duration_ms,
                                "source": "ULTRASONIC",
                            }
                        )
                        continue

                    if line.startswith(
                        "PICKUP_POSITION_REACHED,"
                    ):
                        final_distance_cm = (
                            self._extract_float(
                                response=line,
                                field_name="DISTANCE_CM",
                            )
                        )
                        pickup_reached = True
                        continue

                    if line.startswith(
                        "COMMAND_DONE,"
                        "POSITION_FOR_PICKUP"
                    ):
                        if not pickup_reached:
                            raise RuntimeError(
                                "Arduino completed pickup "
                                "positioning without reporting "
                                "PICKUP_POSITION_REACHED."
                            )

                        return {
                            "success": True,
                            "distance_cm":
                                final_distance_cm,
                            "pulses": pulses,
                        }

                    if line.startswith(
                        "COMMAND_ABORTED,"
                    ):
                        reason = "UNKNOWN"

                        match = re.search(
                            r"REASON=([^,]+)",
                            line,
                        )

                        if match is not None:
                            reason = match.group(1)

                        return {
                            "success": False,
                            "reason": reason,
                            "distance_cm":
                                final_distance_cm,
                            "pulses": pulses,
                        }

                    if line.startswith("ERROR,"):
                        raise RuntimeError(line)

                    if line == "STOPPED":
                        return {
                            "success": False,
                            "reason": "STOP",
                            "distance_cm":
                                final_distance_cm,
                            "pulses": pulses,
                        }

                raise TimeoutError(
                    "Timed out waiting for pickup "
                    "positioning to finish."
                )

            finally:
                self.positioning_active.clear()

    def grab_object(self) -> None:
        self._send_and_wait(
            command="GRAB_OBJECT",
            expected_prefix="COMMAND_DONE,GRAB_OBJECT",
            timeout_seconds=8.0,
        )

    def release_object(self) -> None:
        self._send_and_wait(
            command="RELEASE_OBJECT",
            expected_prefix="COMMAND_DONE,RELEASE_OBJECT",
            timeout_seconds=5.0,
        )

    def stop(self) -> None:
        if not self.is_connected():
            return

        # POSITION_FOR_PICKUP reads serial while holding the command
        # lock. Send STOP directly so Arduino can abort immediately.
        if self.positioning_active.is_set():
            try:
                self._write_command("STOP")
            except Exception as error:
                print(
                    f"Arduino STOP warning: {error}"
                )
            return

        try:
            self._send_and_wait(
                command="STOP",
                expected_prefix="STOPPED",
                timeout_seconds=2,
            )
        except Exception as error:
            print(f"Arduino STOP warning: {error}")

    def close(self) -> None:
        if self.connection is None:
            return

        if self.connection.is_open:
            self.connection.close()

        self.connection = None
        print("Arduino connection closed.")

    @staticmethod
    def _extract_duration(
        response: str,
        fallback: Optional[int],
    ) -> int:
        match = re.search(
            r"DURATION_MS=(\d+)",
            response,
        )

        if match is not None:
            return int(match.group(1))

        if fallback is not None:
            return fallback

        raise RuntimeError(
            f"Duration missing: {response}"
        )

    @staticmethod
    def _extract_float(
        response: str,
        field_name: str,
    ) -> float:
        match = re.search(
            rf"{re.escape(field_name)}="
            r"([-+]?\d+(?:\.\d+)?)",
            response,
        )

        if match is None:
            raise RuntimeError(
                f"{field_name} missing: {response}"
            )

        return float(match.group(1))

    @staticmethod
    def _validate_duration(duration_ms: int) -> None:
        if not 1 <= duration_ms <= 5000:
            raise ValueError(
                "Movement duration must be "
                "from 1 to 5000 ms."
            )