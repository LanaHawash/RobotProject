import time
from typing import Optional

import serial


class SerialController:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud_rate: int = 115200,
        timeout: float = 2.0,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.connection: Optional[serial.Serial] = None

    def connect(self) -> None:
        if self.is_connected():
            return

        print(f"Opening Arduino serial port: {self.port}")

        self.connection = serial.Serial(
            port=self.port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )

        # Arduino Uno usually resets when the USB serial port opens.
        time.sleep(2)

        self.connection.reset_input_buffer()
        self.connection.reset_output_buffer()

        print("Arduino serial connection opened.")

    def is_connected(self) -> bool:
        return (
            self.connection is not None
            and self.connection.is_open
        )

    def send_command(self, command: str) -> str:
        if not self.is_connected():
            raise RuntimeError("Arduino connection is not open.")

        command = command.strip()

        if not command:
            raise ValueError("Command cannot be empty.")

        self.connection.write(f"{command}\n".encode("utf-8"))
        self.connection.flush()

        response = (
            self.connection.readline()
            .decode("utf-8", errors="replace")
            .strip()
        )

        if not response:
            raise TimeoutError(
                f"No response for command: {command}"
            )

        return response

    def stop(self) -> str:
        return self.send_command("STOP")

    def close(self) -> None:
        if self.connection is not None:
            if self.connection.is_open:
                self.connection.close()

            self.connection = None
            print("Arduino serial connection closed.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            if self.is_connected():
                self.stop()
        except Exception as error:
            print(f"Warning: STOP failed: {error}")
        finally:
            self.close()
