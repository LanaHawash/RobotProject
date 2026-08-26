class ManualDriveController:
    """
    Handles mobile-app manual driving.

    This class does not create another Arduino serial connection.
    It reuses the existing ArduinoController instance.
    """

    ALLOWED_COMMANDS = {
        "FORWARD",
        "BACKWARD",
        "LEFT",
        "RIGHT",
        "STOP",
    }

    def __init__(self, arduino):
        self.arduino = arduino

    def move(self, command: str) -> None:
        direction = command.strip().upper()

        if direction not in self.ALLOWED_COMMANDS:
            raise ValueError(
                f"Invalid manual movement command: {direction}"
            )

        if direction == "STOP":
            self.arduino.stop()
            return

        self.arduino._send_and_wait(
            command=f"MANUAL {direction}",
            expected_prefix=(
                f"MANUAL_DRIVE_STARTED,{direction}"
            ),
            timeout_seconds=2.0,
        )