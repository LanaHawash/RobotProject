import time

import serial


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


def read_arduino_line(arduino: serial.Serial) -> str | None:
    raw_line = arduino.readline()

    if not raw_line:
        return None

    return raw_line.decode(
        "utf-8",
        errors="replace",
    ).strip()


def main() -> None:
    print(f"Opening Arduino on {SERIAL_PORT}...")

    arduino = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=1,
        write_timeout=1,
    )

    try:
        # Opening the serial connection resets an Arduino Uno.
        print("Waiting for Arduino startup and IMU calibration...")
        time.sleep(5)

        # Read startup messages currently waiting in the buffer.
        while arduino.in_waiting > 0:
            line = read_arduino_line(arduino)

            if line:
                print(f"Arduino: {line}")

        print("Checking communication...")
        arduino.write(b"PING\n")
        arduino.flush()

        ping_deadline = time.monotonic() + 3

        while time.monotonic() < ping_deadline:
            line = read_arduino_line(arduino)

            if not line:
                continue

            print(f"Arduino: {line}")

            if line == "PONG":
                break
        else:
            raise RuntimeError("Arduino did not respond to PING.")

        print()
        print("Sending RUN_TEST...")
        arduino.write(b"RUN_TEST\n")
        arduino.flush()

        sequence_deadline = time.monotonic() + 15

        while time.monotonic() < sequence_deadline:
            line = read_arduino_line(arduino)

            if not line:
                continue

            print(f"Arduino: {line}")

            if line == "SEQUENCE_DONE":
                print()
                print("Movement and IMU test completed successfully.")
                return

            if line == "MPU6050_CONNECTION_FAILED":
                raise RuntimeError(
                    "Arduino cannot communicate with the MPU6050."
                )

            if line == "TURN_TIMEOUT":
                print(
                    "The robot stopped because the 30-degree turn "
                    "was not detected before the safety timeout."
                )

        raise TimeoutError(
            "The Arduino sequence did not finish in time."
        )

    except KeyboardInterrupt:
        print("\nCtrl+C received. Sending STOP...")

    finally:
        try:
            arduino.write(b"STOP\n")
            arduino.flush()
            time.sleep(0.2)
        except serial.SerialException:
            pass

        arduino.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()