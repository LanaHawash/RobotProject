import time

import serial


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


def main():
    arduino = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=2,
    )

    # Arduino resets when the serial connection opens.
    time.sleep(2)

    print(f"Connected to Arduino on {SERIAL_PORT}")

    # Read the Arduino startup message.
    startup_message = arduino.readline().decode("utf-8").strip()
    print(f"Arduino startup: {startup_message}")

    arduino.write(b"PING\n")
    arduino.flush()

    response = arduino.readline().decode("utf-8").strip()
    print(f"Arduino response: {response}")

    if response == "PONG":
        print("Raspberry Pi <-> Arduino communication works.")
    else:
        print("Communication opened, but PONG was not received.")

    arduino.close()


if __name__ == "__main__":
    main()