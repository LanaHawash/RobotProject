import json

from vosk import KaldiRecognizer, Model

from robot_project.audio.config import (
    SAMPLE_RATE,
    VOSK_MODEL_PATH,
)
from robot_project.audio.microphone import (
    Microphone,
)


COMMANDS = [
    "start navigation",
    "start deep cleaning",
    "stop",
    "cancel",
]


def main():
    microphone = Microphone()

    model = Model(
        str(VOSK_MODEL_PATH)
    )

    recognizer = KaldiRecognizer(
        model,
        SAMPLE_RATE,
        json.dumps(
            COMMANDS + ["[unk]"]
        ),
    )

    microphone.start()

    print("Vosk live test started.")
    print("Say: start deep cleaning")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            stereo = microphone.read()
            mono = microphone.to_mono(
                stereo
            )

            accepted = recognizer.AcceptWaveform(
                mono.tobytes()
            )

            if accepted:
                print(
                    "FINAL:",
                    recognizer.Result(),
                )
            else:
                partial = json.loads(
                    recognizer.PartialResult()
                )

                text = partial.get(
                    "partial",
                    "",
                )

                if text:
                    print(
                        "PARTIAL:",
                        text,
                    )

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        microphone.stop()


if __name__ == "__main__":
    main()