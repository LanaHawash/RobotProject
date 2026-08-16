from robot_project.audio.audio_service import (
    AudioService,
)
from robot_project.audio.microphone import (
    Microphone,
)


def main():
    microphone = Microphone()
    audio_service = AudioService()

    microphone.start()

    print("Live audio test started.")
    print("Say: Hey Robo")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            stereo_audio = microphone.read()

            mono_audio = microphone.to_mono(
                stereo_audio
            )

            events = audio_service.process_audio(
                mono_audio
            )

            for event in events:
                print("EVENT:", event)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        microphone.stop()


if __name__ == "__main__":
    main()