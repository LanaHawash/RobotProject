import random
import subprocess

from robot_project.audio.config import (
    BABY_LULLABY_PATHS,
)


class Speaker:
    """
    Plays speech responses and baby songs through
    the Raspberry Pi configured audio output.
    """

    def __init__(self):
        self.playing = False

    def speak(
        self,
        text: str,
    ) -> None:
        if not text:
            return

        print(
            f"Robot speaking: {text}"
        )

        self.playing = True

        try:
            subprocess.run(
                [
                    "espeak-ng",
                    "-v",
                    "en+f3",
                    "-s",
                    "140",
                    "-p",
                    "45",
                    text,
                ],
                check=True,
                timeout=10,
            )

        finally:
            self.playing = False

    def play_baby_lullaby(self) -> None:
        song_path = random.choice(
            BABY_LULLABY_PATHS
        )

        if not song_path.exists():
            raise FileNotFoundError(
                f"Baby song not found: {song_path}"
            )

        print(
            f"Playing baby song: {song_path.name}"
        )

        self.playing = True

        try:
            process = subprocess.Popen(
                [
                    "mpg123",
                    "-q",
                    str(song_path),
                ]
            )

            try:
                process.wait(timeout=20)

            except subprocess.TimeoutExpired:
                process.terminate()

                try:
                    process.wait(timeout=2)

                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

        finally:
            self.playing = False

    def is_playing(self) -> bool:
        return self.playing