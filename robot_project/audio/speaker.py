import random
import subprocess

from robot_project.audio.config import (
    BABY_LULLABY_PATHS,
)


class Speaker:
    """
    Plays baby songs through the Raspberry Pi's
    configured system audio output.
    """

    def __init__(self):
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
                    "ffplay",
                    "-nodisp",
                    "-autoexit",
                    "-loglevel",
                    "quiet",
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