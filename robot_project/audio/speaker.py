from concurrent.futures import process
import random
import subprocess
import threading
import numpy as np

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
        self.talk_process = None
        self.talk_lock = threading.Lock()

    def speak(self, text: str) -> None:
        if not text:
            return

        print(f"Robot speaking: {text}")

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
        song_path = random.choice(BABY_LULLABY_PATHS)

        if not song_path.exists():
            raise FileNotFoundError(
                f"Baby song not found: {song_path}"
            )

        print(f"Playing baby song: {song_path.name}")

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

    # ---------------------------------------------------------
    # LIVE TALK
    # ---------------------------------------------------------

    def start_live_talk(self) -> None:
        with self.talk_lock:
            if self.talk_process is not None:
                return

            print("Starting live talk speaker.")

            self.playing = True

            self.talk_process = subprocess.Popen(
                [
                    "aplay",
                    "-t", "raw",
                    "-f", "S16_LE",
                    "-r", "16000",
                    "-c", "1",
                ],
                stdin=subprocess.PIPE,
            )
            print(
            f"aplay started with PID: "
            f"{self.talk_process.pid}"
        )

    def write_live_audio(self, audio_data: bytes) -> None:
     with self.talk_lock:
        process = self.talk_process

        print(
            f"Speaker received {len(audio_data)} bytes"
        )

        if process is None:
            print("ERROR: aplay process is None")
            return

        if process.stdin is None:
            print("ERROR: aplay stdin is None")
            return

        return_code = process.poll()

        if return_code is not None:
            print(
                f"ERROR: aplay stopped. "
                f"Return code: {return_code}"
            )
            return

        try:
            # samples = np.frombuffer(
            #     audio_data,
            #     dtype="<i2",
            # ).astype(np.int32)

            # if len(samples) > 0:
            #     peak = int(np.max(np.abs(samples)))

            #     rms = int(
            #         np.sqrt(
            #             np.mean(
            #                 samples.astype(np.float64) ** 2
            #             )
            #         )
            #     )

            #     print(
            #         f"PHONE AUDIO: peak={peak}, rms={rms}"
            #     )
            process.stdin.write(audio_data)
            process.stdin.flush()

            print(
                f"Wrote {len(audio_data)} bytes to aplay"
            )

        except (BrokenPipeError, OSError) as error:
            print(
                f"ERROR writing audio to aplay: {error}"
            )


            
    def stop_live_talk(self) -> None:
        with self.talk_lock:
            process = self.talk_process

            if process is None:
                return

            print("Stopping live talk speaker.")

            try:
                # Closing stdin tells aplay that the raw audio
                # stream has ended normally.
                if process.stdin is not None:
                    try:
                        process.stdin.close()
                    except (BrokenPipeError, OSError):
                        pass

                # Give aplay a chance to exit cleanly first.
                try:
                    process.wait(timeout=1.0)

                except subprocess.TimeoutExpired:
                    process.terminate()

                    try:
                        process.wait(timeout=2.0)

                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()

            finally:
                self.talk_process = None
                self.playing = False