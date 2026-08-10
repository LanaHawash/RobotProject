import time

import numpy as np

from robot_project.audio.config import (
    AUDIO_CHUNK_SAMPLES,
    COMMAND_LISTEN_SECONDS,
    CRY_WINDOW_SAMPLES,
)
from robot_project.audio.wake_word import WakeWordDetector
from robot_project.audio.speech_recognizer import SpeechRecognizer
from robot_project.audio.cry_detector import CryDetector


class AudioService:
    """
    Coordinates the robot's audio intelligence.

    For now this class does NOT:
    - open the microphone
    - control the speaker
    - control navigation
    - communicate with Arduino

    It only processes mono int16 audio.
    """

    STATE_IDLE = "IDLE"
    STATE_COMMAND_LISTENING = "COMMAND_LISTENING"

    def __init__(self):
        self.wake_word = WakeWordDetector()
        self.speech_recognizer = SpeechRecognizer()
        self.cry_detector = CryDetector()

        self.state = self.STATE_IDLE

        self.command_deadline = None

        self.cry_buffer = np.empty(
            0,
            dtype=np.int16,
        )

    def process_audio(
        self,
        audio: np.ndarray,
    ) -> list[dict]:
        """
        Process one 80 ms mono audio chunk.

        Returns zero or more events.
        """

        self._validate_audio(audio)

        events = []

        # Environmental sound detection runs
        # independently of the wake-word state.
        self._process_cry_audio(
            audio,
            events,
        )

        if self.state == self.STATE_IDLE:
            self._process_wake_word(
                audio,
                events,
            )

        elif (
            self.state
            == self.STATE_COMMAND_LISTENING
        ):
            self._process_command(
                audio,
                events,
            )

        return events

    def _validate_audio(
        self,
        audio: np.ndarray,
    ) -> None:

        if audio.dtype != np.int16:
            raise ValueError(
                "AudioService expects np.int16 audio."
            )

        if audio.ndim != 1:
            raise ValueError(
                "AudioService expects mono audio."
            )

        if len(audio) != AUDIO_CHUNK_SAMPLES:
            raise ValueError(
                "AudioService expects "
                f"{AUDIO_CHUNK_SAMPLES} samples "
                "per chunk."
            )

    def _process_wake_word(
        self,
        audio: np.ndarray,
        events: list[dict],
    ) -> None:

        detected, score = (
            self.wake_word.process_audio(
                audio
            )
        )

        if not detected:
            return

        self.speech_recognizer.reset()

        self.state = (
            self.STATE_COMMAND_LISTENING
        )

        self.command_deadline = (
            time.monotonic()
            + COMMAND_LISTEN_SECONDS
        )

        events.append(
            {
                "type": "wake_word",
                "score": score,
            }
        )

    def _process_command(
        self,
        audio: np.ndarray,
        events: list[dict],
    ) -> None:

        command = (
            self.speech_recognizer.process_audio(
                audio
            )
        )

        if command is not None:
            events.append(
                {
                    "type": "command",
                    "command": command,
                }
            )

            self._return_to_idle()
            return

        if (
            self.command_deadline is not None
            and time.monotonic()
            >= self.command_deadline
        ):
            events.append(
                {
                    "type": "command_timeout",
                }
            )

            self._return_to_idle()

    def _process_cry_audio(
        self,
        audio: np.ndarray,
        events: list[dict],
    ) -> None:
        """
        Accumulate the small 80 ms microphone chunks
        until we have enough audio for YAMNet.
        """

        self.cry_buffer = np.concatenate(
            (
                self.cry_buffer,
                audio,
            )
        )

        while (
            len(self.cry_buffer)
            >= CRY_WINDOW_SAMPLES
        ):
            cry_window = self.cry_buffer[
                :CRY_WINDOW_SAMPLES
            ]

            self.cry_buffer = self.cry_buffer[
                CRY_WINDOW_SAMPLES:
            ]

            detected, score = (
                self.cry_detector.process_audio(
                    cry_window
                )
            )

            if detected:
                events.append(
                    {
                        "type": "baby_cry",
                        "score": score,
                    }
                )

    def _return_to_idle(self) -> None:
        self.state = self.STATE_IDLE
        self.command_deadline = None

        self.speech_recognizer.reset()

    def get_status(self) -> dict:
        return {
            "state": self.state,
            "command_deadline_active": (
                self.command_deadline
                is not None
            ),
            "cry_buffer_samples": len(
                self.cry_buffer
            ),
        }