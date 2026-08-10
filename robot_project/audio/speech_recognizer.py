import json

import numpy as np
from vosk import KaldiRecognizer, Model

from robot_project.audio.config import (
    SAMPLE_RATE,
    VOSK_MODEL_PATH,
)


class SpeechRecognizer:
    """
    Converts mono 16-bit PCM speech into robot commands
    using Vosk.
    """

    COMMANDS = [
        "start navigation",
        "start deep cleaning",
        "stop",
        "cancel",
    ]
    PARTIAL_CONFIRMATION_REQUIRED = 3

    def __init__(self):
        if not VOSK_MODEL_PATH.exists():
            raise RuntimeError(
                f"Vosk model not found: {VOSK_MODEL_PATH}"
            )

        self.model = Model(str(VOSK_MODEL_PATH))

        grammar = json.dumps(
            self.COMMANDS + ["[unk]"]
        )

        self.recognizer = KaldiRecognizer(
            self.model,
            SAMPLE_RATE,
            grammar,
        )
        self.partial_candidate = None
        self.partial_confirmation_count = 0

        

    def reset(self) -> None:
        """
        Reset the recognizer before listening for
        a new command.
        """

        self.recognizer.Reset()

        self.partial_candidate = None
        self.partial_confirmation_count = 0

    def process_audio(
        self,
        audio: np.ndarray,
    ) -> str | None:
        """
        Process mono signed 16-bit PCM samples.

        Returns a recognized robot command when either:

        - Vosk produces a valid final result, or
        - the same complete command appears consistently
        in partial recognition results.
        """

        if audio.dtype != np.int16:
            raise ValueError(
                "Vosk audio must use np.int16 samples."
            )

        if audio.ndim != 1:
            raise ValueError(
                "Vosk expects mono audio."
            )

        accepted = self.recognizer.AcceptWaveform(
            audio.tobytes()
        )

        if accepted:
            result = json.loads(
                self.recognizer.Result()
            )

            text = result.get(
                "text",
                "",
            ).strip()

            if text in self.COMMANDS:
                return text

            return None

        partial_result = json.loads(
            self.recognizer.PartialResult()
        )

        partial_text = partial_result.get(
            "partial",
            "",
        ).strip()

        if partial_text not in self.COMMANDS:
            self.partial_candidate = None
            self.partial_confirmation_count = 0
            return None

        if partial_text == self.partial_candidate:
            self.partial_confirmation_count += 1

        else:
            self.partial_candidate = partial_text
            self.partial_confirmation_count = 1

        if (
            self.partial_confirmation_count
            >= self.PARTIAL_CONFIRMATION_REQUIRED
        ):
            command = self.partial_candidate

            self.partial_candidate = None
            self.partial_confirmation_count = 0

            return command

        return None