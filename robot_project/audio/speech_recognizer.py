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

    Final Vosk segments are accumulated so a command can
    survive a small pause between words, for example:

        "start" + "navigation"

    Partial results are also used for lower-latency command
    detection.
    """

    COMMANDS = [
        "start navigation",
        "start deep cleaning",
        "stop",
        
    ]

    # Two matching partials is fast while still giving some
    # protection against one unstable partial result.
    PARTIAL_CONFIRMATION_REQUIRED = 2

    def __init__(self):
        if not VOSK_MODEL_PATH.exists():
            raise RuntimeError(
                f"Vosk model not found: {VOSK_MODEL_PATH}"
            )

        self.model = Model(
            str(VOSK_MODEL_PATH)
        )

        grammar = json.dumps(
            self.COMMANDS + ["[unk]"]
        )

        self.recognizer = KaldiRecognizer(
            self.model,
            SAMPLE_RATE,
            grammar,
        )

        self.committed_text = ""

        self.partial_candidate = None
        self.partial_confirmation_count = 0


    def reset(self) -> None:
        self.recognizer.Reset()

        self.committed_text = ""

        self.partial_candidate = None
        self.partial_confirmation_count = 0


    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join(
            text.lower().strip().split()
        )


    def _match_command(
        self,
        text: str,
    ) -> str | None:

        text = self._normalize(text)

        if text in self.COMMANDS:
            return text

        return None


    def _could_be_command(
        self,
        text: str,
    ) -> bool:

        text = self._normalize(text)

        if not text:
            return True

        return any(
            command.startswith(text)
            for command in self.COMMANDS
        )


    def _process_partial_candidate(
        self,
        text: str,
    ) -> str | None:

        command = self._match_command(text)

        if command is None:
            self.partial_candidate = None
            self.partial_confirmation_count = 0
            return None

        if command == self.partial_candidate:
            self.partial_confirmation_count += 1

        else:
            self.partial_candidate = command
            self.partial_confirmation_count = 1

        if (
            self.partial_confirmation_count
            >= self.PARTIAL_CONFIRMATION_REQUIRED
        ):
            self.partial_candidate = None
            self.partial_confirmation_count = 0

            return command

        return None


    def process_audio(
        self,
        audio: np.ndarray,
    ) -> str | None:

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

            segment = self._normalize(
                result.get(
                    "text",
                    "",
                )
            )

            if not segment:
                return None

            combined = self._normalize(
                f"{self.committed_text} {segment}"
            )

            command = self._match_command(
                combined
            )

            if command is not None:
                return command

            # Keep incomplete command fragments.
            #
            # Example:
            #     committed_text = "start"
            #
            # so that the next Vosk result:
            #     "navigation"
            #
            # becomes:
            #     "start navigation"
            if self._could_be_command(
                combined
            ):
                self.committed_text = combined

            elif self._could_be_command(
                segment
            ):
                self.committed_text = segment

            else:
                self.committed_text = ""

            self.partial_candidate = None
            self.partial_confirmation_count = 0

            return None


        partial_result = json.loads(
            self.recognizer.PartialResult()
        )

        partial_text = self._normalize(
            partial_result.get(
                "partial",
                "",
            )
        )

        if not partial_text:
            return None

        combined = self._normalize(
            f"{self.committed_text} {partial_text}"
        )

        command = self._process_partial_candidate(
            combined
        )

        if command is not None:
            return command

        # If an old committed fragment no longer makes
        # sense, also try the new partial on its own.
        if not self._could_be_command(
            combined
        ):
            return self._process_partial_candidate(
                partial_text
            )

        return None