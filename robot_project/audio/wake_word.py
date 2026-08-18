from pathlib import Path
import numpy as np
import openwakeword
from openwakeword.model import Model
from robot_project.audio.config import (
    WAKE_WORD_MODEL_PATH,
    WAKE_WORD_THRESHOLD,
)


class WakeWordDetector:
    """
    Detects the custom "Hey Robo" wake word
    using openWakeWord.
    """

    DEFAULT_THRESHOLD = WAKE_WORD_THRESHOLD

    def __init__(
        self,
        model_path: str | Path | None = None,
        threshold: float = DEFAULT_THRESHOLD,
    ):
        self.threshold = threshold

        if model_path is None:
            model_path = WAKE_WORD_MODEL_PATH

        self.model_path = Path(model_path)

        if not self.model_path.exists():
            raise RuntimeError(
                "Hey Robo wake-word model was not found: "
                f"{self.model_path}"
            )

        self.model = Model(
            wakeword_models=[
                str(self.model_path)
            ],
            inference_framework="onnx",
        )

   
    def reset(self) -> None:
        """
        Reset openWakeWord's internal prediction state.
        """

        self.model.reset()

    def process_audio(
        self,
        audio: np.ndarray,
    ) -> tuple[bool, float]:
        """
        Process mono signed 16-bit PCM audio.

        Returns:
            (detected, confidence)
        """

        if audio.dtype != np.int16:
            raise ValueError(
                "Wake-word audio must use np.int16."
            )

        if audio.ndim != 1:
            raise ValueError(
                "Wake-word audio must be mono."
            )

        predictions = self.model.predict(audio)

        if not predictions:
            return False, 0.0

        score = max(
            float(value)
            for value in predictions.values()
        )

        # print(
        #     f"Hey Robo score: {score:.4f} | "
        #     f"threshold: {self.threshold:.2f}"
        # )

        detected = score >= self.threshold

        return detected, score