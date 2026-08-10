from pathlib import Path

import numpy as np
import openwakeword
from openwakeword.model import Model


class WakeWordDetector:
    """
    Detects the robot wake word using openWakeWord.

    We temporarily use the built-in "Hey Jarvis"
    model to test the software pipeline.

    Later this model will be replaced by the
    custom "Hey Tiddy" model.
    """

    DEFAULT_THRESHOLD = 0.5

    def __init__(
        self,
        model_path: str | Path | None = None,
        threshold: float = DEFAULT_THRESHOLD,
    ):
        self.threshold = threshold

        if model_path is None:
            model_path = self._find_test_model()

        self.model_path = Path(model_path)

        self.model = Model(
            wakeword_models=[
                str(self.model_path)
            ],
            inference_framework="onnx",
        )

    def _find_test_model(self) -> Path:
        """
        Find the built-in Hey Jarvis ONNX model.
        """

        model_paths = (
            openwakeword.get_pretrained_model_paths()
        )

        for path in model_paths:
            path = Path(path)

            if "hey_jarvis" in path.name.lower():
                onnx_path = path.with_suffix(".onnx")

                if onnx_path.exists():
                    return onnx_path

        raise RuntimeError(
            "Hey Jarvis ONNX test model was not found."
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

        detected = score >= self.threshold

        return detected, score