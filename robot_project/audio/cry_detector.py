import csv
from pathlib import Path

import numpy as np
import tensorflow as tf
import tensorflow_hub as hub

from robot_project.audio.config import (
    BABY_CRY_LABEL,
    BABY_CRY_THRESHOLD,
    CRY_WINDOW_SAMPLES,
    YAMNET_MODEL_DIR,
)


class CryDetector:
    """
    Detects baby crying using YAMNet.

    Input:
        mono signed 16-bit PCM audio at 16 kHz

    Output:
        detected flag and confidence score
    """

    def __init__(
        self,
        threshold: float = BABY_CRY_THRESHOLD,
    ):
        self.threshold = threshold

        self.model_path = (
            self._find_cached_model()
        )

        print(
            f"Loading YAMNet from {self.model_path}"
        )

        self.model = hub.load(
            str(self.model_path)
        )

        self.class_names = (
            self._load_class_names()
        )

        self.baby_cry_index = (
            self._find_baby_cry_index()
        )

    def _find_cached_model(self) -> Path:
        """
        Find the YAMNet SavedModel that TensorFlow Hub
        already downloaded into models/audio/yamnet.
        """

        if not YAMNET_MODEL_DIR.exists():
            raise RuntimeError(
                "YAMNet model directory does not exist."
            )

        for path in YAMNET_MODEL_DIR.iterdir():
            if (
                path.is_dir()
                and (path / "saved_model.pb").exists()
            ):
                return path

        raise RuntimeError(
            "Cached YAMNet SavedModel was not found."
        )

    def _load_class_names(self) -> list[str]:
        """
        Load YAMNet's AudioSet class names.
        """

        class_map_path = (
            self.model
            .class_map_path()
            .numpy()
            .decode()
        )

        class_names = []

        with open(
            class_map_path,
            "r",
            encoding="utf-8",
        ) as class_map_file:

            reader = csv.DictReader(
                class_map_file
            )

            for row in reader:
                class_names.append(
                    row["display_name"]
                )

        return class_names

    def _find_baby_cry_index(self) -> int:
        """
        Find the class index corresponding to
        'Baby cry, infant cry'.
        """

        try:
            return self.class_names.index(
                BABY_CRY_LABEL
            )

        except ValueError as error:
            raise RuntimeError(
                f"YAMNet class not found: "
                f"{BABY_CRY_LABEL}"
            ) from error

    def process_audio(
        self,
        audio: np.ndarray,
    ) -> tuple[bool, float]:
        """
        Process approximately one second of mono
        int16 audio.

        Returns:
            (detected, confidence)
        """

        if audio.dtype != np.int16:
            raise ValueError(
                "YAMNet audio must use np.int16."
            )

        if audio.ndim != 1:
            raise ValueError(
                "YAMNet expects mono audio."
            )

        if len(audio) < CRY_WINDOW_SAMPLES:
            raise ValueError(
                "YAMNet cry detection requires "
                f"at least {CRY_WINDOW_SAMPLES} "
                "samples."
            )

        waveform = (
            audio.astype(np.float32)
            / 32768.0
        )

        scores, _, _ = self.model(
            waveform
        )

        baby_scores = scores[
            :,
            self.baby_cry_index,
        ]

        score = float(
            tf.reduce_max(
                baby_scores
            ).numpy()
        )

        detected = (
            score >= self.threshold
        )

        return detected, score