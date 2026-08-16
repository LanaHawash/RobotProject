import queue
import threading

import numpy as np
import sounddevice as sd

from robot_project.audio.config import (
    SAMPLE_RATE,
    CAPTURE_CHANNELS,
    SAMPLE_WIDTH_BYTES,
    AUDIO_CHUNK_SAMPLES,
)


class Microphone:
    """
    Captures stereo audio from the two INMP441
    microphones through ALSA.

    Hardware format:
        16 kHz
        2 channels
        signed 16-bit PCM
    """

    def __init__(
        self,
        device="inmp441",
    ):
        self.sample_rate = SAMPLE_RATE
        self.channels = CAPTURE_CHANNELS
        self.sample_width_bytes = SAMPLE_WIDTH_BYTES

        self.device = device

        self.running = False
        self.lock = threading.Lock()

        self.stream = None
        self.pending_audio = np.empty(
            (0, self.channels),
            dtype=np.int16,
        )

        self.audio_queue = queue.Queue(
            maxsize=100
        )

    def start(self) -> None:
        """
        Open the ALSA microphone stream.
        """

        with self.lock:
            if self.running:
                return

            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break

            self.pending_audio = np.empty(
                (0, self.channels),
                dtype=np.int16,
            )    

            self.stream = sd.InputStream(
                device=self.device,
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype="int16",
                blocksize=0,
                latency="low",
                callback=self._audio_callback,
            )

            self.stream.start()

            self.running = True

    def stop(self) -> None:
        """
        Stop and close microphone capture.
        """

        with self.lock:
            if not self.running:
                return

            if self.stream is not None:
                self.stream.stop()
                self.stream.close()
                self.stream = None

            self.running = False

    def is_running(self) -> bool:
        with self.lock:
            return self.running

    def read(
        self,
        timeout: float = 1.0,
    ) -> np.ndarray:
        """
        Return exactly AUDIO_CHUNK_SAMPLES stereo samples.

        PortAudio may provide hardware blocks of different
        sizes. They are combined here into the fixed-size
        chunks required by AudioService.
        """

        if not self.is_running():
            raise RuntimeError(
                "Microphone is not running."
            )

        while (
            len(self.pending_audio)
            < AUDIO_CHUNK_SAMPLES
        ):
            try:
                audio = self.audio_queue.get(
                    timeout=timeout
                )

            except queue.Empty as error:
                raise RuntimeError(
                    "Timed out waiting for microphone audio."
                ) from error

            self.pending_audio = np.concatenate(
                (
                    self.pending_audio,
                    audio,
                ),
                axis=0,
            )

        result = self.pending_audio[
            :AUDIO_CHUNK_SAMPLES
        ].copy()

        self.pending_audio = self.pending_audio[
            AUDIO_CHUNK_SAMPLES:
        ]

        return result


    def to_mono(
        self,
        audio: np.ndarray,
    ) -> np.ndarray:
        """
        Convert two-channel int16 microphone audio
        into mono int16 audio for the AI models.
        """

        if audio.dtype != np.int16:
            raise ValueError(
                "Microphone audio must use np.int16."
            )

        if audio.ndim != 2:
            raise ValueError(
                "Stereo audio must have shape "
                "(samples, channels)."
            )

        if audio.shape[1] != self.channels:
            raise ValueError(
                f"Expected {self.channels} channels, "
                f"received {audio.shape[1]}."
            )

        left = audio[:, 0].astype(
            np.int32
        )

        right = audio[:, 1].astype(
            np.int32
        )

        mono = (
            (left + right) // 2
        )

        return mono.astype(
            np.int16
        )

    def _audio_callback(
        self,
        indata,
        frames,
        time_info,
        status,
    ) -> None:
        """
        Called automatically by PortAudio whenever
        a new microphone block is available.

        Keep this callback extremely lightweight.
        """

        try:
            self.audio_queue.put_nowait(
                indata.copy()
            )

        except queue.Full:
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                self.audio_queue.put_nowait(
                    indata.copy()
                )
            except queue.Full:
                pass