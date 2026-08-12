import math
import wave
import struct
from pathlib import Path

out = Path(
    "robot_project/audio/sounds/cute_baby_lullaby.wav"
)

sample_rate = 44100
volume = 0.22

melody = [
    (261.63, 0.5),
    (329.63, 0.5),
    (392.00, 0.75),
    (329.63, 0.5),
    (293.66, 0.5),
    (349.23, 0.5),
    (440.00, 0.75),
    (349.23, 0.5),
    (329.63, 0.5),
    (392.00, 0.5),
    (523.25, 0.75),
    (392.00, 0.5),
    (293.66, 0.5),
    (329.63, 0.5),
    (261.63, 1.0),
]

samples = []

for frequency, duration in melody:
    count = int(sample_rate * duration)

    for i in range(count):
        t = i / sample_rate

        attack = min(
            1.0,
            i / max(1, int(0.04 * sample_rate)),
        )

        release = min(
            1.0,
            (count - i)
            / max(1, int(0.08 * sample_rate)),
        )

        envelope = min(
            attack,
            release,
        )

        sample = (
            math.sin(
                2
                * math.pi
                * frequency
                * t
            )
            + 0.22
            * math.sin(
                2
                * math.pi
                * frequency
                * 2
                * t
            )
        )

        value = int(
            32767
            * volume
            * envelope
            * sample
            / 1.22
        )

        samples.append(value)

    samples.extend(
        [0]
        * int(sample_rate * 0.03)
    )

with wave.open(str(out), "w") as wav:
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(sample_rate)

    wav.writeframes(
        b"".join(
            struct.pack(
                "<h",
                max(
                    -32768,
                    min(32767, sample),
                ),
            )
            for sample in samples
        )
    )

print(f"Created: {out}")