import wave
import numpy as np
import sounddevice as sd


SAMPLE_RATE = 16000
DURATION = 10
DEVICE = "inmp441"

print()
print("Recording for 10 seconds...")
print("Speak normally from about 1-2 meters away.")
print("Say 'Hey Robo' several times.")
print()

audio = sd.rec(
    int(DURATION * SAMPLE_RATE),
    samplerate=SAMPLE_RATE,
    channels=2,
    dtype="int16",
    device=DEVICE,
)

sd.wait()

print("Recording complete.")
print()


for channel in range(2):
    samples = audio[:, channel].astype(np.float32)

    peak = np.max(np.abs(samples))
    rms = np.sqrt(np.mean(samples ** 2))

    peak_dbfs = (
        20 * np.log10(peak / 32768.0)
        if peak > 0
        else -100.0
    )

    rms_dbfs = (
        20 * np.log10(rms / 32768.0)
        if rms > 0
        else -100.0
    )

    clipped = np.sum(np.abs(samples) >= 32767)
    clipped_percent = (
        clipped / len(samples)
    ) * 100

    print(
        f"Mic {channel}: "
        f"peak {peak_dbfs:.1f} dBFS, "
        f"RMS {rms_dbfs:.1f} dBFS, "
        f"clipped {clipped_percent:.3f}%"
    )

# Save microphone 0
with wave.open("mic_0.wav", "wb") as wav:
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(SAMPLE_RATE)
    wav.writeframes(
        audio[:, 0].copy().tobytes()
    )


# Save microphone 1
with wave.open("mic_1.wav", "wb") as wav:
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(SAMPLE_RATE)
    wav.writeframes(
        audio[:, 1].copy().tobytes()
    )


# Save stereo recording too
with wave.open("mics_stereo.wav", "wb") as wav:
    wav.setnchannels(2)
    wav.setsampwidth(2)
    wav.setframerate(SAMPLE_RATE)
    wav.writeframes(audio.tobytes())


print()
print("Saved:")
print("  mic_0.wav")
print("  mic_1.wav")
print("  mics_stereo.wav")