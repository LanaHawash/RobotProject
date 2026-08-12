from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

AUDIO_MODELS_DIR = PROJECT_ROOT / "models" / "audio"

VOSK_MODEL_PATH = (
    AUDIO_MODELS_DIR
    / "vosk"
    / "vosk-model-small-en-us-0.15"
)

OPENWAKEWORD_MODEL_DIR = (
    AUDIO_MODELS_DIR
    / "openwakeword"
)

YAMNET_MODEL_DIR = (
    AUDIO_MODELS_DIR
    / "yamnet"
)


SAMPLE_RATE = 16000

CAPTURE_CHANNELS = 2
MODEL_CHANNELS = 1

SAMPLE_WIDTH_BYTES = 2

BABY_CRY_LABEL = "Baby cry, infant cry"

CRY_WINDOW_SECONDS = 1.0
CRY_WINDOW_SAMPLES = int(
    SAMPLE_RATE * CRY_WINDOW_SECONDS
)

# Initial value only. We will tune this with real audio.
BABY_CRY_THRESHOLD = 0.30

AUDIO_CHUNK_SAMPLES =1280

COMMAND_LISTEN_SECONDS = 5.0

AUDIO_SOUNDS_DIR = (
    PROJECT_ROOT
    / "robot_project"
    / "audio"
    / "sounds"
)

BABY_LULLABY_PATHS = (
    AUDIO_SOUNDS_DIR
    / "twinkle-twinkle-little-star.mp3",

    AUDIO_SOUNDS_DIR
    / "baby-shark.mp3",
)