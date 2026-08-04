from pathlib import Path


# Absolute path to the project root.
PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Camera configuration.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# YOLO configuration.
YOLO_MODEL_PATH = PROJECT_ROOT / "models" / "oak" / "best.pt"
YOLO_CONFIDENCE_THRESHOLD = 0.25

# Web server configuration.
WEB_HOST = "0.0.0.0"
WEB_PORT = 5000

# Bins color configuration.
DESTINATION_BIN_COLORS = {
    "animal": "yellow",
    "toy_car": "red",
    "building_block": "blue",
    "discharge": "black",
}

BIN_COLOR_HSV_RANGES = {
    "yellow": {
        "lower": (20, 100, 100),
        "upper": (35, 255, 255),
    },
    "blue": {
        "lower": (90, 80, 70),
        "upper": (135, 255, 255),
    },

    "black": {
        # Black has no reliable hue. Detect it using its low
        # brightness value instead.
        "lower": (0, 0, 0),
        "upper": (179, 255, 60),
    },
}

RED_BIN_HSV_RANGES = [
    {
        "lower": (0, 100, 80),
        "upper": (10, 255, 255),
    },
    {
        "lower": (170, 100, 80),
        "upper": (179, 255, 255),
    },
]

MIN_BIN_COLOR_AREA = 1500