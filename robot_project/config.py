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