from pathlib import Path
import cv2

IMAGE_DIR = Path("dataset/images/raw")
IMAGE_DIR.mkdir(parents=True, exist_ok=True)


def save_image(frame):

    count = len(list(IMAGE_DIR.glob("*.jpg")))

    filename = IMAGE_DIR / f"{count:04d}.jpg"

    cv2.imwrite(str(filename), frame)

    return filename.name


def image_count():

    return len(list(IMAGE_DIR.glob("*.jpg")))