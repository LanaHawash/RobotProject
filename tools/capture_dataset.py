import cv2
from pathlib import Path

from robot_project.camera.pipeline import CameraPipeline


CLASS_NAME = "toy_car"

SAVE_DIR = Path(f"dataset/images/train/{CLASS_NAME}")
SAVE_DIR.mkdir(parents=True, exist_ok=True)


camera = CameraPipeline()
camera.create_rgb()
camera.start()


counter = len(list(SAVE_DIR.glob("*.jpg")))


print("Press S to save an image.")
print("Press Q to quit.")


while camera.is_running():

    frame = camera.rgb_queue.get().getCvFrame()

    cv2.imshow("Dataset Capture", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("s"):

        filename = SAVE_DIR / f"{counter:04d}.jpg"

        cv2.imwrite(str(filename), frame)

        print(f"Saved {filename}")

        counter += 1

    elif key == ord("q"):

        break


cv2.destroyAllWindows()