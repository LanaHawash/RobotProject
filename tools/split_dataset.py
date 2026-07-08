from pathlib import Path
import random
import shutil

random.seed(42)

dataset = Path("dataset")

raw_images = dataset / "images" / "raw"
raw_labels = dataset / "labels" / "train"

train_images = dataset / "images" / "train"
valid_images = dataset / "images" / "valid"
test_images = dataset / "images" / "test"

train_labels = dataset / "labels" / "train"
valid_labels = dataset / "labels" / "valid"
test_labels = dataset / "labels" / "test"

for folder in [
    train_images,
    valid_images,
    test_images,
    valid_labels,
    test_labels,
]:
    folder.mkdir(parents=True, exist_ok=True)

images = list(raw_images.glob("*.jpg"))
images.sort()

random.shuffle(images)

n = len(images)

train_end = int(n * 0.8)
valid_end = int(n * 0.9)

train = images[:train_end]
valid = images[train_end:valid_end]
test = images[valid_end:]


def move_files(files, image_dst, label_dst):
    for image in files:
        label = raw_labels / (image.stem + ".txt")

        shutil.move(str(image), image_dst / image.name)

        if label.exists():
            shutil.move(str(label), label_dst / label.name)


move_files(train, train_images, train_labels)
move_files(valid, valid_images, valid_labels)
move_files(test, test_images, test_labels)

print("Done.")
print(f"Train: {len(train)}")
print(f"Valid: {len(valid)}")
print(f"Test : {len(test)}")