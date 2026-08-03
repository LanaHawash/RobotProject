# Deployment Model

The robot requires a trained Ultralytics YOLO model at:

`models/oak/best.pt`

The model weights are installed manually on the Raspberry Pi and are not stored
in this Git repository.

Before starting the robot, verify the model with:

```bash
python -c "from robot_project.config import YOLO_MODEL_PATH; print(YOLO_MODEL_PATH); print(YOLO_MODEL_PATH.exists())"