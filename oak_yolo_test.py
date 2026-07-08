import cv2
import depthai as dai
from ultralytics import YOLO

model = YOLO("runs/detect/models/toy_detector-2/weights/best.pt")

pipeline = dai.Pipeline()

cam = pipeline.createColorCamera()
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

xout = pipeline.createXLinkOut()
xout.setStreamName("video")
cam.preview.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=4, blocking=False)

    while True:
        frame = q.get().getCvFrame()

        results = model(frame, conf=0.5)

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                name = model.names[cls_id]
                conf = float(box.conf[0])
                print(name, conf)

        # no imshow because your Pi has no DISPLAY

