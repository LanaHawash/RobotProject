import depthai as dai


class CameraDevice:

    def __init__(self, pipeline):
        self.pipeline = pipeline
        self.device = None

    def start(self):
        self.device = dai.Device()
        self.device.startPipeline(self.pipeline)

    def close(self):
        if self.device is not None:
            self.device.close()