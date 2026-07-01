import depthai as dai


class CameraPipeline:

    def __init__(self):

        self.pipeline = dai.Pipeline()

        self.rgb_queue = None
        self.depth_queue = None

    def create_rgb(self):

        camera = self.pipeline.create(dai.node.Camera).build()

        output = camera.requestOutput(
            size=(640, 480),
            fps=30
        )

        self.rgb_queue = output.createOutputQueue()

    def start(self):

        self.pipeline.start()

    def is_running(self):

        return self.pipeline.isRunning()