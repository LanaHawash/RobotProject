import depthai as dai

from robot_project.config import (
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
)


class CameraPipeline:

    def __init__(self):
        self.pipeline = dai.Pipeline()

        self.rgb_queue = None
        self.depth_queue = None

        self.rgb_camera = None
        self.left_camera = None
        self.right_camera = None
        self.stereo = None

    def create_rgb(self):
        self.rgb_camera = self.pipeline.create(
            dai.node.Camera
        ).build(dai.CameraBoardSocket.RGB)

        rgb_output = self.rgb_camera.requestOutput(
            size=(CAMERA_WIDTH, CAMERA_HEIGHT),
            fps=CAMERA_FPS,
        )

        self.rgb_queue = rgb_output.createOutputQueue()

    def create_depth(self):
        self.left_camera = self.pipeline.create(
            dai.node.Camera
        ).build(dai.CameraBoardSocket.LEFT)

        left_output = self.left_camera.requestOutput(
            size=(CAMERA_WIDTH, CAMERA_HEIGHT),
            fps=CAMERA_FPS,
        )

        self.right_camera = self.pipeline.create(
            dai.node.Camera
        ).build(dai.CameraBoardSocket.RIGHT)

        right_output = self.right_camera.requestOutput(
            size=(CAMERA_WIDTH, CAMERA_HEIGHT),
            fps=CAMERA_FPS,
        )

        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(True)
        self.stereo.setExtendedDisparity(False)

        # Make depth coordinates correspond to RGB coordinates.
        self.stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        # Make the aligned depth frame match the RGB frame size.
        self.stereo.setOutputSize(
            CAMERA_WIDTH,
            CAMERA_HEIGHT,
        )

        left_output.link(self.stereo.left)
        right_output.link(self.stereo.right)

        self.depth_queue = self.stereo.depth.createOutputQueue()

    def start(self):
        self.pipeline.start()

    def close(self):
        if self.pipeline.isRunning():
            self.pipeline.stop()

    def is_running(self):
        return self.pipeline.isRunning()

    def get_pipeline(self):
        return self.pipeline