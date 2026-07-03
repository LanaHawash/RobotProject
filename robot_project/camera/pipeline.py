import depthai as dai


class CameraPipeline:

    def __init__(self):

        self.pipeline = dai.Pipeline()

        self.rgb_queue = None
        self.depth_queue = None

        self.rgb = None
        self.left = None
        self.right = None
        self.stereo = None

    def create_rgb(self):

        camera = self.pipeline.create(dai.node.Camera).build()

        output = camera.requestOutput(
            size=(640, 480),
            fps=30
        )

        self.rgb_queue = output.createOutputQueue()

    def create_depth(self):

        self.left = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.LEFT
        )

        left_out = self.left.requestOutput(
            size=(640, 480),
            fps=30
        )

        self.right = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.RIGHT
        )

        right_out = self.right.requestOutput(
            size=(640, 480),
            fps=30
        )

        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(True)
        self.stereo.setExtendedDisparity(False)
      

        left_out.link(self.stereo.left)
        right_out.link(self.stereo.right)

        self.depth_queue = self.stereo.depth.createOutputQueue()

    def get_pipeline(self):
        return self.pipeline
    

    def debug_depth(self):
        if self.depth_queue is None:
            print("Depth not initialized")
            return

        print("Depth stream started...")

        while self.is_running():
         frame = self.depth_queue.get().getCvFrame()
         print("Depth frame shape:", frame.shape)
         
    def start(self):
        self.pipeline.start()

    def is_running(self):
        return self.pipeline.isRunning()