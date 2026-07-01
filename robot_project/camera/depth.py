import depthai as dai


def create_depth_pipeline():

    pipeline = dai.Pipeline()

    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.LEFT)

    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.RIGHT)

    stereo = pipeline.create(dai.node.StereoDepth)

    left.requestOutput().link(stereo.left)
    right.requestOutput().link(stereo.right)

    depth = stereo.depth

    return pipeline, depth