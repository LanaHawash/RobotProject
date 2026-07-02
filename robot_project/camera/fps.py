import time


class FPS:

    def __init__(self):
        self.start_time = time.time()
        self.frames = 0
        self.fps = 0.0

    def update(self):
        self.frames += 1

        elapsed = time.time() - self.start_time

        if elapsed >= 1.0:
            self.fps = self.frames / elapsed
            self.frames = 0
            self.start_time = time.time()

        return self.fps