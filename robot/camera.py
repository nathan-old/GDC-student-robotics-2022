import time
from robot_module import RobotModule
from threading import Thread


class CameraManager(RobotModule):

    def __init__(self,
                 robot,
                 buffer_length=25 * 10,
                 target_fps=25,
                 debug_markers=True):
        super().__init__(
            "CameraManager",
            "Captures the image data from zoloto, provides the ai the data and sends it back to zoloto when needed",
            "1.0.0", "0.0.0", "1.0.0")

        self.robot = robot
        self.camera = robot.camera
        self._buffer = []
        self.buffer_length = buffer_length
        self.target_fps = target_fps
        self.wait_between_reads = 1.0 / target_fps
        self._top_frame = None
        self.frame_callbacks = []
        self.running = False
        self.debug_markers = debug_markers
        if (self.debug_markers):
            self.add_frame_callback(self.see_codes, every=self.target_fps)

    def add_frame_callback(self, callback, every=2):
        self.frame_callbacks.append((callback, every, 0))

    def remove_frame_callback(self, callback):
        self.frame_callbacks.remove(callback)

    def get_frame_from_camera(self):
        return self.camera.capture_frame()

    def see_codes(self):
        markers = yield self.camera.process_frame(self.top_frame)
        for marker in markers:
            print("{}: distance:{}, cartesian_pos: {}".format(
                marker.id, marker.dist, marker.cartesian_pos))

    def store_frame_in_buffer(self, frame):
        self._buffer.append(frame)
        self._top_frame = frame
        if len(self.buffer) > self.buffer_length:
            self.buffer.pop(0)

    @property
    def top_frame(self):
        return self._top_frame

    def read_wait(self):
        time.sleep(self.wait_between_reads)

    def shutdown(self):
        self.running = False

    def start_loop(self):
        self.running = True
        return Thread(self.loop()).start()

    def loop(self):
        while self.running:
            frame = self.get_frame_from_camera()
            self.store_frame_in_buffer(frame)
            self.callbacks()
            self.read_wait()
        print("Camera loop no longer running")

    def callbacks(self):
        for (cb, every, counter) in self.frame_callbacks:
            if counter % every == 0:
                cb(self.top_frame)
            counter += 1
