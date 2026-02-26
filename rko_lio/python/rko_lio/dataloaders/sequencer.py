from typing import Literal

SensorKind = Literal["imu", "lidar"]


class LidarIMUSequencer:
    """
    Wraps a dataloader and yields data in processing order:
    LiDAR frames only when sufficient IMU coverage is available (latest
    IMU.time > LiDAR.end_time). Maintains internal buffers and returns
    data sequentially.

    Input format: ("imu", {time, accel, gyro}) or ("lidar", {start_time, end_time, scan, timestamps})
    Output format: same as above but sorted as per imu_time and lidar_end_time
    """

    def __init__(self, dataloader):
        self.dataloader = dataloader
        # Each: dict with keys 'time', 'accel', 'gyro'
        self.imu_buffer: list[dict] = []
        # Each: dict with keys 'start_time', 'end_time', 'scan', 'timestamps'
        self.lidar_buffer: list[dict] = []

    def __len__(self):
        return len(self.dataloader)

    def _buffers_ready(self):
        "if true, there's at least one valid lidar and imu data sequence"
        return (
            self.lidar_buffer
            and self.imu_buffer
            and self.imu_buffer[-1]["time"] > self.lidar_buffer[0]["end_time"]
        )

    def _read_next_from_wrapped_loader(self):
        "reads the next data point and appends to the appropriate buffer"
        try:
            kind, data = next(self.dataloader)
        except StopIteration:
            raise

        if kind == "imu":
            self.imu_buffer.append(data)
        elif kind == "lidar":
            self.lidar_buffer.append(data)

    def __iter__(self):
        while True:
            if self._buffers_ready():
                frame = self.lidar_buffer.pop(0)
                imus_to_process = [
                    imu for imu in self.imu_buffer if imu["time"] < frame["end_time"]
                ]
                for imu in imus_to_process:
                    yield ("imu", imu)

                yield ("lidar", frame)

                self.imu_buffer = [
                    imu for imu in self.imu_buffer if imu["time"] >= frame["end_time"]
                ]

            try:
                # buffers dont have valid data to process so
                self._read_next_from_wrapped_loader()
            except StopIteration:
                break
