import csv

from proto.visualization_pb2 import PassFeatures
import os
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.constants import PassFeaturesConstants


class PassFeaturesTracker:
    def __init__(
        self,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ):
        self._friendly_colour_yellow = friendly_colour_yellow

        self.pass_features_buffer = ThreadSafeBuffer(buffer_size, PassFeatures)

        file_name = (
            PassFeaturesConstants.YELLOW_PASS_FEATURES_FILE
            if friendly_colour_yellow
            else PassFeaturesConstants.BLUE_PASS_FEATURES_FILE
        )

        os.makedirs(PassFeaturesConstants.PASS_FEATURES_DIR, exist_ok=True)
        file = open(
            os.path.join(PassFeaturesConstants.PASS_FEATURES_DIR, self.file_name),
            "a",
            buffering=1,
        )
        self.csv_writer = csv.writer(file)

    def refresh(self):
        features_msg = self.pass_features_buffer.get(block=False)

        if features_msg is None:
            return

        data_point = ",".join(
            [
                features_msg.passer_point.x_meters,
                features_msg.passer_point.y_meters,
                features_msg.receiver_point.x_meters,
                features_msg.receiver_point.y_meters,
                features_msg.pass_speed_m_per_s,
                features_msg.ball_position.x_meters,
                features_msg.ball_position.y_meters,
            ]
            + [
                coord
                for point in features_msg.friendly_positions
                for coord in (point.x_meters, point.y_meters)
            ]
            + [
                coord
                for point in features_msg.enemy_positions
                for coord in (point.x_meters, point.y_meters)
            ]
            + [features_msg.score]
        )

        self.csv_writer.writerow(data_point)

    def __del__(self):
        if hasattr(self, "file") and not self.file.closed:
            self.file.close()
