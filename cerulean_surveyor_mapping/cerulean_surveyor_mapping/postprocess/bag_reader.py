from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

from cerulean_surveyor_mapping.postprocess.pointcloud_decode import decode_pointcloud2

FIELD_NAMES = ('x', 'y', 'z', 'lat', 'lon')


def read_pointcloud_map_points(bag_path, topic):
    """Read every message on `topic` (a georeferenced surveyor/pointcloud_map
    topic) from a rosbag2 bag and concatenate into one dict of numpy arrays,
    one entry per field in FIELD_NAMES.
    """
    per_field = {name: [] for name in FIELD_NAMES}

    with AnyReader([Path(bag_path)]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            raise ValueError(f"Topic {topic} not found in bag {bag_path}")

        for connection, _timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            decoded = decode_pointcloud2(msg, FIELD_NAMES)
            for name in FIELD_NAMES:
                per_field[name].append(decoded[name])

    if not per_field['x']:
        raise ValueError(f"No messages found on {topic} in {bag_path}")

    return {name: np.concatenate(arrays) for name, arrays in per_field.items()}
